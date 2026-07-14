"""
UKF Processor - Enhanced Reliability
- Xử lý UKF với độ tin cậy cao
- Tự động phục hồi khi dữ liệu xấu
"""

import json
import math
import os
import sys
from datetime import datetime
from typing import List, Dict, Tuple

# ============================
# CẤU HÌNH
# ============================
RAW_DATA_FILE = "locations.json"
FILTERED_DATA_FILE = "filtered_locations.json"
MAX_RECORDS = 10000

# ============================
# UKF PARAMETERS
# ============================
DT = 0.5
ALPHA = 0.1
BETA = 2.0
KAPPA = 0.0

# ============================
# NOISE PARAMETERS - TỐI ƯU
# ============================
Q_POS = 0.1      # Process noise - position
Q_VEL = 0.5      # Process noise - velocity
R_GPS = 2.0      # Measurement noise - GPS

# ============================
# THAM SỐ PHÁT HIỆN
# ============================
STATIONARY_THRESHOLD = 0.000005
MOVING_AVG_WINDOW = 7
VIBRATION_THRESHOLD = 0.3
MIN_CONFIDENCE = 0.30
MAX_CONFIDENCE = 0.95
DEFAULT_CONFIDENCE = 0.50

# ============================
# CLASS MOVING AVERAGE
# ============================
class MovingAverageFilter:
    def __init__(self, window_size=MOVING_AVG_WINDOW):
        self.window_size = window_size
        self.buffer = []
    
    def filter(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)
    
    def reset(self):
        self.buffer = []

# ============================
# CLASS UKF
# ============================
class UnscentedKalmanFilter:
    def __init__(self, dt=DT):
        self.dt = dt
        self.n = 4
        self.m = 2
        
        self.x = [0.0, 0.0, 0.0, 0.0]
        self.P = [
            [10.0, 0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0, 10.0]
        ]
        
        self.Q = [
            [Q_POS, 0.0, 0.0, 0.0],
            [0.0, Q_POS, 0.0, 0.0],
            [0.0, 0.0, Q_VEL, 0.0],
            [0.0, 0.0, 0.0, Q_VEL]
        ]
        
        self.R = [
            [R_GPS, 0.0],
            [0.0, R_GPS]
        ]
        
        self.lambda_ = ALPHA**2 * (self.n + KAPPA) - self.n
        self.sqrt_lambda = math.sqrt(self.n + self.lambda_)
        
        self.Wm = [0.0] * (2 * self.n + 1)
        self.Wc = [0.0] * (2 * self.n + 1)
        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.Wc[0] = self.Wm[0] + (1 - ALPHA**2 + BETA)
        for i in range(1, 2 * self.n + 1):
            self.Wm[i] = 1 / (2 * (self.n + self.lambda_))
            self.Wc[i] = self.Wm[i]
        
        self.initialized = False
        self.ref_lat = 0
        self.ref_lng = 0
        self.confidence = DEFAULT_CONFIDENCE
        self.uncertainty = 100.0
        
        self.lat_filter = MovingAverageFilter(MOVING_AVG_WINDOW)
        self.lng_filter = MovingAverageFilter(MOVING_AVG_WINDOW)
        
        self.last_lat = 0
        self.last_lng = 0
        self.is_stationary = False
        self.stationary_count = 0
        self.motion_threshold = STATIONARY_THRESHOLD
        
        self.vibration = 0
        self.satellites = 0
        self.speed = 0
        self.innovation_history = []
        self.smoothed_confidence = DEFAULT_CONFIDENCE
    
    def update_vibration(self, vib_value):
        if vib_value is not None:
            self.vibration = vib_value
    
    def update_satellites(self, sats):
        if sats is not None:
            self.satellites = sats
    
    def update_speed(self, speed):
        if speed is not None:
            self.speed = speed
    
    def calculate_dynamic_confidence(self):
        """Tính confidence dựa trên nhiều yếu tố"""
        conf = DEFAULT_CONFIDENCE
        
        # Số vệ tinh
        if self.satellites >= 8:
            conf += 0.25
        elif self.satellites >= 6:
            conf += 0.15
        elif self.satellites >= 4:
            conf += 0.05
        elif self.satellites < 3:
            conf -= 0.15
        
        # Độ rung
        if self.vibration < 0.1:
            conf += 0.15
        elif self.vibration < 0.3:
            conf += 0.05
        elif self.vibration > 1.0:
            conf -= 0.10
        elif self.vibration > 2.0:
            conf -= 0.20
        
        # Uncertainty
        if self.uncertainty < 3.0:
            conf += 0.15
        elif self.uncertainty < 5.0:
            conf += 0.05
        elif self.uncertainty > 15.0:
            conf -= 0.10
        
        # Trạng thái đứng yên
        if self.is_stationary:
            conf += 0.10
            if self.vibration < 0.1:
                conf += 0.10
        
        # Innovation
        if len(self.innovation_history) > 0:
            avg_innovation = sum(self.innovation_history[-10:]) / min(len(self.innovation_history), 10)
            if avg_innovation < 2.0:
                conf += 0.05
            elif avg_innovation > 10.0:
                conf -= 0.10
        
        # Giới hạn
        conf = max(MIN_CONFIDENCE, min(MAX_CONFIDENCE, conf))
        
        # Làm mượt
        alpha = 0.7
        self.smoothed_confidence = alpha * conf + (1 - alpha) * self.smoothed_confidence
        
        return self.smoothed_confidence
    
    # ========== MA TRẬN ==========
    def mat_add(self, A, B):
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def mat_sub(self, A, B):
        return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def mat_mul(self, A, B):
        rows, cols, inner = len(A), len(B[0]), len(B)
        result = [[0.0 for _ in range(cols)] for _ in range(rows)]
        for i in range(rows):
            for j in range(cols):
                for k in range(inner):
                    result[i][j] += A[i][k] * B[k][j]
        return result
    
    def mat_scale(self, A, scalar):
        return [[A[i][j] * scalar for j in range(len(A[0]))] for i in range(len(A))]
    
    def mat_inv_2x2(self, A):
        det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
        if abs(det) < 1e-10:
            return [[1.0, 0.0], [0.0, 1.0]]
        return [
            [A[1][1] / det, -A[0][1] / det],
            [-A[1][0] / det, A[0][0] / det]
        ]
    
    def outer_product(self, vec1, vec2):
        return [[vec1[i] * vec2[j] for j in range(len(vec2))] for i in range(len(vec1))]
    
    def vec_sub(self, v1, v2):
        return [v1[i] - v2[i] for i in range(len(v1))]
    
    def vec_add(self, v1, v2):
        return [v1[i] + v2[i] for i in range(len(v1))]
    
    def vec_scale(self, v, scalar):
        return [v[i] * scalar for i in range(len(v))]
    
    # ========== TỌA ĐỘ ==========
    def lat_to_meters(self, lat, ref_lat):
        return (lat - ref_lat) * 111320.0
    
    def lng_to_meters(self, lng, ref_lng, lat):
        return (lng - ref_lng) * 111320.0 * math.cos(math.radians(lat))
    
    def meters_to_lat(self, meters, ref_lat):
        return ref_lat + meters / 111320.0
    
    def meters_to_lng(self, meters, ref_lng, ref_lat):
        return ref_lng + meters / (111320.0 * math.cos(math.radians(ref_lat)))
    
    # ========== UKF ==========
    def initialize(self, lat, lng):
        self.ref_lat = lat
        self.ref_lng = lng
        self.x = [0.0, 0.0, 0.0, 0.0]
        self.P = [
            [5.0, 0.0, 0.0, 0.0],
            [0.0, 5.0, 0.0, 0.0],
            [0.0, 0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, 5.0]
        ]
        self.initialized = True
        self.confidence = 0.50
        self.uncertainty = 5.0
        self.smoothed_confidence = 0.50
        self.stationary_count = 0
        
        self.lat_filter = MovingAverageFilter(MOVING_AVG_WINDOW)
        self.lng_filter = MovingAverageFilter(MOVING_AVG_WINDOW)
        self.last_lat = lat
        self.last_lng = lng
        
        print(f"[UKF] Initialized at ({lat:.6f}, {lng:.6f})")
    
    def state_transition(self, x):
        return [
            x[0] + x[2] * self.dt,
            x[1] + x[3] * self.dt,
            x[2],
            x[3]
        ]
    
    def measurement_function(self, x):
        return [x[0], x[1]]
    
    def cholesky_4x4(self, A):
        L = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(i + 1):
                if i == j:
                    sum_sq = sum(L[i][k] ** 2 for k in range(i))
                    L[i][i] = math.sqrt(max(A[i][i] - sum_sq, 1e-6))
                else:
                    sum_prod = sum(L[i][k] * L[j][k] for k in range(j))
                    L[i][j] = (A[i][j] - sum_prod) / L[j][j]
        return L
    
    def generate_sigma_points(self):
        L = self.cholesky_4x4(self.P)
        sigma_points = [self.x.copy()]
        for i in range(self.n):
            plus = [self.x[j] + self.sqrt_lambda * L[i][j] for j in range(self.n)]
            minus = [self.x[j] - self.sqrt_lambda * L[i][j] for j in range(self.n)]
            sigma_points.append(plus)
            sigma_points.append(minus)
        return sigma_points
    
    def predict(self):
        if not self.initialized:
            return
        
        sigma_points = self.generate_sigma_points()
        sigma_points_pred = [self.state_transition(sp) for sp in sigma_points]
        
        self.x = [0.0] * self.n
        for i in range(len(sigma_points_pred)):
            self.x = self.vec_add(self.x, self.vec_scale(sigma_points_pred[i], self.Wm[i]))
        
        self.P = [[0.0] * self.n for _ in range(self.n)]
        for i in range(len(sigma_points_pred)):
            diff = self.vec_sub(sigma_points_pred[i], self.x)
            outer = self.outer_product(diff, diff)
            scaled = self.mat_scale(outer, self.Wc[i])
            self.P = self.mat_add(self.P, scaled)
        
        self.P = self.mat_add(self.P, self.Q)
    
    def update(self, z):
        if not self.initialized:
            return
        
        sigma_points = self.generate_sigma_points()
        z_pred = [self.measurement_function(sp) for sp in sigma_points]
        
        z_mean = [0.0] * self.m
        for i in range(len(z_pred)):
            z_mean = self.vec_add(z_mean, self.vec_scale(z_pred[i], self.Wm[i]))
        
        S = [[0.0] * self.m for _ in range(self.m)]
        for i in range(len(z_pred)):
            diff = self.vec_sub(z_pred[i], z_mean)
            outer = self.outer_product(diff, diff)
            scaled = self.mat_scale(outer, self.Wc[i])
            S = self.mat_add(S, scaled)
        
        S = self.mat_add(S, self.R)
        
        Pxz = [[0.0] * self.m for _ in range(self.n)]
        for i in range(len(sigma_points)):
            diff_state = self.vec_sub(sigma_points[i], self.x)
            diff_meas = self.vec_sub(z_pred[i], z_mean)
            outer = self.outer_product(diff_state, diff_meas)
            scaled = self.mat_scale(outer, self.Wc[i])
            Pxz = self.mat_add(Pxz, scaled)
        
        S_inv = self.mat_inv_2x2(S)
        K = self.mat_mul(Pxz, S_inv)
        
        innovation = self.vec_sub(z, z_mean)
        
        # Lưu innovation
        innovation_norm = math.sqrt(innovation[0]**2 + innovation[1]**2)
        self.innovation_history.append(innovation_norm)
        if len(self.innovation_history) > 50:
            self.innovation_history.pop(0)
        
        self.x = self.vec_add(self.x, [K[i][0] * innovation[0] + K[i][1] * innovation[1] for i in range(self.n)])
        
        KH = [[0.0] * self.n for _ in range(self.n)]
        for i in range(self.n):
            for j in range(self.n):
                KH[i][j] = K[i][0] * (1 if j == 0 else 0) + K[i][1] * (1 if j == 1 else 0)
        
        I_KH = [[0.0] * self.n for _ in range(self.n)]
        for i in range(self.n):
            for j in range(self.n):
                I_KH[i][j] = (1.0 if i == j else 0.0) - KH[i][j]
        
        self.P = self.mat_mul(I_KH, self.P)
        
        self.uncertainty = math.sqrt(self.P[0][0] + self.P[1][1]) / 2.0
        self.confidence = self.calculate_dynamic_confidence()
    
    def get_filtered_position(self):
        if not self.initialized:
            return 0, 0
        
        lat = self.meters_to_lat(self.x[0], self.ref_lat)
        lng = self.meters_to_lng(self.x[1], self.ref_lng, self.ref_lat)
        
        dx = abs(lat - self.last_lat)
        dy = abs(lng - self.last_lng)
        
        is_vibrating = self.vibration > VIBRATION_THRESHOLD
        
        if dx < self.motion_threshold and dy < self.motion_threshold and not is_vibrating:
            self.stationary_count += 1
            if self.stationary_count >= 3:
                self.is_stationary = True
                filtered_lat = self.lat_filter.filter(lat)
                filtered_lng = self.lng_filter.filter(lng)
            else:
                filtered_lat = lat
                filtered_lng = lng
        else:
            self.stationary_count = 0
            self.is_stationary = False
            self.lat_filter = MovingAverageFilter(3)
            self.lng_filter = MovingAverageFilter(3)
            filtered_lat = lat
            filtered_lng = lng
        
        self.last_lat = lat
        self.last_lng = lng
        
        return filtered_lat, filtered_lng

# ============================
# XỬ LÝ CHÍNH
# ============================
def load_raw_data() -> List[Dict]:
    if not os.path.exists(RAW_DATA_FILE):
        return []
    try:
        with open(RAW_DATA_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except Exception as e:
        print(f"[ERROR] Cannot read raw data: {e}")
        return []

def save_filtered_data(data: List[Dict]) -> bool:
    try:
        if len(data) > MAX_RECORDS:
            data = data[-MAX_RECORDS:]
        with open(FILTERED_DATA_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        print(f"[ERROR] Cannot save filtered data: {e}")
        return False

def process_raw_data():
    print("[UKF] Processing raw data...")
    print(f"[UKF] Confidence range: {MIN_CONFIDENCE:.0%} - {MAX_CONFIDENCE:.0%}")
    print()
    
    raw_data = load_raw_data()
    if not raw_data:
        print("[UKF] No raw data found")
        return
    
    print(f"[UKF] Found {len(raw_data)} raw records")
    
    ukf = UnscentedKalmanFilter()
    filtered_data = []
    first_fix = True
    
    for i, record in enumerate(raw_data):
        lat = record.get('lat', 0)
        lng = record.get('lng', 0)
        vibration = record.get('vibration', 0)
        sats = record.get('sats', 0)
        speed = record.get('speed', 0)
        
        if abs(lat) < 0.0001 or abs(lng) < 0.0001:
            continue
        
        ukf.update_vibration(vibration)
        ukf.update_satellites(sats)
        ukf.update_speed(speed)
        
        if first_fix:
            ukf.initialize(lat, lng)
            first_fix = False
            filtered_record = {
                "raw_lat": lat, "raw_lng": lng,
                "filtered_lat": lat, "filtered_lng": lng,
                "final_lat": lat, "final_lng": lng,
                "uncertainty": ukf.uncertainty,
                "confidence": ukf.confidence,
                "vibration": vibration,
                "is_stationary": False,
                "sats": sats,
                "datetime": record.get('datetime', datetime.now().isoformat()),
                "filter_type": "UKF_Enhanced"
            }
            filtered_data.append(filtered_record)
            continue
        
        zx = ukf.lat_to_meters(lat, ukf.ref_lat)
        zy = ukf.lng_to_meters(lng, ukf.ref_lng, lat)
        z = [zx, zy]
        
        ukf.predict()
        ukf.update(z)
        
        filtered_lat, filtered_lng = ukf.get_filtered_position()
        
        conf = ukf.confidence
        raw_weight = 0.3 + (1.0 - conf) * 0.4
        filtered_weight = 0.7 - (1.0 - conf) * 0.4
        
        final_lat = lat * raw_weight + filtered_lat * filtered_weight
        final_lng = lng * raw_weight + filtered_lng * filtered_weight
        
        filtered_record = {
            "raw_lat": lat, "raw_lng": lng,
            "filtered_lat": filtered_lat, "filtered_lng": filtered_lng,
            "final_lat": final_lat, "final_lng": final_lng,
            "uncertainty": ukf.uncertainty,
            "confidence": ukf.confidence,
            "is_stationary": ukf.is_stationary,
            "vibration": vibration,
            "sats": sats,
            "raw_weight": raw_weight,
            "filtered_weight": filtered_weight,
            "datetime": record.get('datetime', datetime.now().isoformat()),
            "filter_type": "UKF_Enhanced"
        }
        filtered_data.append(filtered_record)
        
        if i % 10 == 0:
            status = "STATIONARY" if ukf.is_stationary else "MOVING"
            conf_pct = ukf.confidence * 100
            print(f"[UKF] {i+1}/{len(raw_data)} | {status} | Conf: {conf_pct:.1f}% | Vib: {vibration:.3f}m/s² | Sats: {sats}")
    
    if filtered_data:
        save_filtered_data(filtered_data)
        last = filtered_data[-1]
        print(f"\n[UKF] Complete! {len(filtered_data)} records saved")
        print(f"[UKF] Final: ({last['final_lat']:.7f}, {last['final_lng']:.7f})")
        print(f"[UKF] Confidence: {last['confidence']:.1%}")
        print(f"[UKF] Vibration: {last.get('vibration', 0):.3f} m/s²")
        print(f"[UKF] Satellites: {last.get('sats', 0)}")
    else:
        print("[UKF] No filtered data generated")

if __name__ == "__main__":
    print("\n" + "="*50)
    print("   UKF PROCESSOR - ENHANCED")
    print("="*50 + "\n")
    
    try:
        process_raw_data()
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
    
    print("\n[UKF] Done!")
    sys.stdout.flush()