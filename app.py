"""
GPS Tracker Server - Enhanced Reliability
- Xử lý dữ liệu từ ESP32
- Tính vibration từ biến thiên gia tốc
- Tự động phục hồi khi lỗi
"""

from flask import Flask, request, render_template, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
from datetime import datetime
import json
import os
import logging
import threading
import subprocess
import sys
import math
from collections import deque
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gps_tracker_secret'
CORS(app)

socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Cấu hình logging chi tiết
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ============================
# CẤU HÌNH
# ============================
RAW_DATA_FILE = "locations.json"
FILTERED_DATA_FILE = "filtered_locations.json"
MAX_RECORDS = 10000

# ============================
# BUFFER TÍNH ĐỘ RUNG
# ============================
VIBRATION_BUFFER_SIZE = 30
accel_buffer = deque(maxlen=VIBRATION_BUFFER_SIZE)
prev_accel = None
vibration_smooth = 0

# ============================
# HÀM TÍNH ĐỘ RUNG - CẢI TIẾN
# ============================
def calculate_vibration(accel_x, accel_y, accel_z):
    """
    Tính độ rung bằng biến thiên gia tốc theo thời gian
    Công thức: vibration = RMS(Δa)
    """
    global prev_accel, vibration_smooth, accel_buffer
    
    current = (accel_x, accel_y, accel_z)
    accel_buffer.append(current)
    
    if prev_accel is None or len(accel_buffer) < 2:
        prev_accel = current
        return 0.0
    
    # === BIẾN THIÊN TỨC THỜI ===
    dx = accel_x - prev_accel[0]
    dy = accel_y - prev_accel[1]
    dz = accel_z - prev_accel[2]
    delta_magnitude = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    # === RMS BIẾN THIÊN ===
    if len(accel_buffer) >= 3:
        sum_sq = 0
        count = 0
        for i in range(2, len(accel_buffer)):
            dx2 = accel_buffer[i][0] - accel_buffer[i-1][0]
            dy2 = accel_buffer[i][1] - accel_buffer[i-1][1]
            dz2 = accel_buffer[i][2] - accel_buffer[i-2][2]  # Lấy khoảng cách 2 bước
            sum_sq += dx2*dx2 + dy2*dy2 + dz2*dz2
            count += 1
        
        rms_vibration = math.sqrt(sum_sq / max(count, 1)) if count > 0 else 0
    else:
        rms_vibration = delta_magnitude
    
    # === KẾT HỢP VÀ LÀM MƯỢT ===
    raw_vibration = 0.5 * delta_magnitude + 0.5 * rms_vibration
    
    # Lọc low-pass
    alpha = 0.3
    vibration_smooth = alpha * raw_vibration + (1 - alpha) * vibration_smooth
    
    prev_accel = current
    
    # Giới hạn
    return min(max(vibration_smooth, 0), 20.0)

# ============================
# HÀM PHÂN LOẠI ĐỘ RUNG
# ============================
def classify_vibration(vibration):
    if vibration < 0.01:
        return "Không cảm nhận", "🟢"
    elif vibration < 0.1:
        return "Rất nhẹ", "🟢"
    elif vibration < 0.3:
        return "Nhẹ", "🟡"
    elif vibration < 0.8:
        return "Trung bình", "🟠"
    elif vibration < 2.0:
        return "Mạnh", "🔴"
    elif vibration < 5.0:
        return "Rất mạnh", "🔴"
    else:
        return "Thảm khốc", "🔥"

def get_earthquake_equivalent(vibration):
    if vibration < 0.01:
        return "Cấp 0: Không ghi nhận"
    elif vibration < 0.1:
        return "Cấp 1-2: Rất nhẹ"
    elif vibration < 0.3:
        return "Cấp 3: Nhẹ"
    elif vibration < 0.8:
        return "Cấp 4: Trung bình"
    elif vibration < 2.0:
        return "Cấp 5: Mạnh"
    elif vibration < 5.0:
        return "Cấp 6: Rất mạnh"
    else:
        return "Cấp 7+: Thảm khốc"

# ============================
# BIẾN TOÀN CỤC
# ============================
latest_raw = {
    "lat": None, "lng": None, "roll": None, "pitch": None,
    "accel_x": None, "accel_y": None, "accel_z": None,
    "vibration": None, "vibration_class": None,
    "earthquake_level": None, "speed": None, "sats": None,
    "seq": None, "status": "Chưa có dữ liệu"
}

latest_filtered = {
    "lat": None, "lng": None, "uncertainty": None,
    "confidence": None, "vibration": None,
    "vibration_class": None, "earthquake_level": None,
    "status": "Chưa có dữ liệu"
}

history_filtered = []

# ============================
# HÀM ĐỌC/GHI JSON
# ============================
def load_raw_data():
    if not os.path.exists(RAW_DATA_FILE):
        with open(RAW_DATA_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f)
        return []
    
    try:
        with open(RAW_DATA_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except:
        return []

def save_raw_data(data):
    try:
        if len(data) > MAX_RECORDS:
            data = data[-MAX_RECORDS:]
        with open(RAW_DATA_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        logger.error(f"Lỗi ghi raw: {e}")
        return False

def load_filtered_data():
    if not os.path.exists(FILTERED_DATA_FILE):
        with open(FILTERED_DATA_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f)
        return []
    
    try:
        with open(FILTERED_DATA_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except:
        return []

def save_filtered_data(data):
    try:
        if len(data) > MAX_RECORDS:
            data = data[-MAX_RECORDS:]
        with open(FILTERED_DATA_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        logger.error(f"Lỗi ghi filtered: {e}")
        return False

def append_raw_record(data):
    records = load_raw_data()
    
    accel_x = data.get('accel_x', 0)
    accel_y = data.get('accel_y', 0)
    accel_z = data.get('accel_z', 0)
    
    vibration = calculate_vibration(accel_x, accel_y, accel_z)
    vibration_class, icon = classify_vibration(vibration)
    earthquake_level = get_earthquake_equivalent(vibration)
    
    record = {
        "type": "raw",
        "lat": data.get('lat', 0),
        "lng": data.get('lng', 0),
        "roll": data.get('roll', 0),
        "pitch": data.get('pitch', 0),
        "accel_x": accel_x,
        "accel_y": accel_y,
        "accel_z": accel_z,
        "vibration": vibration,
        "vibration_class": vibration_class,
        "vibration_icon": icon,
        "earthquake_level": earthquake_level,
        "speed": data.get('speed', 0),
        "sats": data.get('sats', 0),
        "seq": data.get('seq', 0),
        "timestamp": data.get('timestamp', datetime.now().isoformat()),
        "datetime": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    records.append(record)
    return save_raw_data(records)

def clear_all_data():
    global history_filtered, latest_filtered, latest_raw, accel_buffer, prev_accel, vibration_smooth
    
    try:
        save_raw_data([])
        save_filtered_data([])
        history_filtered = []
        accel_buffer.clear()
        prev_accel = None
        vibration_smooth = 0
        
        latest_filtered = {
            "lat": None, "lng": None, "uncertainty": None,
            "confidence": None, "vibration": None,
            "vibration_class": None, "earthquake_level": None,
            "status": "Chưa có dữ liệu"
        }
        latest_raw = {
            "lat": None, "lng": None, "roll": None,
            "pitch": None, "accel_x": None, "accel_y": None,
            "accel_z": None, "vibration": None,
            "vibration_class": None, "earthquake_level": None,
            "speed": None, "sats": None, "seq": None,
            "status": "Chưa có dữ liệu"
        }
        logger.info("🗑️ All data cleared")
        return True
    except Exception as e:
        logger.error(f"Lỗi xóa dữ liệu: {e}")
        return False

# ============================
# UKF PROCESSOR
# ============================
def run_ukf_processor():
    try:
        logger.info("🔄 Running UKF Processor...")
        
        result = subprocess.run(
            [sys.executable, "ukf_processor.py"],
            capture_output=True,
            text=True,
            encoding='utf-8',
            errors='replace',
            timeout=15
        )
        
        if result.stdout:
            for line in result.stdout.split('\n'):
                if line.strip():
                    logger.info(f"UKF: {line}")
        
        if result.stderr:
            for line in result.stderr.split('\n'):
                if line.strip():
                    logger.warning(f"UKF Error: {line}")
        
        if result.returncode == 0:
            logger.info("✅ UKF Processor completed")
            update_filtered_data()
        else:
            logger.error(f"❌ UKF failed with code {result.returncode}")
            
    except subprocess.TimeoutExpired:
        logger.error("❌ UKF timeout")
    except Exception as e:
        logger.error(f"❌ UKF exception: {e}")

def update_filtered_data():
    global latest_filtered, history_filtered
    
    filtered = load_filtered_data()
    
    if filtered:
        last = filtered[-1]
        vibration = last.get('vibration', 0)
        vibration_class, icon = classify_vibration(vibration)
        earthquake_level = get_earthquake_equivalent(vibration)
        confidence = last.get('confidence', 0.5)
        
        latest_filtered = {
            "lat": last.get('final_lat', 0),
            "lng": last.get('final_lng', 0),
            "raw_lat": last.get('raw_lat', 0),
            "raw_lng": last.get('raw_lng', 0),
            "uncertainty": last.get('uncertainty', 0),
            "confidence": confidence,
            "vibration": vibration,
            "vibration_class": vibration_class,
            "vibration_icon": icon,
            "earthquake_level": earthquake_level,
            "is_stationary": last.get('is_stationary', False),
            "sats": last.get('sats', 0),
            "filter_type": last.get('filter_type', 'UKF_Enhanced'),
            "timestamp": last.get('datetime', datetime.now().strftime("%H:%M:%S")),
            "status": f"UKF ({confidence:.1%})"
        }
        
        history_filtered = filtered[-200:]
        socketio.emit('filtered_update', latest_filtered)
        logger.info(f"✅ UKF updated: Lat={latest_filtered['lat']:.6f}, Lng={latest_filtered['lng']:.6f}, Conf={confidence:.1%}, Vib={vibration:.3f}m/s²")
    else:
        latest_filtered = {
            "lat": None, "lng": None, "uncertainty": None,
            "confidence": None, "vibration": None,
            "vibration_class": None, "earthquake_level": None,
            "status": "Chưa có dữ liệu"
        }
        history_filtered = []

# ============================
# ROUTES
# ============================
@app.route('/raw_location', methods=['POST'])
def receive_raw():
    global latest_raw
    
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "No data"}), 400
        
        # Lưu và tính độ rung
        append_raw_record(data)
        
        # Lấy dữ liệu mới nhất
        raw_records = load_raw_data()
        if raw_records:
            last = raw_records[-1]
            latest_raw = {
                "lat": last.get('lat', 0),
                "lng": last.get('lng', 0),
                "roll": last.get('roll', 0),
                "pitch": last.get('pitch', 0),
                "accel_x": last.get('accel_x', 0),
                "accel_y": last.get('accel_y', 0),
                "accel_z": last.get('accel_z', 0),
                "vibration": last.get('vibration', 0),
                "vibration_class": last.get('vibration_class', 'N/A'),
                "vibration_icon": last.get('vibration_icon', ''),
                "earthquake_level": last.get('earthquake_level', 'N/A'),
                "speed": last.get('speed', 0),
                "sats": last.get('sats', 0),
                "seq": last.get('seq', 0),
                "timestamp": last.get('datetime', datetime.now().strftime("%H:%M:%S")),
                "status": "Raw GPS"
            }
        
        socketio.emit('raw_update', latest_raw)
        
        # Chạy UKF trong thread riêng (không block)
        threading.Thread(target=run_ukf_processor, daemon=True).start()
        
        logger.info(f"📥 Raw: ({data.get('lat', 0):.6f}, {data.get('lng', 0):.6f}), Vib: {latest_raw.get('vibration', 0):.4f}m/s², seq={data.get('seq', 0)}")
        
        return jsonify({"message": "OK", "received": True}), 200
        
    except Exception as e:
        logger.error(f"Error: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/clear', methods=['POST'])
def clear_data():
    try:
        if clear_all_data():
            socketio.emit('data_cleared', {'timestamp': datetime.now().isoformat()})
            return jsonify({"message": "Cleared"}), 200
        return jsonify({"error": "Failed to clear"}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/raw')
def get_raw():
    return jsonify(latest_raw)

@app.route('/api/filtered')
def get_filtered():
    return jsonify(latest_filtered)

@app.route('/api/history')
def get_history():
    return jsonify(history_filtered)

@app.route('/api/all_filtered')
def get_all_filtered():
    return jsonify(load_filtered_data())

@app.route('/api/status')
def get_status():
    raw_count = len(load_raw_data())
    filtered_count = len(load_filtered_data())
    return jsonify({
        "status": "running",
        "raw_count": raw_count,
        "filtered_count": filtered_count,
        "latest_filtered": latest_filtered.get('timestamp'),
        "ukf_ready": bool(load_filtered_data()),
        "confidence": latest_filtered.get('confidence', 0),
        "vibration": latest_filtered.get('vibration', 0),
        "vibration_class": latest_filtered.get('vibration_class', 'N/A'),
        "earthquake_level": latest_filtered.get('earthquake_level', 'N/A')
    })

# ============================
# SOCKETIO
# ============================
@socketio.on('connect')
def handle_connect():
    emit('connection_status', {'status': 'connected'})
    if latest_filtered['lat'] is not None:
        emit('filtered_update', latest_filtered)

@socketio.on('ping')
def handle_ping():
    emit('pong', {'timestamp': datetime.now().isoformat()})

# ============================
# RUN
# ============================
if __name__ == '__main__':
    update_filtered_data()
    
    print("\n" + "="*50)
    print("GPS TRACKER - ENHANCED RELIABILITY")
    print("="*50)
    print(f"Vibration buffer size: {VIBRATION_BUFFER_SIZE}")
    print(f"Max records: {MAX_RECORDS}")
    print("Truy cập: http://localhost:5000")
    print("="*50 + "\n")
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)