/*
 * ESP32 GPS Tracker - ULTRA STABLE (Multi-Strategy)
 * - FreeRTOS + Task separation
 * - Queue for non-blocking communication
 * - Deep Sleep power saving
 * - ESP-NOW backup
 * - Memory monitoring
 * - Task Watchdog
 * 
 * Giữ nguyên tính năng: GPS, MPU, Gửi dữ liệu lên server
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <esp_now.h>

// ============================
// CẤU HÌNH WIFI
// ============================
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

// ============================
// CẤU HÌNH SERVER
// ============================
const char* SERVER_HOST = "192.168.2.108";
const int SERVER_PORT = 5000;
const char* SERVER_PATH = "/raw_location";
char SERVER_URL[100];

// ============================
// CẤU HÌNH GPS
// ============================
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// ============================
// CẤU HÌNH MPU6050
// ============================
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_READ_INTERVAL 100
#define I2C_TIMEOUT_MS 50
#define I2C_CLOCK_SPEED 100000

// ============================
// CẤU HÌNH KẾT NỐI
// ============================
#define SEND_INTERVAL_MS 5000
#define WIFI_CHECK_INTERVAL 10000
#define MAX_WIFI_RETRIES 10

// ============================
// CẤU HÌNH RESET
// ============================
#define RESET_AFTER_SUCCESSFUL_SEND 60
#define COUNTDOWN_SECONDS 3
#define MIN_RUN_TIME_BEFORE_RESET 30
#define WATCHDOG_TIMEOUT 60000
#define MAX_MPU_FAILS 5

// ============================
// CẤU HÌNH DEEP SLEEP
// ============================
#define DEEP_SLEEP_ENABLED true
#define DEEP_SLEEP_DURATION 10  // Ngủ 10 giây sau mỗi 60 giây chạy
#define DEEP_SLEEP_INTERVAL 60   // Chạy 60 giây rồi ngủ

// ============================
// CẤU HÌNH ESP-NOW (DỰ PHÒNG)
// ============================
#define ESP_NOW_ENABLED true
#define GATEWAY_MAC {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56}  // Thay MAC gateway

// ============================
// CẤU HÌNH QUEUE
// ============================
#define QUEUE_SIZE 20

// ============================
// BIẾN TOÀN CỤC
// ============================
TinyGPSPlus gps;
MPU6050 mpu(Wire);
WiFiClient client;

// Dữ liệu GPS
double raw_lat = 0, raw_lng = 0;
float speed = 0;
int sats = 0;

// Dữ liệu MPU
float roll = 0, pitch = 0;
float accel_x = 0, accel_y = 0, accel_z = 0;

bool mpu_ok = false;
int mpu_fail_count = 0;
unsigned long last_mpu_success = 0;

// Trạng thái
unsigned long sequence = 0;
bool gps_fixed = false;
bool wifi_connected = false;
bool server_ok = false;
bool first_gps = true;

// Reset management
bool first_successful_send = false;
unsigned long first_success_time = 0;
unsigned long start_time = 0;
bool is_counting_down = false;
unsigned long countdown_start = 0;

// Thời gian
unsigned long last_send_time = 0;
unsigned long last_print_time = 0;
unsigned long last_wifi_check = 0;
unsigned long last_activity_time = 0;
unsigned long last_mpu_read = 0;
unsigned long last_wifi_attempt = 0;
unsigned long deep_sleep_start = 0;
unsigned long run_time = 0;

// WiFi retry
int wifi_retry_count = 0;
int send_fail_count = 0;

// Queue
typedef struct {
    double lat;
    double lng;
    float roll;
    float pitch;
    float accel_x;
    float accel_y;
    float accel_z;
    float speed;
    int sats;
    unsigned long seq;
} DataPacket;

QueueHandle_t dataQueue;

// ESP-NOW
bool esp_now_ready = false;
uint8_t gatewayMac[] = GATEWAY_MAC;

// ============================
// HÀM RESET
// ============================
void resetESP32() {
    Serial.println("\n========================================");
    Serial.println("🔄 RESET ESP32");
    Serial.printf("📊 Runtime: %.0f seconds\n", (millis() - start_time) / 1000.0);
    Serial.printf("📊 Data sent: %d records\n", sequence);
    Serial.printf("📊 Send fails: %d\n", send_fail_count);
    Serial.printf("💾 Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("========================================\n");
    delay(300);
    ESP.restart();
}

// ============================
// HÀM KIỂM TRA RESET
// ============================
void checkAndPerformReset() {
    if (!first_successful_send) return;
    if (millis() - start_time < MIN_RUN_TIME_BEFORE_RESET * 1000) return;
    
    unsigned long elapsed = (millis() - first_success_time) / 1000;
    
    if (elapsed >= RESET_AFTER_SUCCESSFUL_SEND) {
        if (!is_counting_down) {
            is_counting_down = true;
            countdown_start = millis();
            Serial.println("\n⚠️ RESET COUNTDOWN: 3 seconds");
        }
        
        unsigned long remaining = (millis() - countdown_start) / 1000;
        if (remaining >= COUNTDOWN_SECONDS) {
            resetESP32();
        }
    }
}

// ============================
// HÀM KIỂM TRA TREO (WATCHDOG)
// ============================
void checkFreeze() {
    // Task Watchdog
    esp_task_wdt_reset();
    
    if (millis() - last_activity_time > WATCHDOG_TIMEOUT) {
        Serial.println("⚠️ System frozen! Resetting...");
        delay(500);
        ESP.restart();
    }
}

// ============================
// HÀM KIỂM TRA BỘ NHỚ
// ============================
void checkMemory() {
    static unsigned long last_mem_check = 0;
    if (millis() - last_mem_check > 30000) {  // Mỗi 30s
        last_mem_check = millis();
        uint32_t heap = ESP.getFreeHeap();
        Serial.printf("💾 Heap: %d bytes\n", heap);
        
        if (heap < 20000) {
            Serial.println("⚠️ Low memory! Resetting...");
            delay(500);
            ESP.restart();
        }
    }
}

// ============================
// HÀM DEEP SLEEP
// ============================
void enterDeepSleep() {
    if (!DEEP_SLEEP_ENABLED) return;
    
    run_time = (millis() - start_time) / 1000;
    if (run_time < DEEP_SLEEP_INTERVAL) return;
    if (!first_successful_send) return;
    
    Serial.printf("🌙 Entering Deep Sleep for %d seconds...\n", DEEP_SLEEP_DURATION);
    delay(100);
    
    // Cấu hình wake-up
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION * 1000000);
    
    // Vào ngủ
    esp_deep_sleep_start();
}

// ============================
// HÀM KẾT NỐI WIFI
// ============================
bool connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        wifi_retry_count = 0;
        return true;
    }
    
    unsigned long delay_ms = min((unsigned long)pow(2, wifi_retry_count) * 1000, 30000UL);
    if (millis() - last_wifi_attempt < delay_ms) {
        return false;
    }
    last_wifi_attempt = millis();
    
    wifi_retry_count++;
    if (wifi_retry_count <= 3) {
        Serial.printf("📶 WiFi attempt %d/%d...\n", wifi_retry_count, MAX_WIFI_RETRIES);
    }
    
    WiFi.disconnect(true);
    delay(50);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 15) {
        delay(150);
        attempts++;
        last_activity_time = millis();
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        wifi_retry_count = 0;
        Serial.printf("✅ WiFi: %s\n", WiFi.localIP().toString().c_str());
        return true;
    } else {
        wifi_connected = false;
        if (wifi_retry_count <= 3 || wifi_retry_count % 5 == 0) {
            Serial.printf("❌ WiFi failed (attempt %d)\n", wifi_retry_count);
        }
        
        if (wifi_retry_count >= MAX_WIFI_RETRIES) {
            Serial.println("⚠️ Too many WiFi failures! Using ESP-NOW...");
            return false;
        }
        return false;
    }
}

// ============================
// HÀM KIỂM TRA SERVER
// ============================
bool checkServer() {
    if (WiFi.status() != WL_CONNECTED) {
        server_ok = false;
        return false;
    }
    
    HTTPClient http;
    char url[100];
    sprintf(url, "http://%s:%d/api/status", SERVER_HOST, SERVER_PORT);
    
    http.begin(client, url);
    http.setTimeout(1500);
    int code = http.GET();
    http.end();
    
    server_ok = (code == 200);
    return server_ok;
}

// ============================
// HÀM GỬI DỮ LIỆU (HTTP)
// ============================
bool sendHTTP(DataPacket* packet) {
    if (WiFi.status() != WL_CONNECTED || !server_ok) {
        return false;
    }
    
    HTTPClient http;
    http.begin(client, SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(2000);
    
    StaticJsonDocument<200> doc;
    doc["lat"] = packet->lat;
    doc["lng"] = packet->lng;
    doc["roll"] = packet->roll;
    doc["pitch"] = packet->pitch;
    doc["accel_x"] = packet->accel_x;
    doc["accel_y"] = packet->accel_y;
    doc["accel_z"] = packet->accel_z;
    doc["speed"] = packet->speed;
    doc["sats"] = packet->sats;
    doc["seq"] = packet->seq;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    int code = http.POST(jsonString);
    http.end();
    
    if (code == 200) {
        send_fail_count = 0;
        return true;
    } else {
        send_fail_count++;
        if (code == -1) {
            server_ok = false;
        }
        return false;
    }
}

// ============================
// HÀM GỬI QUA ESP-NOW (DỰ PHÒNG)
// ============================
bool sendESPNow(DataPacket* packet) {
    if (!esp_now_ready) return false;
    
    // Chuyển đổi dữ liệu sang dạng ESP-NOW
    esp_now_send(gatewayMac, (uint8_t*)packet, sizeof(DataPacket));
    
    Serial.printf("📡 ESP-NOW sent seq=%d\n", packet->seq);
    return true;
}

// ============================
// HÀM RESET I2C
// ============================
void resetI2C() {
    Serial.println("🔄 Resetting I2C...");
    Wire.end();
    delay(50);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_CLOCK_SPEED);
    Wire.setTimeout(I2C_TIMEOUT_MS);
    delay(50);
    
    byte status = mpu.begin();
    if (status == 0) {
        mpu_ok = true;
        mpu_fail_count = 0;
        last_mpu_success = millis();
        Serial.println("✅ I2C reset ok");
    } else {
        mpu_ok = false;
        Serial.printf("❌ I2C reset failed (0x%02X)\n", status);
    }
}

// ============================
// HÀM ĐỌC MPU
// ============================
void readMPU() {
    if (millis() - last_mpu_read < MPU_READ_INTERVAL) return;
    last_mpu_read = millis();
    
    if (!mpu_ok) {
        mpu_fail_count++;
        if (mpu_fail_count >= MAX_MPU_FAILS) {
            resetI2C();
            mpu_fail_count = 0;
        }
        return;
    }
    
    Wire.setTimeout(I2C_TIMEOUT_MS);
    mpu.update();
    
    float new_roll = mpu.getAngleX();
    float new_pitch = mpu.getAngleY();
    float raw_ax = mpu.getAccX();
    float raw_ay = mpu.getAccY();
    float raw_az = mpu.getAccZ();
    
    if (!isnan(new_roll) && !isinf(new_roll) && 
        !isnan(new_pitch) && !isinf(new_pitch) &&
        !isnan(raw_ax) && !isinf(raw_ax)) {
        
        roll = new_roll;
        pitch = new_pitch;
        accel_x = raw_ax;
        accel_y = raw_ay;
        accel_z = raw_az;
        mpu_ok = true;
        mpu_fail_count = 0;
        last_mpu_success = millis();
    } else {
        mpu_ok = false;
        mpu_fail_count++;
    }
    
    last_activity_time = millis();
}

// ============================
// TASK: GPS READING
// ============================
void taskGPS(void *pvParameters) {
    while (1) {
        while (Serial2.available() > 0) {
            char c = Serial2.read();
            gps.encode(c);
        }
        
        if (gps.location.isValid() && gps.location.age() < 3000) {
            raw_lat = gps.location.lat();
            raw_lng = gps.location.lng();
            speed = gps.speed.kmph();
            sats = gps.satellites.value();
            
            if (!gps_fixed) {
                gps_fixed = true;
                Serial.println("✅ GPS fixed!");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============================
// TASK: DATA PROCESSING & SENDING
// ============================
void taskSend(void *pvParameters) {
    DataPacket packet;
    
    while (1) {
        // Đợi dữ liệu từ queue
        if (xQueueReceive(dataQueue, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Gửi HTTP
            bool sent = sendHTTP(&packet);
            
            if (sent) {
                if (!first_successful_send) {
                    first_successful_send = true;
                    first_success_time = millis();
                    Serial.printf("✅ First success! Reset in 60s\n");
                }
                sequence++;
                Serial.printf("📤 Sent seq=%d\n", sequence);
            } else {
                // Thử ESP-NOW nếu HTTP thất bại
                if (ESP_NOW_ENABLED && esp_now_ready) {
                    sendESPNow(&packet);
                }
                
                send_fail_count++;
                if (send_fail_count > 5) {
                    server_ok = false;
                }
            }
        }
    }
}

// ============================
// SETUP
// ============================
void setup() {
    Serial.begin(115200);
    delay(100);
    
    start_time = millis();
    last_activity_time = millis();
    
    Serial.println("\n========================================");
    Serial.println("   ESP32 - ULTRA STABLE (Multi-Strategy)");
    Serial.println("========================================\n");
    Serial.printf("📶 WiFi: %s\n", WIFI_SSID);
    Serial.printf("📊 Send interval: %d ms\n", SEND_INTERVAL_MS);
    Serial.printf("📊 MPU interval: %d ms\n", MPU_READ_INTERVAL);
    Serial.printf("🔄 Reset after %d seconds\n", RESET_AFTER_SUCCESSFUL_SEND);
    Serial.printf("🌙 Deep Sleep: %s (%ds / %ds)\n", 
                 DEEP_SLEEP_ENABLED ? "ON" : "OFF",
                 DEEP_SLEEP_DURATION, DEEP_SLEEP_INTERVAL);
    Serial.printf("📡 ESP-NOW: %s\n", ESP_NOW_ENABLED ? "ON" : "OFF");
    Serial.println("========================================\n");
    
    sprintf(SERVER_URL, "http://%s:%d%s", SERVER_HOST, SERVER_PORT, SERVER_PATH);
    Serial.printf("📡 Server: %s\n", SERVER_URL);
    Serial.println();
    
    // ===== 1. CẤU HÌNH WATCHDOG =====
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    Serial.println("✅ Watchdog initialized");
    
    // ===== 2. I2C =====
    Serial.println("🔧 Initializing I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_CLOCK_SPEED);
    Wire.setTimeout(I2C_TIMEOUT_MS);
    delay(50);
    
    // ===== 3. MPU6050 =====
    Serial.print("🔧 MPU6050... ");
    unsigned long mpu_start = millis();
    byte status = 0;
    bool mpu_init_ok = false;
    
    while (millis() - mpu_start < 2000) {
        status = mpu.begin();
        if (status == 0) {
            mpu_init_ok = true;
            break;
        }
        delay(100);
        last_activity_time = millis();
    }
    
    if (mpu_init_ok) {
        Serial.println("✅ Connected");
        Serial.print("   Calibrating... ");
        mpu.calcOffsets(true, true);
        Serial.println("Done");
        mpu_ok = true;
        last_mpu_success = millis();
    } else {
        Serial.printf("❌ Not found (0x%02X)\n", status);
        Serial.println("   Continuing without MPU");
        mpu_ok = false;
    }
    
    // ===== 4. GPS =====
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("✅ GPS ready");
    Serial.println();
    
    // ===== 5. WiFi =====
    Serial.println("📶 Connecting WiFi...");
    connectWiFi();
    
    // ===== 6. Server =====
    if (wifi_connected) {
        checkServer();
    }
    
    // ===== 7. ESP-NOW =====
    if (ESP_NOW_ENABLED) {
        WiFi.mode(WIFI_STA);
        if (esp_now_init() == ESP_OK) {
            esp_now_peer_info_t peerInfo;
            memcpy(peerInfo.peer_addr, gatewayMac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            if (esp_now_add_peer(&peerInfo) == ESP_OK) {
                esp_now_ready = true;
                Serial.println("✅ ESP-NOW ready");
            }
        }
    }
    
    // ===== 8. TẠO QUEUE =====
    dataQueue = xQueueCreate(QUEUE_SIZE, sizeof(DataPacket));
    if (dataQueue == NULL) {
        Serial.println("❌ Queue creation failed!");
        delay(1000);
        ESP.restart();
    }
    
    // ===== 9. TẠO TASK =====
    xTaskCreatePinnedToCore(
        taskGPS, 
        "GPS Task", 
        4096, 
        NULL, 
        1, 
        NULL, 
        0
    );
    
    xTaskCreatePinnedToCore(
        taskSend, 
        "Send Task", 
        8192, 
        NULL, 
        2, 
        NULL, 
        1
    );
    
    Serial.println("✅ Tasks created!");
    
    last_send_time = millis();
    last_print_time = millis();
    last_wifi_check = millis();
    last_mpu_read = millis();
    
    Serial.println("\n📍 Waiting for GPS...\n");
}

// ============================
// LOOP CHÍNH (TỐI GIẢN)
// ============================
void loop() {
    // === WATCHDOG ===
    esp_task_wdt_reset();
    checkFreeze();
    last_activity_time = millis();
    
    // === WIFI & SERVER ===
    if (millis() - last_wifi_check > WIFI_CHECK_INTERVAL) {
        last_wifi_check = millis();
        if (!wifi_connected) {
            connectWiFi();
            if (wifi_connected) {
                delay(200);
                checkServer();
            }
        } else if (!server_ok) {
            checkServer();
        }
    }
    
    // === ĐỌC MPU ===
    readMPU();
    
    // === GỬI DỮ LIỆU VÀO QUEUE ===
    if (gps_fixed && mpu_ok) {
        if (millis() - last_send_time >= SEND_INTERVAL_MS) {
            last_send_time = millis();
            
            DataPacket packet;
            packet.lat = raw_lat;
            packet.lng = raw_lng;
            packet.roll = roll;
            packet.pitch = pitch;
            packet.accel_x = accel_x;
            packet.accel_y = accel_y;
            packet.accel_z = accel_z;
            packet.speed = speed;
            packet.sats = sats;
            packet.seq = sequence;
            
            // Gửi vào queue (non-blocking)
            if (xQueueSend(dataQueue, &packet, 0) != pdTRUE) {
                Serial.println("⚠️ Queue full!");
            }
        }
    }
    
    // === HIỂN THỊ ===
    if (millis() - last_print_time > 5000) {
        last_print_time = millis();
        
        if (is_counting_down) {
            unsigned long remaining = (millis() - countdown_start) / 1000;
            int countdown = COUNTDOWN_SECONDS - remaining;
            if (countdown > 0) {
                Serial.printf("⏰ RESET: %ds\n", countdown);
            }
        }
        
        if (gps_fixed) {
            Serial.printf("📍 %.6f, %.6f | Sats:%d | Seq:%d | Heap:%d\n", 
                         raw_lat, raw_lng, sats, sequence, ESP.getFreeHeap());
        } else {
            Serial.printf("⏳ GPS: Sats=%d\n", sats);
        }
    }
    
    // === KIỂM TRA RESET ===
    checkAndPerformReset();
    
    // === KIỂM TRA BỘ NHỚ ===
    checkMemory();
    
    // === DEEP SLEEP ===
    enterDeepSleep();
    
    // === DELAY ===
    delay(5);
}