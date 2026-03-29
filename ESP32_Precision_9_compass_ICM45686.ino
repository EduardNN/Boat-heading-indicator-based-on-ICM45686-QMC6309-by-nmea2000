//
// NMEA 2000 Курсовой датчик с ICM45686 + QMC6309
// Использует ESPAsyncWebServer с JSON API
//

// ==================== ПИНЫ CAN ====================
#define ESP32_CAN_TX_PIN GPIO_NUM_32
#define ESP32_CAN_RX_PIN GPIO_NUM_34

// ==================== NMEA2000 НАСТРОЙКИ ====================
#define NMEA2000_DEVICEID 65
#define DEV_COMPASS 0  // <-- ДОБАВЛЯЕМ ЭТУ СТРОКУ!

// ==================== БИБЛИОТЕКИ ====================
#include <Arduino.h>
#include <EEPROM.h>  
#include <Wire.h>
#include <math.h>
#include <esp_task_wdt.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA2000_CAN.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// WiFi и Web Server
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// VQF фильтр
#include "vqf.h"

// ==================== ПИНЫ I2C ====================
#define I2C_SDA 16
#define I2C_SCL 17

// ==================== АДРЕСА ДАТЧИКОВ ====================
#define ICM45686_ADDR 0x68
#define QMC6309_ADDR 0x30

// ==================== РЕГИСТРЫ ICM45686 ====================
#define ICM_WHO_AM_I     0x75
#define ICM_DEVICE_CONFIG 0x11
#define ICM_PWR_MGMT_0    0x4E
#define ICM_GYRO_CONFIG   0x4F
#define ICM_ACCEL_CONFIG  0x50
#define ICM_FIFO_CONFIG0  0x16
#define ICM_FIFO_CONFIG1  0x5F
#define ICM_FIFO_COUNTS   0x2E
#define ICM_FIFO_DATA     0x30

// ==================== РЕГИСТРЫ QMC6309 ====================
#define QMC_CHIP_ID      0x00
#define QMC_RESET        0x01
#define QMC_CTRL1        0x02
#define QMC_XOUT_L       0x06
#define QMC_YOUT_L       0x08
#define QMC_ZOUT_L       0x0A

// ==================== ПАРАМЕТРЫ ДАТЧИКОВ ====================
#define ICM_GYRO_SENSITIVITY 32.8f
#define ICM_ACCEL_SENSITIVITY 4096.0f
#define QMC_SENSITIVITY  0.15f

// ==================== WATCHDOG ====================
#define WDT_TIMEOUT 40

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================

// Ориентация
float Pitch = 0, Roll = 0, HeadingFiltered = 0;
float RateofTurn = 0;
int SID = 0;
bool send_heading = true;

// VQF объект
VQF vqf(1.0f/200.0f);

// Калибровочные данные
float gyroBias[3] = {0, 0, 0};
float magBias[3] = {0, 0, 0};
float magScale[3] = {1, 1, 1};

// Web Server
AsyncWebServer server(80);

tN2kMsg N2kMsg;
int DEVICE_ID = 65;

// ==================== ДРАЙВЕРЫ ДАТЧИКОВ ====================

bool initICM45686() {
    Wire.beginTransmission(ICM45686_ADDR);
    Wire.write(ICM_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM45686_ADDR, 1);
    uint8_t whoAmI = Wire.read();
    
    if (whoAmI != 0x47) {
        Serial.print("ICM45686 WHO_AM_I error: 0x");
        Serial.println(whoAmI, HEX);
        return false;
    }
    
    // Сброс
    Wire.beginTransmission(ICM45686_ADDR);
    Wire.write(ICM_DEVICE_CONFIG);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(20);
    
    // Настройка гироскопа: 1000dps, 200Hz
    Wire.beginTransmission(ICM45686_ADDR);
    Wire.write(ICM_GYRO_CONFIG);
    Wire.write((0b001 << 5) | 0b0111);
    Wire.endTransmission();
    
    // Настройка акселерометра: 8g, 100Hz
    Wire.beginTransmission(ICM45686_ADDR);
    Wire.write(ICM_ACCEL_CONFIG);
    Wire.write((0b001 << 5) | 0b1000);
    Wire.endTransmission();
    
    // Включение датчиков
    Wire.beginTransmission(ICM45686_ADDR);
    Wire.write(ICM_PWR_MGMT_0);
    Wire.write(0b1111);
    Wire.endTransmission();
    
    return true;
}

bool initQMC6309() {
    Wire.beginTransmission(QMC6309_ADDR);
    Wire.write(QMC_CHIP_ID);
    Wire.endTransmission(false);
    Wire.requestFrom(QMC6309_ADDR, 1);
    uint8_t chipId = Wire.read();
    
    if (chipId != 0x20) {
        Serial.print("QMC6309 CHIP_ID error: 0x");
        Serial.println(chipId, HEX);
        return false;
    }
    
    // Сброс
    Wire.beginTransmission(QMC6309_ADDR);
    Wire.write(QMC_RESET);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(10);
    
    // Режим непрерывных измерений, 100Hz
    Wire.beginTransmission(QMC6309_ADDR);
    Wire.write(QMC_CTRL1);
    Wire.write(0b00100000);
    Wire.endTransmission();
    
    return true;
}

bool readICM45686(float& gx, float& gy, float& gz, float& ax, float& ay, float& az) {
    // Проверяем наличие данных в FIFO
    Wire.beginTransmission(ICM45686_ADDR);
    Wire.write(ICM_FIFO_COUNTS);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM45686_ADDR, 2);
    if (Wire.available() < 2) return false;
    
    uint16_t fifoCount = Wire.read() | (Wire.read() << 8);
    if (fifoCount < 15) return false;
    
    // Читаем данные из FIFO
    Wire.beginTransmission(ICM45686_ADDR);
    Wire.write(ICM_FIFO_DATA);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM45686_ADDR, min(fifoCount, (uint16_t)30));
    
    if (Wire.available() < 1) return false;
    Wire.read(); // пропускаем заголовок
    
    if (Wire.available() < 14) return false;
    uint8_t data[14];
    for (int i = 0; i < 14; i++) data[i] = Wire.read();
    
    // Распаковка акселерометра (16 бит)
    int16_t ax_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t ay_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t az_raw = (int16_t)((data[5] << 8) | data[4]);
    
    // Распаковка гироскопа (16 бит)
    int16_t gx_raw = (int16_t)((data[7] << 8) | data[6]);
    int16_t gy_raw = (int16_t)((data[9] << 8) | data[8]);
    int16_t gz_raw = (int16_t)((data[11] << 8) | data[10]);
    
    // Преобразование в физические единицы
    ax = (float)ax_raw / ICM_ACCEL_SENSITIVITY;
    ay = (float)ay_raw / ICM_ACCEL_SENSITIVITY;
    az = (float)az_raw / ICM_ACCEL_SENSITIVITY;
    
    gx = (float)gx_raw / ICM_GYRO_SENSITIVITY;
    gy = (float)gy_raw / ICM_GYRO_SENSITIVITY;
    gz = (float)gz_raw / ICM_GYRO_SENSITIVITY;
    
    return true;
}

bool readQMC6309(float& mx, float& my, float& mz) {
    // Проверяем статус
    Wire.beginTransmission(QMC6309_ADDR);
    Wire.write(QMC_CTRL1);
    Wire.endTransmission(false);
    Wire.requestFrom(QMC6309_ADDR, 1);
    if (Wire.available() < 1) return false;
    if (!(Wire.read() & 0x01)) return false;
    
    // Читаем данные
    Wire.beginTransmission(QMC6309_ADDR);
    Wire.write(QMC_XOUT_L);
    Wire.endTransmission(false);
    Wire.requestFrom(QMC6309_ADDR, 6);
    
    if (Wire.available() < 6) return false;
    
    int16_t mx_raw = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t my_raw = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t mz_raw = (int16_t)((Wire.read() << 8) | Wire.read());
    
    mx = (float)mx_raw * QMC_SENSITIVITY;
    my = (float)my_raw * QMC_SENSITIVITY;
    mz = (float)mz_raw * QMC_SENSITIVITY;
    
    return true;
}

// ==================== КАЛИБРОВКА ====================

void calibrateGyroscope() {
    float sum[3] = {0, 0, 0};
    int samples = 1000;
    
    Serial.println("Calibrating gyroscope... Keep device still!");
    
    for (int i = 0; i < samples; i++) {
        float gx, gy, gz, ax, ay, az;
        if (readICM45686(gx, gy, gz, ax, ay, az)) {
            sum[0] += gx;
            sum[1] += gy;
            sum[2] += gz;
        }
        delay(1);
    }
    
    gyroBias[0] = sum[0] / samples;
    gyroBias[1] = sum[1] / samples;
    gyroBias[2] = sum[2] / samples;
    
    Serial.printf("Gyro bias: %.2f, %.2f, %.2f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
    Serial.println("Gyroscope calibration complete");
}

void calibrateMagnetometer() {
    const int samples = 500;
    float mx_sum = 0, my_sum = 0, mz_sum = 0;
    float mx_min = 1000, mx_max = -1000;
    float my_min = 1000, my_max = -1000;
    float mz_min = 1000, mz_max = -1000;
    
    Serial.println("Calibrating magnetometer... Rotate device in all directions!");
    
    for (int i = 0; i < samples; i++) {
        float mx, my, mz;
        if (readQMC6309(mx, my, mz)) {
            mx_sum += mx; my_sum += my; mz_sum += mz;
            mx_min = min(mx_min, mx); mx_max = max(mx_max, mx);
            my_min = min(my_min, my); my_max = max(my_max, my);
            mz_min = min(mz_min, mz); mz_max = max(mz_max, mz);
        }
        delay(10);
    }
    
    magBias[0] = mx_sum / samples;
    magBias[1] = my_sum / samples;
    magBias[2] = mz_sum / samples;
    
    float mx_range = (mx_max - mx_min) / 2.0f;
    float my_range = (my_max - my_min) / 2.0f;
    float mz_range = (mz_max - mz_min) / 2.0f;
    float avg_range = (mx_range + my_range + mz_range) / 3.0f;
    
    if (mx_range > 0) magScale[0] = avg_range / mx_range;
    if (my_range > 0) magScale[1] = avg_range / my_range;
    if (mz_range > 0) magScale[2] = avg_range / mz_range;
    
    Serial.println("Magnetometer calibration complete");
}

// ==================== VQF ФИЛЬТРАЦИЯ ====================

bool getSensorValues() {
    static unsigned long lastGyrTime = 0;
    static unsigned long lastMagTime = 0;
    
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    unsigned long now = millis();
    
    if (!readICM45686(gx, gy, gz, ax, ay, az)) return false;
    
    // Вычитаем смещение гироскопа
    gx -= gyroBias[0];
    gy -= gyroBias[1];
    gz -= gyroBias[2];
    
    // Обновляем VQF
    float gyrRad[3] = {gx * 0.0174533f, gy * 0.0174533f, gz * 0.0174533f};
    vqf.updateGyr(gyrRad, 0.005f);
    
    // Обновляем акселерометр (10Hz)
    if (now - lastGyrTime >= 10) {
        lastGyrTime = now;
        float accMs2[3] = {ax * 9.81f, ay * 9.81f, az * 9.81f};
        vqf.updateAcc(accMs2);
    }
    
    // Обновляем магнитометр (10Hz)
    if (readQMC6309(mx, my, mz) && (now - lastMagTime >= 100)) {
        lastMagTime = now;
        // Калибровка магнитометра
        mx = (mx - magBias[0]) * magScale[0];
        my = (my - magBias[1]) * magScale[1];
        mz = (mz - magBias[2]) * magScale[2];
        float magData[3] = {mx, my, mz};
        vqf.updateMag(magData);
    }
    
    // Получаем кватернион
    float quat9D[4];
    vqf.getQuat9D(quat9D);
    
    float q0 = quat9D[0], q1 = quat9D[1], q2 = quat9D[2], q3 = quat9D[3];
    
    // Конвертируем в углы Эйлера
    Pitch = asin(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
    Roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    
    float yaw = atan2(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
    if (yaw < 0) yaw += 360.0f;
    HeadingFiltered = yaw;
    
    RateofTurn = gz;
    
    return true;
}

// ==================== WEB ИНТЕРФЕЙС ====================

void setupWebServer() {
    // Эндпоинт для получения данных в формате JSON
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncJsonResponse *response = new AsyncJsonResponse();
        JsonObject root = response->getRoot().to<JsonObject>();
        root["heading"] = HeadingFiltered;
        root["pitch"] = Pitch;
        root["roll"] = Roll;
        root["rate"] = RateofTurn;
        response->setLength();
        request->send(response);
    });
    
    // Эндпоинт для отправки команд
    server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request) {
        String responseMsg = "";
        if (request->hasParam("c")) {
            String cmd = request->getParam("c")->value();
            Serial.print("Command: ");
            Serial.println(cmd);
            
            if (cmd == "reset") {
                request->send(200, "text/plain", "Resetting...");
                delay(100);
                ESP.restart();
                return;
            }
            else if (cmd == "sendhead") {
                send_heading = true;
                responseMsg = "Heading transmission ENABLED";
            }
            else if (cmd == "stophead") {
                send_heading = false;
                responseMsg = "Heading transmission DISABLED";
            }
            else if (cmd == "cal") {
                request->send(200, "text/plain", "Starting full calibration...");
                calibrateGyroscope();
                calibrateMagnetometer();
                responseMsg = "Calibration complete";
            }
            else if (cmd == "calmag") {
                request->send(200, "text/plain", "Starting magnetic calibration...");
                calibrateMagnetometer();
                responseMsg = "Magnetic calibration complete";
            }
            else {
                responseMsg = "Unknown command. Available: reset, sendhead, stophead, cal, calmag";
            }
        } else {
            responseMsg = "No command specified. Use ?c=command";
        }
        request->send(200, "text/plain", responseMsg);
    });
    
    // Главная страница
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = "<!DOCTYPE HTML><html><head>";
        html += "<title>NMEA 2000 Compass</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>";
        html += "body{font-family:Arial;margin:0;padding:20px;background:#1a1a2e;color:#eee;}";
        html += ".container{max-width:800px;margin:auto;}";
        html += ".card{background:#16213e;border-radius:10px;padding:20px;margin:10px 0;}";
        html += ".data-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:15px;}";
        html += ".data-item{background:#0f3460;padding:15px;border-radius:8px;text-align:center;}";
        html += ".data-value{font-size:28px;font-weight:bold;color:#e94560;}";
        html += ".data-label{font-size:12px;color:#888;margin-top:5px;}";
        html += "button{background:#e94560;color:white;border:none;padding:10px 20px;margin:5px;border-radius:5px;cursor:pointer;}";
        html += "button:hover{background:#ff6b6b;}";
        html += "</style></head><body>";
        html += "<div class='container'><h1>🧭 NMEA 2000 Compass</h1>";
        html += "<div class='card'><div class='data-grid'>";
        html += "<div class='data-item'><div class='data-value' id='heading'>---</div><div class='data-label'>Heading (°)</div></div>";
        html += "<div class='data-item'><div class='data-value' id='pitch'>---</div><div class='data-label'>Pitch (°)</div></div>";
        html += "<div class='data-item'><div class='data-value' id='roll'>---</div><div class='data-label'>Roll (°)</div></div>";
        html += "<div class='data-item'><div class='data-value' id='rate'>---</div><div class='data-label'>Rate of Turn (°/s)</div></div>";
        html += "</div></div>";
        html += "<div class='card'><h3>Commands</h3>";
        html += "<button onclick=\"fetch('/cmd?c=cal').then(r=>r.text()).then(t=>document.getElementById('response').innerHTML=t)\">Full Calibration</button>";
        html += "<button onclick=\"fetch('/cmd?c=calmag').then(r=>r.text()).then(t=>document.getElementById('response').innerHTML=t)\">Mag Only</button>";
        html += "<button onclick=\"fetch('/cmd?c=reset')\">Reset</button>";
        html += "<button onclick=\"fetch('/cmd?c=sendhead')\">Send Heading ON</button>";
        html += "<button onclick=\"fetch('/cmd?c=stophead')\">Send Heading OFF</button>";
        html += "<div id='response' style='margin-top:10px;background:#0a0c10;padding:10px;font-family:monospace;'>Ready...</div>";
        html += "</div></div>";
        html += "<script>";
        html += "function updateData(){fetch('/data').then(r=>r.json()).then(d=>{";
        html += "document.getElementById('heading').innerHTML=d.heading.toFixed(1);";
        html += "document.getElementById('pitch').innerHTML=d.pitch.toFixed(1);";
        html += "document.getElementById('roll').innerHTML=d.roll.toFixed(1);";
        html += "document.getElementById('rate').innerHTML=d.rate.toFixed(1);";
        html += "});}";
        html += "setInterval(updateData,1000);updateData();";
        html += "</script></body></html>";
        request->send(200, "text/html", html);
    });
    
    server.begin();
    Serial.println("HTTP server started");
}

void setupWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Compass_AP", "12345678");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
}

// ==================== NMEA2000 ====================

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
    // Обработка входящих сообщений NMEA2000
}

// ==================== SETUP ====================

void setup() {
    pinMode(18, OUTPUT);
    digitalWrite(18, HIGH);
    
    Serial.begin(115200);
    Serial.println("Starting NMEA2000 Compass with ICM45686+QMC6309...");
    
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    delay(100);
    
    // EEPROM инициализация
    if (!EEPROM.begin(512)) {
        Serial.println("EEPROM start failed");
    }
    
    // I2C инициализация
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    
    // WiFi и Web сервер
    setupWiFi();
    setupWebServer();
    
    // Инициализация датчиков
    if (!initICM45686()) {
        Serial.println("ICM45686 initialization failed!");
    } else {
        Serial.println("ICM45686 initialized OK");
    }
    
    if (!initQMC6309()) {
        Serial.println("QMC6309 initialization failed!");
    } else {
        Serial.println("QMC6309 initialized OK");
    }
    
    // Калибровка
    calibrateGyroscope();
    calibrateMagnetometer();
    
    // Настройка VQF
    VQFParams params = vqf.getParams();
    params.tauAcc = 4.0;
    params.tauMag = 10.0;
    vqf = VQF(params, 0.005f);
    
    send_heading = true;
    
    // NMEA2000 инициализация
    NMEA2000.SetProductInformation("107018103", 13233, 
                                   "Precision-9 Compass", "2.0.3-VQF-WiFi", 
                                   "", 1, 0xffff, 0xff, DEV_COMPASS);
    
    NMEA2000.SetDeviceInformation(1048678, 140, 60, 275, 4, DEV_COMPASS);
    NMEA2000.SetN2kCANMsgBufSize(20);
    NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);
    NMEA2000.SetForwardSystemMessages(false);
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, DEVICE_ID);
    NMEA2000.EnableForward(false);
    
    if(false == NMEA2000.Open()) 
        Serial.println("NMEA2000 not Ready");
    else
        Serial.println("NMEA2000 is Ready");
    
    NMEA2000.SendProductInformation();
    
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);
    
    Serial.println("Setup complete!");
    Serial.print("Connect to WiFi AP 'Compass_AP' (password: 12345678) and go to http://");
    Serial.println(WiFi.softAPIP());
}

// ==================== LOOP ====================

void loop() {
    if (getSensorValues()) {
        if (send_heading) {
            N2kMsg.Clear();
            SetN2kPGN127250(N2kMsg, SID, HeadingFiltered * 0.0174533f, 
                           N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
            NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
            
            N2kMsg.Clear();
            SetN2kRateOfTurn(N2kMsg, SID, -RateofTurn * 0.0174533f);
            NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
        }
        
        N2kMsg.Clear();
        SetN2kAttitude(N2kMsg, SID, N2kDoubleNA, 
                      -(Pitch * 0.0174533f), -(Roll * 0.0174533f));
        NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
        
        SID++;
        if (SID > 250) SID = 1;
        
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 5000) {
            lastDebug = millis();
            Serial.printf("H:%.1f P:%.1f R:%.1f RoT:%.1f\n", 
                         HeadingFiltered, Pitch, Roll, RateofTurn);
        }
    }
    
    NMEA2000.ParseMessages();
    delay(5);
}