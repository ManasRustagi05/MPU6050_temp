#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

// ==========================================
// 1. Network Credentials
// ==========================================
const char* ssid     = "JioFiber-Rita";     // Replace with your Wi-Fi Name
const char* password = "(@bAbu#1)"; // Replace with your Wi-Fi Password

// ==========================================
// 2. MPU-6050 Registers & Calibration Constants
// ==========================================
#define MPU6050_ADDR       0x68
#define PWR_MGMT_1         0x6B
#define ACCEL_XOUT_H       0x3B

#define ACCEL_SCALE        16384.0f // +/-2g
#define GYRO_SCALE         131.0f   // +/-250 deg/s

WebServer server(80);

// Global Telemetry Variables
float temperature = 0.0;
float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;
float roll = 0.0, pitch = 0.0;

// ==========================================
// 3. MPU-6050 Sensor Hardware Interface
// ==========================================
void initMPU6050() {
    Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);   // Wake up MPU-6050 (clear sleep mode)
    Wire.endTransmission(true);
}

void readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

    if (Wire.available() >= 14) {
        int16_t raw_ax   = (Wire.read() << 8) | Wire.read();
        int16_t raw_ay   = (Wire.read() << 8) | Wire.read();
        int16_t raw_az   = (Wire.read() << 8) | Wire.read();
        int16_t raw_temp = (Wire.read() << 8) | Wire.read();
        int16_t raw_gx   = (Wire.read() << 8) | Wire.read();
        int16_t raw_gy   = (Wire.read() << 8) | Wire.read();
        int16_t raw_gz   = (Wire.read() << 8) | Wire.read();

        // Convert raw register values to physical units
        temperature = ((float)raw_temp / 340.0f) + 36.53f; // Temperature formula
        ax = raw_ax / ACCEL_SCALE;
        ay = raw_ay / ACCEL_SCALE;
        az = raw_az / ACCEL_SCALE;
        gx = raw_gx / GYRO_SCALE;
        gy = raw_gy / GYRO_SCALE;
        gz = raw_gz / GYRO_SCALE;

        // Calculate Roll and Pitch angles (in degrees)
        roll  = atan2(ay, az) * 180.0 / M_PI;
        pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    }
}

// ==========================================
// 4. CORS-Enabled REST Endpoint
// ==========================================
void handleData() {
    readMPU6050(); // Read latest sensor state

    // Enable Cross-Origin Resource Sharing (CORS) for external web apps
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET");
    server.sendHeader("Access-Control-Allow-Headers", "*");

    // Format metrics into JSON stream
    String json = "{";
    json += "\"connected\": true,";
    json += "\"temp\":"   + String(temperature, 2) + ",";
    json += "\"ax\":"     + String(ax, 2)          + ",";
    json += "\"ay\":"     + String(ay, 2)          + ",";
    json += "\"az\":"     + String(az, 2)          + ",";
    json += "\"gx\":"     + String(gx, 2)          + ",";
    json += "\"gy\":"     + String(gy, 2)          + ",";
    json += "\"gz\":"     + String(gz, 2)          + ",";
    json += "\"roll\":"   + String(roll, 1)        + ",";
    json += "\"pitch\":"  + String(pitch, 1);
    json += "}";

    server.send(200, "application/json", json);
}

void handleNotFound() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Endpoint Not Found");
}

// ==========================================
// 5. Setup & Execution Loop
// ==========================================
void setup() {
    Serial.begin(115200);
    initMPU6050();

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWi-Fi Connected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());

    // Configure Routes
    server.on("/data", handleData);
    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("HTTP Data Server Started.");
}

void loop() {
    server.handleClient();
}
    