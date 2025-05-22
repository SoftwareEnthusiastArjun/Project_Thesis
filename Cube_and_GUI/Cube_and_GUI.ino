#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>

// Function Prototypes
void loadParameters();
void saveParameters();
void calibrateMPU6050();
void initMPU6050();
void updateMPU6050();
int i2c_read(int addr, int start, uint8_t* buffer, int size);
int i2c_write_reg(int addr, int reg, uint8_t data);

// EEPROM & Filter Settings
#define EEPROM_SIZE 12
#define ACCEL_FILTER_ADDR 0
#define GYRO_FILTER_ADDR 4
#define COMP_FILTER_ADDR 8

const int ledPin = 2;

float ACCEL_FILTER = 0.3;
float GYRO_FILTER = 0.08;
float COMP_FILTER = 0.7;

// MPU6050
#define MPU6050_I2C_ADDRESS 0x68
float FREQ = 50.0;
double gSensitivity = 65.5;
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
double gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;
int16_t accX = 0, accY = 0, accZ = 0;
double filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
double filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// WiFi
const char* ssid = "aju";
const char* password = "@ajujcd@";
WiFiServer server(12345);

// PWM Pins
#define PITCH_IP 15
#define ROLL_IP 16
#define YAW_IP 17
#define AUTO_PILOT 18

#define MIN_PULSE_WIDTH 999
#define MAX_PULSE_WIDTH 1993
#define PULSE_TIMEOUT 25000

int lastPercentage1 = -1, lastPercentage2 = -1, lastPercentage3 = -1, lastPercentage4 = -1;
unsigned long lastMPUTime = 0;

bool cubeStreaming = false;
bool inside=false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStarting...");

  pinMode(ledPin, OUTPUT);

  EEPROM.begin(EEPROM_SIZE);
  Wire.begin(21, 22);
  loadParameters();
  calibrateMPU6050();
  initMPU6050();

  pinMode(PITCH_IP, INPUT);
  pinMode(ROLL_IP, INPUT);
  pinMode(YAW_IP, INPUT);
  pinMode(AUTO_PILOT, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("mDNS responder started: esp32.local");
  }

  server.begin();
  Serial.println("TCP server started on port 12345");
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Client connected");
    client.setTimeout(2);

    bool inStreamingMode = false;
    unsigned long lastSent = 0;

    while (client.connected()) {
      if (client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();
        Serial.print("Received: ");
        Serial.println(command);

        if (command == "get") {
          String response = String(ACCEL_FILTER, 3) + "," + String(GYRO_FILTER, 3) + "," + String(COMP_FILTER, 3);
          client.println(response);
        } else if (command.startsWith("setA")) {
          ACCEL_FILTER = command.substring(4).toFloat();
          client.println("OK");
        } else if (command.startsWith("setG")) {
          GYRO_FILTER = command.substring(4).toFloat();
          client.println("OK");
        } else if (command.startsWith("setC")) {
          COMP_FILTER = command.substring(4).toFloat();
          client.println("OK");
        } else if (command == "save") {
          saveParameters();
          client.println("OK");
        } else if (command == "startPWMStream") {
          inStreamingMode = true;
          client.println("PWM_STREAM_START");
        } else if (command == "startCubeStream") {
          cubeStreaming = true;
          inStreamingMode = false;  // Disable PWM stream
          client.println("CUBE_STREAM_START");
        } else if (command == "stopCubeStream") {
          cubeStreaming = false;
          client.println("CUBE_STREAM_STOPPED");
        } else {
          client.println("ERR");
        }
      }

      // Handle PWM streaming (when inStreamingMode is true)
      if (inStreamingMode) {
        uint32_t pulseWidth1 = pulseIn(PITCH_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth2 = pulseIn(ROLL_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth3 = pulseIn(YAW_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth4 = pulseIn(AUTO_PILOT, HIGH, PULSE_TIMEOUT);

        int p1 = constrain(map(pulseWidth1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
        int p2 = constrain(map(pulseWidth2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
        int p3 = constrain(map(pulseWidth3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
        int p4 = constrain(map(pulseWidth4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);

        if (p1 != lastPercentage1 || p2 != lastPercentage2 || p3 != lastPercentage3 || p4 != lastPercentage4) {
          client.printf("%d,%d,%d,%d\n", p1, p2, p3, p4);
          lastPercentage1 = p1;
          lastPercentage2 = p2;
          lastPercentage3 = p3;
          lastPercentage4 = p4;
          lastSent = millis();
        }

        if (millis() - lastSent > 2000) {
          client.println("No signal");
          lastSent = millis();
        }
      }
     // for streaming to py code
      if (cubeStreaming && millis() - lastMPUTime >= (1000 / FREQ)) {
        updateMPU6050();
        gz=0;
        client.printf("%.2f,%.2f,%.2f\n", gx, gy, gz);
        lastMPUTime = millis();
      }
      
    }

    client.stop();
    lastPercentage1 = lastPercentage2 = lastPercentage3 = lastPercentage4 = -1;
  }

  delay(10);
}

// -------------------- MPU6050 Functions --------------------

void initMPU6050() {
  Serial.println("Initiating MPU6050 sensor...");
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x6b, 0x00);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1a, 0x06);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1c, 0x08);
  uint8_t sample_div = (1000 / FREQ) - 1;
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x19, sample_div);
}

void calibrateMPU6050() {
  int num = 500;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t data[6];

  for (int i = 0; i < num; i++) {
    if (i2c_read(MPU6050_I2C_ADDRESS, 0x43, data, 6) != 0) return;
    xSum += ((data[0] << 8) | data[1]);
    ySum += ((data[2] << 8) | data[3]);
    zSum += ((data[4] << 8) | data[5]);
    delay(2);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;
}

void updateMPU6050() {
  static unsigned long last_time = millis();
  uint8_t data[14];

  if (i2c_read(MPU6050_I2C_ADDRESS, 0x3b, data, 14) != 0) return;

  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);

  gyrX = (((data[8] << 8) | data[9]) - gyrXoffs) / gSensitivity;
  gyrY = (((data[10] << 8) | data[11]) - gyrYoffs) / gSensitivity;
  gyrZ = (((data[12] << 8) | data[13]) - gyrZoffs) / gSensitivity;

  filtered_ax = filtered_ax * (1.0 - ACCEL_FILTER) + accX * ACCEL_FILTER;
  filtered_ay = filtered_ay * (1.0 - ACCEL_FILTER) + accY * ACCEL_FILTER;
  filtered_az = filtered_az * (1.0 - ACCEL_FILTER) + accZ * ACCEL_FILTER;

  filtered_gx = filtered_gx * (1.0 - GYRO_FILTER) + gyrX * GYRO_FILTER;
  filtered_gy = filtered_gy * (1.0 - GYRO_FILTER) + gyrY * GYRO_FILTER;
  filtered_gz = filtered_gz * (1.0 - GYRO_FILTER) + gyrZ * GYRO_FILTER;

  double ay = atan2(filtered_ax, sqrt(pow(filtered_ay, 2) + pow(filtered_az, 2))) * 180 / M_PI;
  double ax = atan2(filtered_ay, sqrt(pow(filtered_ax, 2) + pow(filtered_az, 2))) * 180 / M_PI;

  gx += filtered_gx / FREQ;
  gy -= filtered_gy / FREQ;
  gz += filtered_gz / FREQ;

  gx = gx * (1.0 - COMP_FILTER) + ax * COMP_FILTER;
  gy = gy * (1.0 - COMP_FILTER) + ay * COMP_FILTER;

  while (millis() - last_time < (1000 / FREQ)) delay(1);
  last_time = millis();
}

// -------------------- EEPROM --------------------

void loadParameters() {
  EEPROM.get(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.get(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.get(COMP_FILTER_ADDR, COMP_FILTER);

  if (isnan(ACCEL_FILTER) || ACCEL_FILTER <= 0 || ACCEL_FILTER > 1.0) ACCEL_FILTER = 0.3;
  if (isnan(GYRO_FILTER) || GYRO_FILTER <= 0 || GYRO_FILTER > 1.0) GYRO_FILTER = 0.08;
  if (isnan(COMP_FILTER) || COMP_FILTER <= 0 || COMP_FILTER > 1.0) COMP_FILTER = 0.7;
}

void saveParameters() {
  EEPROM.put(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.put(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.put(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.commit();
}

// -------------------- I2C Helpers --------------------

int i2c_read(int addr, int start, uint8_t* buffer, int size) {
  Wire.beginTransmission(addr);
  Wire.write(start);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom(addr, size, true);
  int i = 0;
  while (Wire.available() && i < size) buffer[i++] = Wire.read();
  return (i == size) ? 0 : -1;
}

int i2c_write_reg(int addr, int reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  return Wire.endTransmission(true);
}