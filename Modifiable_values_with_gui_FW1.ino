#include <Wire.h>
#include <EEPROM.h>

#define MPU6050_I2C_ADDRESS 0x68

// EEPROM addresses for parameters
#define EEPROM_SIZE 12  // 3 floats (4 bytes each)
#define ACCEL_FILTER_ADDR 0
#define GYRO_FILTER_ADDR 4
#define COMP_FILTER_ADDR 8

// Default filter parameters
float ACCEL_FILTER = 0.3;
float GYRO_FILTER = 0.08;
float COMP_FILTER = 0.7;
float FREQ = 50.0;

// ESP32 I2C pins
const int ledPin = 2;

// Sensor variables
double gSensitivity = 65.5;
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
double gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;
int16_t accX = 0, accY = 0, accZ = 0;

// Filtered values
double filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
double filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

void setup() {
  Serial.begin(38400);
  pinMode(ledPin, OUTPUT);
  
  Wire.begin(21, 22);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load saved parameters
  loadParameters();
  
  // Initialize MPU6050
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x6b, 0x00);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1a, 0x06);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1c, 0x08);
  
  uint8_t sample_div = (1000 / FREQ) - 1;
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x19, sample_div);
  
  digitalWrite(ledPin, HIGH);
  calibrate();
  digitalWrite(ledPin, LOW);
}

void loop() {
  static unsigned long last_time = millis();
  
  // Check for incoming serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == '.') {
      // Send current angles
      Serial.print(gx, 2); Serial.print(", ");
      Serial.print(gy, 2); Serial.print(", ");
      Serial.println(gz, 2);
    }
    else if (cmd == 'z') {
      // Reset yaw angle
      gz = 0;
    }
    else if (cmd == 'c') {
      // Recalibrate
      digitalWrite(ledPin, HIGH);
      calibrate();
      digitalWrite(ledPin, LOW);
    }
    else if (cmd == 'p') {
      // Parameter update: p0.30,0.08,0.70
      String params = Serial.readStringUntil('\n');
      int comma1 = params.indexOf(',');
      int comma2 = params.indexOf(',', comma1+1);
      
      if (comma1 != -1 && comma2 != -1) {
        ACCEL_FILTER = params.substring(0, comma1).toFloat();
        GYRO_FILTER = params.substring(comma1+1, comma2).toFloat();
        COMP_FILTER = params.substring(comma2+1).toFloat();
      }
    }
    else if (cmd == 'f') {
      // Flash parameters to EEPROM
      saveParameters();
      // Visual confirmation
      for (int i = 0; i < 3; i++) {
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
      }
    }
    else if (cmd == '?') {
      // Send current parameters
      Serial.print("params:");
      Serial.print(ACCEL_FILTER, 4);
      Serial.print(",");
      Serial.print(GYRO_FILTER, 4);
      Serial.print(",");
      Serial.println(COMP_FILTER, 4);
    }
  }
  
  // Read and process sensor data
  read_sensor_data();
  
  // Apply filters
  filtered_ax = filtered_ax * (1.0 - ACCEL_FILTER) + accX * ACCEL_FILTER;
  filtered_ay = filtered_ay * (1.0 - ACCEL_FILTER) + accY * ACCEL_FILTER;
  filtered_az = filtered_az * (1.0 - ACCEL_FILTER) + accZ * ACCEL_FILTER;
  
  filtered_gx = filtered_gx * (1.0 - GYRO_FILTER) + gyrX * GYRO_FILTER;
  filtered_gy = filtered_gy * (1.0 - GYRO_FILTER) + gyrY * GYRO_FILTER;
  filtered_gz = filtered_gz * (1.0 - GYRO_FILTER) + gyrZ * GYRO_FILTER;
  
  // Calculate angles
  double ay = atan2(filtered_ax, sqrt(pow(filtered_ay, 2) + pow(filtered_az, 2))) * 180 / M_PI;
  double ax = atan2(filtered_ay, sqrt(pow(filtered_ax, 2) + pow(filtered_az, 2))) * 180 / M_PI;
  
  // Integrate gyro rates
  gx = gx + filtered_gx / FREQ;
  gy = gy - filtered_gy / FREQ;
  gz = gz + filtered_gz / FREQ;
  
  // Apply complementary filter
  gx = gx * (1.0 - COMP_FILTER) + ax * COMP_FILTER;
  gy = gy * (1.0 - COMP_FILTER) + ay * COMP_FILTER;
  
  // Maintain consistent loop timing
  while (millis() - last_time < (1000 / FREQ)) {
    delay(1);
  }
  last_time = millis();
}

void loadParameters() {
  // Read from EEPROM
  EEPROM.get(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.get(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.get(COMP_FILTER_ADDR, COMP_FILTER);
  
  // Validate read values
  if (isnan(ACCEL_FILTER) || ACCEL_FILTER <= 0 || ACCEL_FILTER > 1.0) ACCEL_FILTER = 0.3;
  if (isnan(GYRO_FILTER) || GYRO_FILTER <= 0 || GYRO_FILTER > 1.0) GYRO_FILTER = 0.08;
  if (isnan(COMP_FILTER) || COMP_FILTER <= 0 || COMP_FILTER > 1.0) COMP_FILTER = 0.7;
}

void saveParameters() {
  // Write to EEPROM
  EEPROM.put(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.put(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.put(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.commit();
}

// Calibration function
void calibrate() {
  int num = 500;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t i2cData[6];
  
  for (int x = 0; x < num; x++) {
    if (i2c_read(MPU6050_I2C_ADDRESS, 0x43, i2cData, 6) != 0) return;
    xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);
    delay(2);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;
}

// Sensor reading function
void read_sensor_data() {
  uint8_t i2cData[14];
  if (i2c_read(MPU6050_I2C_ADDRESS, 0x3b, i2cData, 14) != 0) return;
  
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
  gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
  gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;
}

// I2C helper functions
int i2c_read(int addr, int start, uint8_t *buffer, int size) {
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
