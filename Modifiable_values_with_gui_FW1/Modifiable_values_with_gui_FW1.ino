#include <Wire.h>
#include <EEPROM.h>
#include <ESP32Servo.h>

#define MPU6050_I2C_ADDRESS 0x68

// EEPROM configuration
#define EEPROM_SIZE 12
#define ACCEL_FILTER_ADDR 0
#define GYRO_FILTER_ADDR 4
#define COMP_FILTER_ADDR 8

// Default filter parameters
float ACCEL_FILTER = 0.3;
float GYRO_FILTER = 0.08;
float COMP_FILTER = 0.7;
float FREQ = 50.0;

// Hardware pins
const int ledPin = 2;
const int servoPitchPin = 13;
const int servoRollPin = 12;
const int servoYawPin = 14;

// Servo configuration
Servo servoPitch;
Servo servoRoll;
Servo servoYaw;
const int SERVO_MIN_US = 1000;
const int SERVO_MAX_US = 2000;
const int SERVO_NEUTRAL = 1500;

// Sensor variables
double gSensitivity = 65.5;
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
double gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;
int16_t accX = 0, accY = 0, accZ = 0;

// Filtered values
double filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
double filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// Connection tracking
unsigned long lastSerialActivity = 0;
const unsigned long SERIAL_TIMEOUT = 2000;
bool standaloneMode = true;

void setup() {
  Serial.begin(38400);
  pinMode(ledPin, OUTPUT);
  
  // Initialize servos with proper timer allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  servoPitch.setPeriodHertz(50);
  servoRoll.setPeriodHertz(50);
  servoYaw.setPeriodHertz(50);
  
  servoPitch.attach(servoPitchPin, SERVO_MIN_US, SERVO_MAX_US);
  servoRoll.attach(servoRollPin, SERVO_MIN_US, SERVO_MAX_US);
  servoYaw.attach(servoYawPin, SERVO_MIN_US, SERVO_MAX_US);
  
  // Center servos
  servoPitch.writeMicroseconds(SERVO_NEUTRAL);
  servoRoll.writeMicroseconds(SERVO_NEUTRAL);
  servoYaw.writeMicroseconds(SERVO_NEUTRAL);
  
  // Initialize I2C and EEPROM
  Wire.begin(21, 22);
  EEPROM.begin(EEPROM_SIZE);
  loadParameters();
  
  // Configure MPU6050
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x6B, 0x00);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1A, 0x06);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1B, 0x08);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1C, 0x08);
  
  uint8_t sample_div = (1000 / FREQ) - 1;
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x19, sample_div);
  
  // Calibrate gyro
  calibrate();
}

void loop() {
  Serial.print("Inside loop\n");
  static unsigned long last_time = millis();
  static unsigned long lastBlink = 0;
  
  // Handle connection state
  if (millis() - lastSerialActivity > SERIAL_TIMEOUT) {
    if (!standaloneMode) {
      standaloneMode = true;
      loadParameters(); // Revert to EEPROM values
    }
  }
  
  // LED status indication
  unsigned long blinkInterval = standaloneMode ? 200 : 1000;
  if (millis() - lastBlink > blinkInterval) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    lastBlink = millis();
  }

  // Handle serial commands
  checkSerialCommands();
  
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
  
  // Update servos
  updateServos();

  // Maintain timing
  while (millis() - last_time < (1000 / FREQ)) {
    delay(1);
  }
  last_time = millis();
}

void checkSerialCommands() {
  while (Serial.available()) {
    lastSerialActivity = millis();
    standaloneMode = false;
    
    char cmd = Serial.read();
    
    if (cmd == '.') {  // Print current angles
      Serial.print(gx, 2); Serial.print(", ");
      Serial.print(gy, 2); Serial.print(", ");
      Serial.println(gz, 2);
    }
    else if (cmd == 'z') {  // Zero yaw
      gz = 0;
    }
    else if (cmd == 'c') {  // Calibrate gyro
      calibrate();
    }
    else if (cmd == 'p') {  // Receive new parameters
      String params = Serial.readStringUntil('\n');
      int comma1 = params.indexOf(',');
      int comma2 = params.indexOf(',', comma1+1);
      
      if (comma1 != -1 && comma2 != -1) {
        ACCEL_FILTER = params.substring(0, comma1).toFloat();
        GYRO_FILTER = params.substring(comma1+1, comma2).toFloat();
        COMP_FILTER = params.substring(comma2+1).toFloat();
      }
    }
    else if (cmd == 'f') {  // Save to EEPROM
      saveParameters();
      // Visual confirmation blink
      for (int i = 0; i < 3; i++) {
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
      }
    }
    else if (cmd == '?') {  // Report current parameters
      Serial.print("params:");
      Serial.print(ACCEL_FILTER, 4);
      Serial.print(",");
      Serial.print(GYRO_FILTER, 4);
      Serial.print(",");
      Serial.println(COMP_FILTER, 4);
    }
    else if (cmd == 'x') {  // Custom signal to return to standalone mode
      standaloneMode = true;
      lastSerialActivity = 0;  // Reset timer
      Serial.end();           // Optional: close serial to prevent stalling
    }
  }
}

void updateServos() {
  // Map angles to servo microseconds
  int pitchUs = map(gy, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
  int rollUs = map(gx, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
  int yawUs = map(gz, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
  
  // Constrain to valid servo range
  pitchUs = constrain(pitchUs, SERVO_MIN_US, SERVO_MAX_US);
  rollUs = constrain(rollUs, SERVO_MIN_US, SERVO_MAX_US);
  yawUs = constrain(yawUs, SERVO_MIN_US, SERVO_MAX_US);
  
  // Write to servos
  servoPitch.writeMicroseconds(pitchUs);
  servoRoll.writeMicroseconds(rollUs);
  servoYaw.writeMicroseconds(yawUs);
}

void loadParameters() {
  // Try reading multiple times if needed
  for (int i = 0; i < 3; i++) {
    EEPROM.get(ACCEL_FILTER_ADDR, ACCEL_FILTER);
    EEPROM.get(GYRO_FILTER_ADDR, GYRO_FILTER);
    EEPROM.get(COMP_FILTER_ADDR, COMP_FILTER);
    
    // Validate loaded values
    if (!isnan(ACCEL_FILTER) && ACCEL_FILTER > 0 && ACCEL_FILTER <= 1.0 &&
        !isnan(GYRO_FILTER) && GYRO_FILTER > 0 && GYRO_FILTER <= 1.0 &&
        !isnan(COMP_FILTER) && COMP_FILTER > 0 && COMP_FILTER <= 1.0) {
      return;
    }
    delay(10);
  }
  
  // Fallback to defaults if still invalid
  ACCEL_FILTER = 0.3;
  GYRO_FILTER = 0.08;
  COMP_FILTER = 0.7;
}

void saveParameters() {
  EEPROM.put(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.put(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.put(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.commit();
}

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

void read_sensor_data() {
  uint8_t i2cData[14];
  if (i2c_read(MPU6050_I2C_ADDRESS, 0x3B, i2cData, 14) != 0) return;
  
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
  gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
  gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;
}

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
