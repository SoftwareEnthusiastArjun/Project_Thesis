/*
 * ESP32-based Flight Controller with MPU6050 IMU
 * Features:
 * - PID stabilization for pitch and roll axes
 * - Manual override capability via PWM inputs
 * - WiFi configuration interface
 * - EEPROM parameter storage
 * - Complementary filter for sensor fusion
 */

// Include necessary libraries
#include <Wire.h>          // I2C communication
#include <WiFi.h>          // WiFi connectivity
#include <WiFiClient.h>    // TCP client
#include <WiFiServer.h>    // TCP server
#include <ESPmDNS.h>       // mDNS for network discovery
#include <EEPROM.h>        // Non-volatile storage
#include <ESP32Servo.h>    // Servo control

// EEPROM & Filter Settings
#define EEPROM_SIZE 36     // Increased size to store PID parameters (3 floats per PID x 2 axes = 24 bytes + filter values)
#define ACCEL_FILTER_ADDR 0    // EEPROM address for accelerometer filter
#define GYRO_FILTER_ADDR 4     // EEPROM address for gyro filter
#define COMP_FILTER_ADDR 8     // EEPROM address for complementary filter
#define PITCH_PID_ADDR 12      // 12 bytes for pitch PID (Kp, Ki, Kd as floats)
#define ROLL_PID_ADDR 24       // 12 bytes for roll PID (Kp, Ki, Kd as floats)
#define M_PI 3.14159265358979323846  // Definition of π for calculations

const int ledPin = 2;      // Built-in LED pin for status indication

// Default filter values (will be overwritten by EEPROM if available)
float ACCEL_FILTER = 0.3;  // Low-pass filter factor for accelerometer
float GYRO_FILTER = 0.08;  // Low-pass filter factor for gyro
float COMP_FILTER = 0.7;   // Complementary filter factor (gyro vs accel)

// PID Controller Structure
struct PID {
  float Kp;                // Proportional gain
  float Ki;                // Integral gain
  float Kd;                // Derivative gain
  float integral;          // Accumulated integral term
  float previous_error;    // Previous error for derivative calculation
  unsigned long last_time; // Last update time for delta-T calculation
};

// PID Controllers initialization with default values
PID pitchPID = {2.0, 0.1, 0.5, 0.0, 0.0, 0};  // Pitch axis PID
PID rollPID = {2.0, 0.1, 0.5, 0.0, 0.0, 0};    // Roll axis PID

// Function Prototypes
void loadParameters();      // Load settings from EEPROM
void saveParameters();      // Save settings to EEPROM
void calibrateMPU6050();    // Calibrate gyro offsets
void initMPU6050();         // Initialize MPU6050 sensor
void updateMPU6050();       // Read and process sensor data
void updateServoFromMPU();  // Update servos based on IMU data
int i2c_read(int addr, int start, uint8_t* buffer, int size);  // I2C read helper
int i2c_write_reg(int addr, int reg, uint8_t data);            // I2C write helper
void resetPID(PID &pid);    // Reset PID controller state

/*
 * Computes PID output with bumpless transfer to prevent sudden jumps
 * when switching between manual and automatic modes
 * Parameters:
 *   pid - PID controller instance
 *   setpoint - Desired value (typically 0 for stabilization)
 *   input - Current measured value
 *   currentOutput - Current actuator position for bumpless transfer
 * Returns:
 *   Computed PID output
 */
float computePID(PID& pid, float setpoint, float input, float currentOutput = 0) {
  unsigned long now = millis();
  float dt = (now - pid.last_time) / 1000.0;  // Convert to seconds
  if (dt <= 0) dt = 0.001;  // Prevent division by zero erro
  
  float error = setpoint - input;  // Calculate error
  
  // Bumpless transfer: adjust integral to match current output
  if (pid.Ki != 0) {
    pid.integral = (currentOutput - pid.Kp * error - pid.Kd * (error - pid.previous_error)/dt) / pid.Ki;
  }
  
  // Update integral term with anti-windup
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -50, 50);  // Limit integral windup
  
  // Calculate derivative term
  float derivative = (error - pid.previous_error) / dt;
  
  // Compute PID output
  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  
  // Update state variables
  pid.previous_error = error;
  pid.last_time = now;
  
  return output;
}

/*
 * Resets PID controller state
 * Parameters:
 *   pid - PID controller instance to reset
 */
void resetPID(PID &pid) {
  pid.integral = 0;
  pid.previous_error = 0;
  pid.last_time = millis();
}

// MPU6050 Configuration
#define MPU6050_I2C_ADDRESS 0x68  // Default I2C address of MPU6050
float FREQ = 50.0;                // Sampling frequency (Hz)
double gSensitivity = 65.5;       // Gyro sensitivity (LSB/°/sec)
// Raw and filtered sensor data variables
double gx = 0, gy = 0, gz = 0;    // Filtered angles (degrees)
double gyrX = 0, gyrY = 0, gyrZ = 0;  // Raw gyro readings
double gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;  // Gyro offsets
int16_t accX = 0, accY = 0, accZ = 0;  // Raw accelerometer readings
// Filtered sensor values
double filtered_ax = 0, filtered_ay = 0, filtered_az = 0;  // Filtered accelerometer
double filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;  // Filtered gyro

// WiFi Configuration
const char* ssid = "aju";         // WiFi SSID
const char* password = "@ajujcd@"; // WiFi password
WiFiServer server(12345);         // TCP server on port 12345

// PWM Input Configuration
#define PITCH_IP 15    // Pitch input pin
#define ROLL_IP 16     // Roll input pin
#define YAW_IP 17      // Yaw input pin
#define AUTO_PILOT 18  // Auto-pilot mode switch pin

// PWM signal parameters
#define MIN_PULSE_WIDTH 999    // Minimum expected pulse width (µs)
#define MAX_PULSE_WIDTH 1993   // Maximum expected pulse width (µs)
#define PULSE_TIMEOUT 25000    // Timeout for pulse reading (µs); if no pulse is detected within 25 milliseconds(25000µs), pulseIn() returns 0.

// Servo Output Configuration
#define PITCH_SERVO_PIN 25  // Pitch servo output pin
#define ROLL_SERVO_PIN 26   // Roll servo output pin

//Servo is a class provided by the ESP32Servo.h library.
Servo pitchServo;  // Pitch axis servo
Servo rollServo;   // Roll axis servo

// Variables for tracking PWM input states
int lastPercentage1 = -1, lastPercentage2 = -1, lastPercentage3 = -1, lastPercentage4 = -1;
unsigned long lastMPUTime = 0;  // Last IMU update time

// Streaming flags
bool cubeStreaming = false;     // 3D cube visualization streaming flag
bool inside = false;            // Not used in current code
bool lastStabState = false;     // Track last stabilization state for mode transitions

/*
 * Setup function - runs once at startup
 */
void setup() {
  Serial.begin(115200);  // Initialize serial communication
  delay(1000);           // Wait for serial to stabilize
  Serial.println("\nEntered Setup...");

  pinMode(ledPin, OUTPUT);  // Configure LED pin

  // Initialize subsystems
  EEPROM.begin(EEPROM_SIZE);  // Initialize EEPROM with specified size
  Wire.begin(21, 22);         // Initialize I2C on pins 21 (SDA), 22 (SCL)
  loadParameters();           // Load parameters from EEPROM
  calibrateMPU6050();         // Calibrate gyro offsets
  initMPU6050();              // Configure MPU6050

  // Configure PWM input pins
  pinMode(PITCH_IP, INPUT);
  pinMode(ROLL_IP, INPUT);
  pinMode(YAW_IP, INPUT);
  pinMode(AUTO_PILOT, INPUT);

  // Attach servos to pins
  pitchServo.attach(PITCH_SERVO_PIN);
  rollServo.attach(ROLL_SERVO_PIN);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);  // Station mode (connect to WiFi)
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  // WiFi connection timeout handling
  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 10000; // 10 second timeout
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    Serial.print(".");
    delay(500);
  }
  
  // WiFi connection success handling
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  
    // Start mDNS responder for easy network discovery
    if (MDNS.begin("esp32")) {
      Serial.println("mDNS responder started: esp32.local");
    }
  
    server.begin();  // Start TCP server
    Serial.println("TCP server started on port 12345");
  } else {
    Serial.println("\nWiFi connection failed. Continuing without network.");
  }
  Serial.println("\nExiting Setup...");
}

/*
 * Main loop - runs continuously after setup
 */
void loop() {
  WiFiClient client = server.available();  // Check for incoming TCP connections

  // Main control loop running at specified frequency(once every 20ms for 50Hz)
  if (millis() - lastMPUTime >= (1000 / FREQ)) {
    lastMPUTime = millis();
    
    // Read PWM inputs with timeout
    uint32_t pulseWidth1 = pulseIn(PITCH_IP, HIGH, PULSE_TIMEOUT);
    uint32_t pulseWidth2 = pulseIn(ROLL_IP, HIGH, PULSE_TIMEOUT);
    uint32_t pulseWidth3 = pulseIn(YAW_IP, HIGH, PULSE_TIMEOUT);
    uint32_t pulseWidth4 = pulseIn(AUTO_PILOT, HIGH, PULSE_TIMEOUT);

    // Map pulse widths to percentage (0-100)
    int p1 = constrain(map(pulseWidth1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p2 = constrain(map(pulseWidth2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p3 = constrain(map(pulseWidth3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p4 = constrain(map(pulseWidth4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);

    // Add deadband around center (3% tolerance)
    const int DEADBAND = 3;
    if (abs(p1 - 50) < DEADBAND) p1 = 50;
    if (abs(p2 - 50) < DEADBAND) p2 = 50;
    if (abs(p3 - 50) < DEADBAND) p3 = 50;

    // Determine stabilization mode
    bool isStabilizationActive = (p4 > 90);
    // Map stick positions to angle ranges
    int manualRollAngle = map(p1, 0, 100, 45, 135);    // 45-135° range
    int manualPitchAngle = map(p2, 0, 100, 45, 135);   // 45-135° range

    // Handle stabilization mode transitions
    if (isStabilizationActive != lastStabState) {
      if (isStabilizationActive) {
        resetPID(rollPID);  // Reset PID when enabling stabilization
        Serial.println("Stabilization ON");
      } else {
        Serial.println("Stabilization OFF");
      }
      lastStabState = isStabilizationActive;
    }

    // Stabilization mode logic
    if (isStabilizationActive) {
      if (abs(p1 - 50) <= 5 && abs(p2 - 50) <= 5) {
        // Both sticks centered - use full stabilization
        updateServoFromMPU();
      } else {
        // Partial manual override for non-centered axes
        if (abs(p1 - 50) > 5) {
          resetPID(rollPID);
          rollServo.write(manualRollAngle);
        }
        if (abs(p2 - 50) > 5) {
          resetPID(pitchPID);
          pitchServo.write(manualPitchAngle);
        }
      }
    } else {
      // Full manual mode
      resetPID(rollPID);
      resetPID(pitchPID);
      rollServo.write(manualRollAngle);
      pitchServo.write(manualPitchAngle);
    }

    // Update last percentage values
    lastPercentage1 = p1;
    lastPercentage2 = p2;
    lastPercentage3 = p3;
    lastPercentage4 = p4;
  }

  // TCP client handling
  if (client) {
    Serial.println("Client connected");
    client.setTimeout(2);  // Short timeout for commands

    bool inStreamingMode = false;  // PWM streaming flag
    unsigned long lastSent = 0;    // Last data sent time

    // Client connection loop
    while (client.connected()) {
      // Maintain control loop timing while client connected
      if (millis() - lastMPUTime >= (1000 / FREQ)) {
        lastMPUTime = millis();
        updateServoFromMPU();
      }
      
      // Process incoming commands
      if (client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();
        Serial.print("Received: ");
        Serial.println(command);

        // Command processing
        if (command == "get") {
          // Return current filter values
          String response = String(ACCEL_FILTER, 3) + "," + String(GYRO_FILTER, 3) + "," + String(COMP_FILTER, 3);
          client.println(response);
          Serial.println(response);
        } 
        else if (command == "getPitchPID") {
          // Return pitch PID values
          String response = String(pitchPID.Kp, 3) + "," + String(pitchPID.Ki, 3) + "," + String(pitchPID.Kd, 3);
          client.println(response);
          Serial.println(response);
        }
        else if (command == "getRollPID") {
          // Return roll PID values
          String response = String(rollPID.Kp, 3) + "," + String(rollPID.Ki, 3) + "," + String(rollPID.Kd, 3);
          client.println(response);
          Serial.println(response);
        }
        // Parameter setting commands
        else if (command.startsWith("setA")) {
          ACCEL_FILTER = command.substring(4).toFloat();
          client.println("OK");
        } else if (command.startsWith("setG")) {
          GYRO_FILTER = command.substring(4).toFloat();
          client.println("OK");
        } else if (command.startsWith("setC")) {
          COMP_FILTER = command.substring(4).toFloat();
          client.println("OK");
        } 
        // PID parameter setting commands
        else if (command.startsWith("setPitchP")) {
          pitchPID.Kp = command.substring(9).toFloat();
          client.println("OK");
        }
        else if (command.startsWith("setPitchI")) {
          pitchPID.Ki = command.substring(9).toFloat();
          client.println("OK");
        }
        else if (command.startsWith("setPitchD")) {
          pitchPID.Kd = command.substring(9).toFloat();
          client.println("OK");
        }
        else if (command.startsWith("setRollP")) {
          rollPID.Kp = command.substring(8).toFloat();
          client.println("OK");
        }
        else if (command.startsWith("setRollI")) {
          rollPID.Ki = command.substring(8).toFloat();
          client.println("OK");
        }
        else if (command.startsWith("setRollD")) {
          rollPID.Kd = command.substring(8).toFloat();
          client.println("OK");
        }
        else if (command == "save") {
          saveParameters();  // Save parameters to EEPROM
          client.println("OK");
        } else if (command == "startPWMStream") {
          client.println("PWM_STREAM_START");
          inStreamingMode = true;
        } else if (command == "stopPWMStream") {
          inStreamingMode = false;
          client.println("PWM_STREAM_STOPPED");
        }else if (command == "startCubeStream") {
          inStreamingMode = false;
          client.println("CUBE_STREAM_START");
          cubeStreaming = true;
        } else if (command == "stopCubeStream") {
          client.println("CUBE_STREAM_STOPPED");
          cubeStreaming = false;
        } else {
          client.println("ERR");  // Unknown command
        }
      }

      // PWM streaming mode
      if (inStreamingMode) {
        uint32_t pulseWidth1 = pulseIn(PITCH_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth2 = pulseIn(ROLL_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth3 = pulseIn(YAW_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth4 = pulseIn(AUTO_PILOT, HIGH, PULSE_TIMEOUT);

        // Map to percentages
        int p1 = constrain(map(pulseWidth1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
        int p2 = constrain(map(pulseWidth2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
        int p3 = constrain(map(pulseWidth3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
        int p4 = constrain(map(pulseWidth4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);

        // Send updates only when values change
        if (p1 != lastPercentage1 || p2 != lastPercentage2 || p3 != lastPercentage3 || p4 != lastPercentage4) {
          client.printf("%d,%d,%d,%d\n", p1, p2, p3, p4);
          lastPercentage1 = p1;
          lastPercentage2 = p2;
          lastPercentage3 = p3;
          lastPercentage4 = p4;
          lastSent = millis();
        }

        // Send keepalive if no data sent recently
        if (millis() - lastSent > 2000) {
          client.println("No signal");
          lastSent = millis();
        }
      }

      // 3D cube visualization streaming
      if (cubeStreaming && millis() - lastMPUTime >= (1000 / FREQ)) {
        updateMPU6050();
        gz = 0;  // Zero out yaw for visualization
        client.printf("%.2f,%.2f,%.2f\n", gx, gy, gz);
        lastMPUTime = millis();
      }
    }

    // Client disconnected
    client.stop();
    lastPercentage1 = lastPercentage2 = lastPercentage3 = lastPercentage4 = -1;
  }

  delay(10);  // Small delay to prevent watchdog timer issues
}

//--------------------Servo Control--------------------------
/*
 * Updates servo positions based on IMU data using PID control
 */
void updateServoFromMPU() {
  updateMPU6050();  // Get latest sensor data

  // Get current servo positions for bumpless transfer
  float currentPitchPos = pitchServo.read();
  float currentRollPos = rollServo.read();

  // Calculate PID outputs with bumpless transfer
  float pitchOutput = computePID(pitchPID, 0, gy, currentPitchPos - 90);
  float rollOutput = computePID(rollPID, 0, gx, currentRollPos - 90);

  // Map PID outputs to servo angles (90° is center)
  int pitchAngle = constrain(90 + pitchOutput, 45, 135);
  int rollAngle = constrain(90 + rollOutput, 45, 135);

  // Update servos
  pitchServo.write(pitchAngle);
  rollServo.write(rollAngle);
}

// -------------------- MPU6050 Functions --------------------
/*
 * Initializes MPU6050 with appropriate settings
 */
void initMPU6050() {
  Serial.println("Initiating MPU6050 sensor...");
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x6b, 0x00);  // Wake up device
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1a, 0x06);  // Low-pass filter config
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);  // Gyro full-scale range (±500°/s)
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1c, 0x08);  // Accel full-scale range (±4g)
  uint8_t sample_div = (1000 / FREQ) - 1;          // Calculate sample rate divider
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x19, sample_div);  // Set sample rate
}

/*
 * Calibrates MPU6050 gyroscope by calculating offsets
 */
void calibrateMPU6050() {
  int num = 500;  // Number of samples for calibration
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t data[6];

  // Collect samples
  for (int i = 0; i < num; i++) {
    if (i2c_read(MPU6050_I2C_ADDRESS, 0x43, data, 6) != 0) return;
    xSum += ((data[0] << 8) | data[1]);
    ySum += ((data[2] << 8) | data[3]);
    zSum += ((data[4] << 8) | data[5]);
    delay(2);
  }
  // Calculate average offsets
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;
}

/*
 * Reads and processes MPU6050 data
 */
void updateMPU6050() {
  static unsigned long last_time = millis();
  uint8_t data[14];

  // Read all sensor data (accel, temp, gyro)
  if (i2c_read(MPU6050_I2C_ADDRESS, 0x3b, data, 14) != 0) return;

  // Extract accelerometer data
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);

  // Extract gyro data and apply offsets and sensitivity scaling
  gyrX = (((data[8] << 8) | data[9]) - gyrXoffs) / gSensitivity;
  gyrY = (((data[10] << 8) | data[11]) - gyrYoffs) / gSensitivity;
  gyrZ = (((data[12] << 8) | data[13]) - gyrZoffs) / gSensitivity;

  // Apply low-pass filters
  filtered_ax = filtered_ax * (1.0 - ACCEL_FILTER) + accX * ACCEL_FILTER;
  filtered_ay = filtered_ay * (1.0 - ACCEL_FILTER) + accY * ACCEL_FILTER;
  filtered_az = filtered_az * (1.0 - ACCEL_FILTER) + accZ * ACCEL_FILTER;

  filtered_gx = filtered_gx * (1.0 - GYRO_FILTER) + gyrX * GYRO_FILTER;
  filtered_gy = filtered_gy * (1.0 - GYRO_FILTER) + gyrY * GYRO_FILTER;
  filtered_gz = filtered_gz * (1.0 - GYRO_FILTER) + gyrZ * GYRO_FILTER;

  // Calculate angles from accelerometer
  double ay = atan2(filtered_ax, sqrt(pow(filtered_ay, 2) + pow(filtered_az, 2))) * 180 / M_PI;
  double ax = atan2(filtered_ay, sqrt(pow(filtered_ax, 2) + pow(filtered_az, 2))) * 180 / M_PI;

  // Integrate gyro rates to get angles
  gx += filtered_gx / FREQ;
  gy -= filtered_gy / FREQ;
  gz += filtered_gz / FREQ;

  // Apply complementary filter to combine accel and gyro
  gx = gx * (1.0 - COMP_FILTER) + ax * COMP_FILTER;
  gy = gy * (1.0 - COMP_FILTER) + ay * COMP_FILTER;

  // Maintain timing
  while (millis() - last_time < (1000 / FREQ)) delay(1);
  last_time = millis();
}

// -------------------- EEPROM --------------------
/*
 * Loads parameters from EEPROM with validation
 */
void loadParameters() {
  // Read values from EEPROM
  EEPROM.get(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.get(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.get(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.get(PITCH_PID_ADDR, pitchPID.Kp);
  EEPROM.get(PITCH_PID_ADDR + 4, pitchPID.Ki);
  EEPROM.get(PITCH_PID_ADDR + 8, pitchPID.Kd);
  EEPROM.get(ROLL_PID_ADDR, rollPID.Kp);
  EEPROM.get(ROLL_PID_ADDR + 4, rollPID.Ki);
  EEPROM.get(ROLL_PID_ADDR + 8, rollPID.Kd);

  // Validate loaded values and set defaults if invalid
  if (isnan(ACCEL_FILTER) || ACCEL_FILTER <= 0 || ACCEL_FILTER > 1.0) ACCEL_FILTER = 0.3;
  if (isnan(GYRO_FILTER) || GYRO_FILTER <= 0 || GYRO_FILTER > 1.0) GYRO_FILTER = 0.08;
  if (isnan(COMP_FILTER) || COMP_FILTER <= 0 || COMP_FILTER > 1.0) COMP_FILTER = 0.7;
  
  if (isnan(pitchPID.Kp) || pitchPID.Kp < 0) pitchPID.Kp = 2.0;
  if (isnan(pitchPID.Ki) || pitchPID.Ki < 0) pitchPID.Ki = 0.1;
  if (isnan(pitchPID.Kd) || pitchPID.Kd < 0) pitchPID.Kd = 0.5;
  
  if (isnan(rollPID.Kp) || rollPID.Kp < 0) rollPID.Kp = 2.0;
  if (isnan(rollPID.Ki) || rollPID.Ki < 0) rollPID.Ki = 0.1;
  if (isnan(rollPID.Kd) || rollPID.Kd < 0) rollPID.Kd = 0.5;
}

/*
 * Saves current parameters to EEPROM
 */
void saveParameters() {
  // Write all parameters to EEPROM
  EEPROM.put(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.put(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.put(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.put(PITCH_PID_ADDR, pitchPID.Kp);
  EEPROM.put(PITCH_PID_ADDR + 4, pitchPID.Ki);
  EEPROM.put(PITCH_PID_ADDR + 8, pitchPID.Kd);
  EEPROM.put(ROLL_PID_ADDR, rollPID.Kp);
  EEPROM.put(ROLL_PID_ADDR + 4, rollPID.Ki);
  EEPROM.put(ROLL_PID_ADDR + 8, rollPID.Kd);
  EEPROM.commit();  // Commit changes to flash
}

// -------------------- I2C Helpers --------------------
/*
 * Reads data from I2C device
 * Parameters:
 *   addr - Device address
 *   start - Starting register address
 *   buffer - Buffer to store read data
 *   size - Number of bytes to read
 * Returns:
 *   0 on success, -1 on failure
 */
int i2c_read(int addr, int start, uint8_t* buffer, int size) {
  Wire.beginTransmission(addr);
  Wire.write(start);
  if (Wire.endTransmission(false) != 0) return -1;  // Non-zero indicates error
  Wire.requestFrom(addr, size, true); 
  int i = 0;
  while (Wire.available() && i < size) buffer[i++] = Wire.read();
  return (i == size) ? 0 : -1;  // Return success only if all bytes read
}

/*
 * Writes a single byte to I2C device register
 * Parameters:
 *   addr - Device address
 *   reg - Register address
 *   data - Data byte to write
 * Returns:
 *   Result of endTransmission()
 */
int i2c_write_reg(int addr, int reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  return Wire.endTransmission(true);
}