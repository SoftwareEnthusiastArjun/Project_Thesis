// =======================================================
//                    INCLUDED LIBRARIES
// =======================================================
#include <Wire.h>         // For I2C communication with MPU6050
#include <WiFi.h>         // For connecting ESP32 to WiFi
#include <WiFiClient.h>   // For handling WiFi clients (like a phone or computer)
#include <WiFiServer.h>   // For setting up a TCP server on ESP32
#include <ESPmDNS.h>      // For mDNS (allows accessing ESP32 via name like esp32.local)
#include <EEPROM.h>       // For saving configuration data across reboots
#include <ESP32Servo.h>   // To control servo motors on ESP32

// =======================================================
//                 CONSTANTS & CONFIGURATION
// =======================================================

// ---------------- EEPROM Configuration ----------------
// Defines how much space EEPROM will use and where to store parameter
#define EEPROM_SIZE       36
#define ACCEL_FILTER_ADDR 0
#define GYRO_FILTER_ADDR  4
#define COMP_FILTER_ADDR  8
#define PITCH_PID_ADDR   12
#define ROLL_PID_ADDR    24

#define M_PI 3.14159265358979323846

// ---------------- MPU6050 Settings ---------------------
// gSensitivity is for converting raw gyro data into degrees/sec (depends on gyro config)
#define MPU6050_I2C_ADDRESS 0x68
float FREQ = 50.0;
double gSensitivity = 65.5; //?

// ---------------- PWM Input Pins -----------------------
// PWM Input pins
#define PITCH_IP     15
#define ROLL_IP      16
#define YAW_IP       17
#define AUTO_PILOT   18

// PWM pulse width boundaries (microseconds) used for signal normalization
#define MIN_PULSE_WIDTH 999
#define MAX_PULSE_WIDTH 1993
#define PULSE_TIMEOUT   25000 //?

// ---------------- Servo Output Pins --------------------
#define PITCH_SERVO_PIN 25
#define ROLL_SERVO_PIN  26

// ---------------- WiFi Settings -------------------------
const char* ssid     = "aju";
const char* password = "@ajujcd@";
WiFiServer server(12345);

// ---------------- LED Pin ------------------------------
const int ledPin = 2;

// =======================================================
//                    GLOBAL VARIABLES
// =======================================================

// ---------------- Filter and State ---------------------
// Filter coefficients control responsiveness vs. smoothness of sensor data.
float ACCEL_FILTER   = 0.3;
float GYRO_FILTER    = 0.08;
float COMP_FILTER    = 0.7;

// Tracks stabilization references and mode states
float referencePitch = 0;
float referenceRoll  = 0;
float pitchOffset    = 90;  // Baseline center
float rollOffset     = 90;
bool firstStabLoop   = true;
bool lastStabState   = false;
bool cubeStreaming   = false;
bool inside          = false;

// ---------------- IMU Data -----------------------------
// Raw and filtered accelerometer & gyroscope values.
// Offsets calculated during calibration.
// gx, gy, gz: estimated angles
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
double gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;

int16_t accX = 0, accY = 0, accZ = 0;

double filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
double filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

unsigned long lastMPUTime = 0;

// ---------------- PID Control --------------------------
// PID structure to hold gains and error history for pitch & roll.
struct PID {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float previous_error;
  unsigned long last_time;
};

PID pitchPID = {2.0, 0.1, 0.5, 0.0, 0.0, 0};
PID rollPID  = {2.0, 0.1, 0.5, 0.0, 0.0, 0};

// ---------------- Servo & Input ------------------------
// Servo objects for control
// Stores last known PWM input values for change detection
Servo pitchServo;
Servo rollServo;

int lastPercentage1 = -1, lastPercentage2 = -1;
int lastPercentage3 = -1, lastPercentage4 = -1;

// =======================================================
//                  FUNCTION PROTOTYPES
// =======================================================
void loadParameters();
void saveParameters();
void calibrateMPU6050();
void initMPU6050();
void updateMPU6050();
void updateServoFromMPU();

int i2c_read(int addr, int start, uint8_t* buffer, int size);
int i2c_write_reg(int addr, int reg, uint8_t data);

float computePID(PID& pid, float setpoint, float input, float currentOutput = 0);
void resetPID(PID &pid);

// =======================================================
//                        SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStarting...");

  pinMode(ledPin, OUTPUT);

  // Initialize EEPROM, I2C and load params
  EEPROM.begin(EEPROM_SIZE); // must be called before using EEPROM.read or .write or . commit
  Wire.begin(21, 22);  // SDA = 21, SCL = 22, to start I2C communication with IMU
  loadParameters(); //Reads saved filter and PID values from EEPROM and stores into variables. Also validates them to avoid corrupt values
  initMPU6050(); //Configures the MPU6050: Wakes it up from sleep mode, Sets sample rate divider, Configures filters and sensitivity
  calibrateMPU6050(); //Reads 500 samples from MPU6050 gyro to calculate offsets (bias correction). These offsets are subtracted from all future readings.

  // Configure input pins
  pinMode(PITCH_IP, INPUT);
  pinMode(ROLL_IP, INPUT);
  pinMode(YAW_IP, INPUT);
  pinMode(AUTO_PILOT, INPUT);

  // Connects the Servo library to the physical pins for servo control.
  pitchServo.attach(PITCH_SERVO_PIN); //ESP32Servo library starts PWM generation on that GPIO pin.
  rollServo.attach(ROLL_SERVO_PIN);

  // Connect to WiFi
  // Tries to connect to WiFi in station mode. If it doesn’t connect in 10 seconds, moves on.
  WiFi.mode(WIFI_STA); //The ESP32 acts like a client, connecting to an existing Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();//captures the start time of the connection attempt using millis()(returns time since boot in ms).
  const unsigned long wifiTimeout = 10000; // sets a 10-second timeout

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    Serial.print(".");
    delay(500); //waits for 10 sec, and prints "." every half seconds
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());//returns the local IP address that the ESP32 got from your Wi-Fi router

    if (MDNS.begin("esp32")) { //Starts mDNS (so you can connect via esp32.local).
      Serial.println("mDNS responder started: esp32.local");
    }

    server.begin(); //Starts a TCP server to accept commands.
    Serial.println("TCP server started on port 12345");
  } else {
    Serial.println("\nWiFi connection failed. Continuing without network.");
  }

  // Take initial sensor reading, Reads MPU6050 data once and sets initial reference angles for stabilization.
  updateMPU6050();
  referencePitch = gy;
  referenceRoll = gx;
}

void loop() {
  WiFiClient client = server.available();

  if (millis() - lastMPUTime >= (1000 / FREQ)) {//This block limits how often inputs and IMU data are read — eg FREQ=50 would mean every 20ms.
    lastMPUTime = millis();
    
    // Read PWM inputs with deadband
    uint32_t pulseWidth1 = pulseIn(PITCH_IP, HIGH, PULSE_TIMEOUT);//pulseIN readsPWM
    uint32_t pulseWidth2 = pulseIn(ROLL_IP, HIGH, PULSE_TIMEOUT);
    uint32_t pulseWidth3 = pulseIn(YAW_IP, HIGH, PULSE_TIMEOUT);
    uint32_t pulseWidth4 = pulseIn(AUTO_PILOT, HIGH, PULSE_TIMEOUT);

    //Each PWM value is mapped from microseconds(1000–2000µs) to 0–100%. Then constrained to stay in that range.
    int p1 = constrain(map(pulseWidth1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p2 = constrain(map(pulseWidth2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p3 = constrain(map(pulseWidth3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p4 = constrain(map(pulseWidth4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);

    // Add deadband around center(3% tolerance), If input is near center(e.g. 48–52),treat it as exactly center to prevent jittering from small fluctuations.
    const int DEADBAND = 3;
    if (abs(p1 - 50) < DEADBAND) p1 = 50;
    if (abs(p2 - 50) < DEADBAND) p2 = 50;
    if (abs(p3 - 50) < DEADBAND) p3 = 50;

    bool isStabilizationActive = (p4 > 90);//Mode Determination(>90)means stabilization mode
    int manualRollAngle = map(p1, 0, 100, 45, 135);
    int manualPitchAngle = map(p2, 0, 100, 45, 135);//--

    // Handle mode transitions
        //Update IMU reference (gy, gx).
        //Save servo positions as offset.
        //Reset PID controllers.
        //This prevents sudden servo jumps during transition.
    if (isStabilizationActive != lastStabState) {
      if (isStabilizationActive) {
        // Set reference angles
        updateMPU6050();  
        referencePitch = gy;
        referenceRoll = gx;
    
        // Save current servo positions as offset
        pitchOffset = pitchServo.read();
        rollOffset = rollServo.read();
    
        // Reset PID controllers
        resetPID(pitchPID);
        resetPID(rollPID);
        
        firstStabLoop = false;
    
        Serial.println("Stabilization ON");
      } else {
        Serial.println("Stabilization OFF");
      }
      lastStabState = isStabilizationActive;
    }

  if (isStabilizationActive) {
    if (abs(p1 - 50) <= 5 && abs(p2 - 50) <= 5) {
      // Both sticks centered – fully stabilized
      updateServoFromMPU();
    } else {
      // Manual override for non-centered axes
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
    // Manual mode – both axes
    resetPID(rollPID);
    resetPID(pitchPID);
    rollServo.write(manualRollAngle);
    pitchServo.write(manualPitchAngle);
  }


    lastPercentage1 = p1;
    lastPercentage2 = p2;
    lastPercentage3 = p3;
    lastPercentage4 = p4;
  }

  if (client) {
    Serial.println("Client connected");
    client.setTimeout(2);

    bool inStreamingMode = false;
    unsigned long lastSent = 0;

    while (client.connected()) {
      if (millis() - lastMPUTime >= (1000 / FREQ)) {
        lastMPUTime = millis();
        updateServoFromMPU();
      }
      if (client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();
        Serial.print("Received: ");
        Serial.println(command);

        if (command == "get") {
          String response = String(ACCEL_FILTER, 3) + "," + String(GYRO_FILTER, 3) + "," + String(COMP_FILTER, 3);
          client.println(response);
          Serial.println(response);
        } 
        else if (command == "getPitchPID") {
          String response = String(pitchPID.Kp, 3) + "," + String(pitchPID.Ki, 3) + "," + String(pitchPID.Kd, 3);
          client.println(response);
          Serial.println(response);
        }
        else if (command == "getRollPID") {
          String response = String(rollPID.Kp, 3) + "," + String(rollPID.Ki, 3) + "," + String(rollPID.Kd, 3);
          client.println(response);
          Serial.println(response);
        }
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
          saveParameters();
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
          client.println("ERR");
        }
      }

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

      if (cubeStreaming && millis() - lastMPUTime >= (1000 / FREQ)) {
        updateMPU6050();
        gz = 0;
        client.printf("%.2f,%.2f,%.2f\n", gx, gy, gz);
        lastMPUTime = millis();
      }
    }

    client.stop();
    lastPercentage1 = lastPercentage2 = lastPercentage3 = lastPercentage4 = -1;
  }

  delay(10);
}
//--------------------PID Control----------------------------
// Function to compute PID output with bumpless transfer
float computePID(PID& pid, float setpoint, float input, float currentOutput) {
  unsigned long now = millis();
  float dt = (now - pid.last_time) / 1000.0;
  if (dt <= 0) dt = 0.001;  // prevent division by zero
  
  float error = setpoint - input;
  
  // Bumpless transfer: adjust integral to match current output
  // It recalculates the integral term so that the current PID output matches the existing output (currentOutput).
  if (pid.Ki != 0) {
    pid.integral = (currentOutput - pid.Kp * error - pid.Kd * (error - pid.previous_error)/dt) / pid.Ki;
  }
  // Accumulates error over time. The integral term helps eliminate steady-state error.
  // constrained to prevent integral windup, which causes overshooting
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -50, 50);  // Anti-windup

  // Measures how fast the error is changing. Predicts the future behavior of the error.
  float derivative = (error - pid.previous_error) / dt;

  // Standard PID formula: P + I + D. Each component contributes to the final output value (e.g., servo angle).
  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  
  pid.previous_error = error;
  pid.last_time = now;
  
  return output;
}

// Reset PID controller
//to avoid unwanted behavior(overshoots because of accumulated error) when control modes change or conditions reset.
void resetPID(PID &pid) {
  pid.integral = 0;
  pid.previous_error = 0;
  pid.last_time = millis();
}

//--------------------Servo Control--------------------------
void updateServoFromMPU() {
  updateMPU6050();

  // Calculate PID outputs relative to reference
  float pitchOutput = computePID(pitchPID, referencePitch, -gy);
  float rollOutput = computePID(rollPID, referenceRoll, -gx);

  // Apply output as delta from saved position
  int pitchAngle = constrain(pitchOffset + pitchOutput, 45, 135);
  int rollAngle = constrain(rollOffset + rollOutput, 45, 135);

  pitchServo.write(pitchAngle);
  rollServo.write(rollAngle);
}


// -------------------- MPU6050 Functions --------------------
void initMPU6050() {
  // Initializes the MPU6050 by configuring its internal registers using I²C communication.
  Serial.println("Initiating MPU6050 sensor...");
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x6b, 0x00); //writing 0x00 wakes up the MPU6050 from sleep mode
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1a, 0x03); //0x03 = Digital low pass filter setting with ~44 Hz bandwidth for gyroscope (helps reduce noise).
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08); //Sets gyroscope sensitivity, 0x08 → ±500°/s, This affects how raw gyroscope readings are scaled
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1c, 0x08); //Sets accelerometer sensitivity, 0x08 → ±4g, This affects how raw accelerometer values are scaled.
  uint8_t sample_div = (1000 / FREQ) - 1;// Sets the sample rate divider., MPU's internal sampling is 1 kHz.
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x19, sample_div);// So, if FREQ = 50, sample_div = 19, meaning actual output rate is 1000 / (19+1) = 50 Hz., Ensures sensor output frequency matches the desired loop rate.
}

void calibrateMPU6050() {
  // This function measures those biases and stores them so they can be subtracted from future readings.
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
  //Reads raw sensor data(6 axis), filters it and computes orientation angles and updates gx,gy,gz
  static unsigned long last_time = millis(); //remembers when the function last ran(time stamp)
  uint8_t data[14];

  if (i2c_read(MPU6050_I2C_ADDRESS, 0x3b, data, 14) != 0) return;//reads 14 byts(acc6,temp2,gyro6)

  accX = ((data[0] << 8) | data[1]);//constructiong 16 bit int from high and low byts, this is raw acc readings in x,y,z
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);

  gyrX = (((data[8] << 8) | data[9]) - gyrXoffs) / gSensitivity;//Similar to above, Offsets(gyrXoffs, etc.) are subtracted(rm biases),continued below... 
  gyrY = (((data[10] << 8) | data[11]) - gyrYoffs) / gSensitivity;//gSensitivity = 65.5, converting raw data to deg/s assuming +/-500 dps sensitivity.
  gyrZ = (((data[12] << 8) | data[13]) - gyrZoffs) / gSensitivity;

  //Low-Pass Filter Accelerometer Data
  filtered_ax = filtered_ax * (1.0 - ACCEL_FILTER) + accX * ACCEL_FILTER;//ACCEL_FILTER (like 0.3) controls how much "new" data affects the value.
  filtered_ay = filtered_ay * (1.0 - ACCEL_FILTER) + accY * ACCEL_FILTER;
  filtered_az = filtered_az * (1.0 - ACCEL_FILTER) + accZ * ACCEL_FILTER;

  //Low-Pass Filter Gyroscope Data
  filtered_gx = filtered_gx * (1.0 - GYRO_FILTER) + gyrX * GYRO_FILTER;//same as above, removes spikes/noise from the gyroscope data.
  filtered_gy = filtered_gy * (1.0 - GYRO_FILTER) + gyrY * GYRO_FILTER;
  filtered_gz = filtered_gz * (1.0 - GYRO_FILTER) + gyrZ * GYRO_FILTER;

  //Estimate Pitch and Roll from Accelerometer (Angle from Gravity), calculating tilt angle in degree
  double ay = atan2(filtered_ax, sqrt(pow(filtered_ay, 2) + pow(filtered_az, 2))) * 180 / M_PI;//These are pitch(ay) and roll(ax) from acc only
  double ax = atan2(filtered_ay, sqrt(pow(filtered_ax, 2) + pow(filtered_az, 2))) * 180 / M_PI;//absolute orientation but noisy and affec by acceleration

  //Integrate Gyroscope Readings to Track Rotation Over Time
  gx += filtered_gx / FREQ;//Integrates angular velocity (°/s) over time → gives angle in degrees.
  gy -= filtered_gy / FREQ;//Using FREQ to convert to time step: 1 / FREQ = seconds per update.
  gz += filtered_gz / FREQ;

  //Complementary Filter to Combine Gyro and Accelerometer
  gx = gx * (1.0 - COMP_FILTER) + ax * COMP_FILTER;
  gy = gy * (1.0 - COMP_FILTER) + ay * COMP_FILTER;

  //Ensures the function takes exactly 1 / FREQ seconds (e.g., 20 ms for 50 Hz). Ensures consistent sampling for filters and integration
  while (millis() - last_time < (1000 / FREQ)) delay(1);
  last_time = millis();
}

// -------------------- EEPROM --------------------
//loads value from EEPROM and also checks if the value loaded is valid
void loadParameters() {
  EEPROM.get(ACCEL_FILTER_ADDR, ACCEL_FILTER); //this and 8 below will read the necessary param from EEPROM
  EEPROM.get(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.get(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.get(PITCH_PID_ADDR, pitchPID.Kp);
  EEPROM.get(PITCH_PID_ADDR + 4, pitchPID.Ki);
  EEPROM.get(PITCH_PID_ADDR + 8, pitchPID.Kd);
  EEPROM.get(ROLL_PID_ADDR, rollPID.Kp);
  EEPROM.get(ROLL_PID_ADDR + 4, rollPID.Ki);
  EEPROM.get(ROLL_PID_ADDR + 8, rollPID.Kd);

  // Validate loaded values
  if (isnan(ACCEL_FILTER) || ACCEL_FILTER <= 0 || ACCEL_FILTER > 1.0) ACCEL_FILTER = 0.3;//If the ACCEL_FILTER is not a number, <= 0, or > 1 → reset 0.3.
  if (isnan(GYRO_FILTER) || GYRO_FILTER <= 0 || GYRO_FILTER > 1.0) GYRO_FILTER = 0.08;
  if (isnan(COMP_FILTER) || COMP_FILTER <= 0 || COMP_FILTER > 1.0) COMP_FILTER = 0.7;
  
  if (isnan(pitchPID.Kp) || pitchPID.Kp < 0) pitchPID.Kp = 1.0;
  if (isnan(pitchPID.Ki) || pitchPID.Ki < 0) pitchPID.Ki = 0.0;
  if (isnan(pitchPID.Kd) || pitchPID.Kd < 0) pitchPID.Kd = 0.0;
  
  if (isnan(rollPID.Kp) || rollPID.Kp < 0) rollPID.Kp = 1.0;
  if (isnan(rollPID.Ki) || rollPID.Ki < 0) rollPID.Ki = 0.0;
  if (isnan(rollPID.Kd) || rollPID.Kd < 0) rollPID.Kd = 0.0;
}

void saveParameters() {
  EEPROM.put(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.put(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.put(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.put(PITCH_PID_ADDR, pitchPID.Kp);
  EEPROM.put(PITCH_PID_ADDR + 4, pitchPID.Ki);
  EEPROM.put(PITCH_PID_ADDR + 8, pitchPID.Kd);
  EEPROM.put(ROLL_PID_ADDR, rollPID.Kp);
  EEPROM.put(ROLL_PID_ADDR + 4, rollPID.Ki);
  EEPROM.put(ROLL_PID_ADDR + 8, rollPID.Kd);
  EEPROM.commit();
}

// -------------------- I2C Helpers --------------------
int i2c_read(int addr, int start, uint8_t* buffer, int size) {
  Wire.beginTransmission(addr);
  Wire.write(start);
  if (Wire.endTransmission(false) != 0) return -1;
//  Wire.requestFrom(addr, size, true); 
  Wire.requestFrom((uint8_t)addr, (size_t)size, (bool)true);
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