#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>

// ------------------- EEPROM & Filter Settings -------------------
#define EEPROM_SIZE 12
#define ACCEL_FILTER_ADDR 0
#define GYRO_FILTER_ADDR 4
#define COMP_FILTER_ADDR 8

float ACCEL_FILTER = 0.3;
float GYRO_FILTER = 0.08;
float COMP_FILTER = 0.7;

// ------------------- WiFi Settings -------------------
const char* ssid = "aju";
const char* password = "@ajujcd@";

WiFiServer server(12345);

// ------------------- PWM Input Settings -------------------
#define PITCH_IP 15
#define ROLL_IP 16
#define YAW_IP 17
#define AUTO_PILOT 18

#define MIN_PULSE_WIDTH 999
#define MAX_PULSE_WIDTH 1993
#define PULSE_TIMEOUT 25000

int lastPercentage1 = -1;
int lastPercentage2 = -1;
int lastPercentage3 = -1;
int lastPercentage4 = -1;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStarting...");

  EEPROM.begin(EEPROM_SIZE);
  loadParameters();

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

// ------------------- Loop -------------------
void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Client connected");
    client.setTimeout(2);

    bool inStreamingMode = false;
    unsigned long lastSent = 0;

    while (client.connected()) {
      // Check for commands
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

        } else {
          client.println("ERR");
        }
      }

      // Stream PWM data if requested
      if (inStreamingMode) {
        uint32_t pulseWidth1 = pulseIn(PITCH_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth2 = pulseIn(ROLL_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth3 = pulseIn(YAW_IP, HIGH, PULSE_TIMEOUT);
        uint32_t pulseWidth4 = pulseIn(AUTO_PILOT, HIGH, PULSE_TIMEOUT);

        int percentage1 = map(pulseWidth1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100);
        int percentage2 = map(pulseWidth2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100);
        int percentage3 = map(pulseWidth3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100);
        int percentage4 = map(pulseWidth4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100);

        percentage1 = constrain(percentage1, 0, 100);
        percentage2 = constrain(percentage2, 0, 100);
        percentage3 = constrain(percentage3, 0, 100);
        percentage4 = constrain(percentage4, 0, 100);

        bool updated = false;

        if (percentage1 != lastPercentage1 || percentage2 != lastPercentage2 || percentage3 != lastPercentage3 || percentage4 != lastPercentage4) {
          String data = String(percentage1) + "," + String(percentage2) + "," + String(percentage3) + "," + String(percentage4);
          client.println(data);
          Serial.print("Sent PWM %s: ");
          Serial.println(data);
          lastPercentage1 = percentage1;
          lastPercentage2 = percentage2;
          lastPercentage3 = percentage3;
          lastPercentage4 = percentage4;
          lastSent = millis();
          updated = true;
        }

        if (!updated && millis() - lastSent > 2000) {
          client.println("No signal");
          lastSent = millis();
        }
      }

      delay(20);
    }

    client.stop();
    Serial.println("Client disconnected");
    lastPercentage1 = -1;
    lastPercentage2 = -1;
    lastPercentage3 = -1;
    lastPercentage4 = -1;
  }

  delay(10);
}

// ------------------- EEPROM Functions -------------------
void loadParameters() {
  EEPROM.get(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.get(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.get(COMP_FILTER_ADDR, COMP_FILTER);

  if (isnan(ACCEL_FILTER) || ACCEL_FILTER <= 0 || ACCEL_FILTER > 1) ACCEL_FILTER = 0.3;
  if (isnan(GYRO_FILTER) || GYRO_FILTER <= 0 || GYRO_FILTER > 1) GYRO_FILTER = 0.08;
  if (isnan(COMP_FILTER) || COMP_FILTER <= 0 || COMP_FILTER > 1) COMP_FILTER = 0.7;

  Serial.println("Loaded filter parameters from EEPROM:");
  Serial.printf("  ACCEL: %.3f\n", ACCEL_FILTER);
  Serial.printf("  GYRO:  %.3f\n", GYRO_FILTER);
  Serial.printf("  COMP:  %.3f\n", COMP_FILTER);
}

void saveParameters() {
  EEPROM.put(ACCEL_FILTER_ADDR, ACCEL_FILTER);
  EEPROM.put(GYRO_FILTER_ADDR, GYRO_FILTER);
  EEPROM.put(COMP_FILTER_ADDR, COMP_FILTER);
  EEPROM.commit();
  Serial.println("Filter parameters saved to EEPROM.");
}
