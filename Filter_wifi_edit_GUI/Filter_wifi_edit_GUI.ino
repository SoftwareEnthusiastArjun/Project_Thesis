#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>

#define EEPROM_SIZE 12
#define ACCEL_FILTER_ADDR 0
#define GYRO_FILTER_ADDR 4
#define COMP_FILTER_ADDR 8

float ACCEL_FILTER = 0.3;
float GYRO_FILTER = 0.08;
float COMP_FILTER = 0.7;

// WiFi credentials
const char* ssid = "aju";
const char* password = "@ajujcd@";

WiFiServer server(12345);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStarting...");

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadParameters();

  // Connect WiFi
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

  // Start mDNS
  if (MDNS.begin("esp32")) {
    Serial.println("mDNS responder started: esp32.local");
  } else {
    Serial.println("Error starting mDNS");
  }

  // Start TCP server
  server.begin();
  Serial.println("TCP server started on port 12345");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    client.setTimeout(2000);

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
        } else {
          client.println("ERR");
        }
      }
    }

    client.stop();
    Serial.println("Client disconnected");
  }
}

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
