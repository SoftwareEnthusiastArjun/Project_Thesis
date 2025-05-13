#include <WiFi.h>
#include <EEPROM.h>

const char* ssid = "aju";
const char* password = "@ajujcd@";
WiFiServer server(12345);

const int ledPin = 2;  // Built-in LED pin
float storedValue = 0.5f;  // Default value

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  EEPROM.begin(512);  // Initialize EEPROM
  
  // Load stored value from EEPROM
  EEPROM.get(0, storedValue);
  
  // Validate the loaded value
  if (isnan(storedValue) || storedValue < 0 || storedValue > 1) {
    storedValue = 0.5f;  // Reset to default if invalid
  }
  
  Serial.print("Stored value: ");
  Serial.println(storedValue, 2);  // Print with 2 decimal places

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    server.begin();
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}

void saveValueToEEPROM(float value) {
  // Blink LED while saving
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
  
  EEPROM.put(0, value);
  EEPROM.commit();
  storedValue = value;
  Serial.print("Value saved to EEPROM: ");
  Serial.println(value, 2);
}

void handleClient(WiFiClient &client) {
  String command = client.readStringUntil('\n');
  command.trim();
  
  if (command == "get") {
    client.print(String(storedValue, 2) + "\n");
    Serial.println("sending value to client");
  } 
  else if (command.startsWith("set")) {
    float newValue = command.substring(3).toFloat();
    newValue = constrain(newValue, 0.0f, 1.0f);
    saveValueToEEPROM(newValue);
    client.println("OK");
    Serial.printf("Setting new value: %.2f\n", newValue);
  }
  else if (command == "save") {
  saveValueToEEPROM(storedValue);
  client.println("OK");
  Serial.println("Value saved to EEPROM on command");
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");
    while (client.connected()) {
      if (client.available()) {
        handleClient(client);
      }
      delay(10);
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
