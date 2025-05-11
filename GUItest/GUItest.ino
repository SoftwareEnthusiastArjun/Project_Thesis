#include <WiFi.h>
#include <EEPROM.h>

const char* ssid = "aju";
const char* password = "@ajujcd@";
WiFiServer server(12345);

const int ledPin = 2;
String currentMode = "off";

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  EEPROM.begin(10);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    server.begin();
  } else {
    Serial.println("\nFailed to connect!");
  }

  // Load saved mode
  currentMode = EEPROM.readString(0);
  if(currentMode == "") currentMode = "off";
  updateLED();
}

void updateLED() {
  if(currentMode == "on") digitalWrite(ledPin, HIGH);
  else if(currentMode == "off") digitalWrite(ledPin, LOW);
  else if(currentMode == "blink"){
    for(unsigned int i=0;i<10;i++)
    {
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
    }
  }
}

void loop() {
  if(WiFi.status() != WL_CONNECTED) return;
  
  WiFiClient client = server.available();
  if(client) {
    Serial.println("New client connected");
    
    while(client.connected()) {  // Maintain connection
      if(client.available()) {
        Serial.println("Data available");
        String command = client.readStringUntil('\n');
        command.trim();
        Serial.println("Received: " + command);
        
        if(command == "on" || command == "off" || command == "blink") {
          currentMode = command;
          updateLED();
          client.println("OK_" + command);
          Serial.println("Sent response");
        }
        else if(command == "save") {
          EEPROM.writeString(0, currentMode);
          EEPROM.commit();
          client.println("SAVED_" + currentMode);
          Serial.println("State saved");
        }
      }
      delay(10);  // Small delay to prevent watchdog trigger
    }
    client.stop();
    Serial.println("Client disconnected");
  }

  // Handle blinking
  static unsigned long lastBlink = 0;
  if(currentMode == "blink" && millis() - lastBlink >= 500) {
    lastBlink = millis();
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
}

