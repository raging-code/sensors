#include <WiFi.h>
#include <WebSocketsClient.h>

// WiFi credentials
const char* ssid = "HUAWEI-2";
const char* password = "34444444";

// WebSocket server details
const char* websocket_server = "192.168.18.222"; // Change this
const int websocket_port = 8765;
const char* websocket_path = "/";

WebSocketsClient webSocket;

// DO Sensor pin
const int DO_PIN = 34; // GPIO34 for analog input
float DO_value = 0.0;
int DO_adc = 0;

// Calibration variables
float calibration_offset = 0.0;
bool calibrating = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_TIME = 900000; // 15 minutes in milliseconds

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize WebSocket
  webSocket.begin(websocket_server, websocket_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("SENSORS|DO: |DO ADC: ");
}

void loop() {
  webSocket.loop();
  
  // Read DO sensor
  DO_adc = analogRead(DO_PIN);
  
  // Convert ADC to DO value (simplified - you'll need proper calibration)
  // Reference: SEN0237 outputs 0-3V corresponding to 0-20mg/L
  float voltage = DO_adc * (3.3 / 4095.0); // ESP32 has 12-bit ADC
  DO_value = (voltage / 3.0) * 20.0; // Simplified conversion
  
  // Apply calibration offset
  DO_value += calibration_offset;
  
  // Send data via WebSocket every 2 seconds
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 2000) {
    String data = "DO:" + String(DO_value, 2) + ",ADC:" + String(DO_adc);
    webSocket.sendTXT(data);
    lastSend = millis();
    
    // Display on serial monitor
    Serial.print("DO: ");
    Serial.print(DO_value, 2);
    Serial.print(" mg/L | DO ADC: ");
    Serial.println(DO_adc);
  }
  
  // Handle calibration timing
  if (calibrating && millis() - calibration_start >= CALIBRATION_TIME) {
    calibrating = false;
    // In dry air, DO should be approximately 8.24 mg/L at 25°C
    // We'll set this as our target
    calibration_offset = 8.24 - DO_value;
    Serial.println("Calibration complete. Offset: " + String(calibration_offset));
    
    // Send calibration complete message
    webSocket.sendTXT("CALIBRATION_COMPLETE");
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      break;
      
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      break;
      
    case WStype_TEXT:
      handleWebSocketMessage((char*)payload);
      break;
  }
}

void handleWebSocketMessage(char* message) {
  String msg = String(message);
  
  if (msg == "START_CALIBRATION") {
    calibrating = true;
    calibration_start = millis();
    calibration_offset = 0.0; // Reset offset
    Serial.println("Starting 15-minute calibration...");
  }
  else if (msg == "STOP_CALIBRATION") {
    calibrating = false;
    Serial.println("Calibration stopped");
  }
}
