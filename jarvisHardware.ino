#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient_Generic.h>
#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
WebSocketsClient webSocketRelay;  // WebSocket client for relay control
WebSocketsClient webSocketCurrent;  // WebSocket client for current readings

#define USE_SSL false
#define RELAY_PIN D0  // Define the D1 pin for the relay
#define CURRENT_PIN A0  // Define the A0 pin for current sensing
#define SEND_INTERVAL 30000L  // Set the send interval to 30 seconds

#if USE_SSLf
  #define WS_SERVER "wss://echo.websocket.org"
  #define WS_PORT 443
#else
  #define WS_SERVER "192.168.0.197"
  #define WS_PORT 8000
#endif

bool alreadyConnectedRelay = false;
bool alreadyConnectedCurrent = false;
unsigned long connectionTimeRelay = 0;
unsigned long connectionTimeCurrent = 0;

String latestPayloadRelay = "";  // Stores the latest relay WebSocket payload
String latestPayloadCurrent = "";  // Stores the latest current WebSocket payload
bool newPayloadReceivedRelay = false;  // Indicates if new payload is received for relay
bool newPayloadReceivedCurrent = false;  // Indicates if new payload is received for current readings

// Function to send current readings to WebSocket
void sendCurrentReading() {
  static uint64_t sendCurrentReading_timeout = 0;
  uint64_t now = millis();

  if (now > sendCurrentReading_timeout) {
    int sensorValue = analogRead(CURRENT_PIN);  // Read the current value from A0 pin
    float voltage = sensorValue * (3.3 / 1024.0);  // Convert sensor value to voltage
    float current = voltage * 1000;  // Calculate current (assuming a known scaling factor)

    // Debug sensor readings
    Serial.println("Sensor Value: " + String(sensorValue));
    Serial.println("Voltage: " + String(voltage));
    Serial.println("Current: " + String(current));

    DynamicJsonDocument doc(1024);
    doc["current"] = current;
    doc["voltage"] = voltage;

    String output;
    serializeJson(doc, output);

    if (webSocketCurrent.isConnected()) {
      webSocketCurrent.sendTXT(output);  // Send current data via WebSocket
      Serial.println("Current reading sent: " + output);
    } else {
      Serial.println("WebSocket for current is not connected. Unable to send.");
    }

    sendCurrentReading_timeout = millis() + SEND_INTERVAL;
  }
}

// General WebSocket Event Handler
void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    String message = String((char*)payload);
    Serial.println("[WebSocket] Received: " + message);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }

    if (doc.containsKey("switch_control")) {
      const char* state = doc["switch_control"];
      if (strcmp(state, "Yes") == 0) {
        digitalWrite(RELAY_PIN, LOW);  // Turn relay ON
        Serial.println("[WebSocket] Relay turned ON");
      } else if (strcmp(state, "No") == 0) {
        digitalWrite(RELAY_PIN, HIGH);  // Turn relay OFF
        Serial.println("[WebSocket] Relay turned OFF");
      }
    } else if (doc.containsKey("current_values")) {
      // Handle current values payload if needed
      Serial.println("Current values payload received.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(200);

  Serial.println("\nStarting ESP8266 WebSocket Clients");
  WiFiMulti.addAP("MTN_2.4G_343A53", "55A6CFBF");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println();
  Serial.print("WebSocket Client started @ IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Initially turn off the relay

  // Connect to WebSocket servers
  webSocketRelay.begin(WS_SERVER, WS_PORT, "/ws/switch-control/");
  webSocketRelay.onEvent(handleWebSocketEvent);
  webSocketRelay.setReconnectInterval(5000);

  webSocketCurrent.begin(WS_SERVER, WS_PORT, "/ws/current-values/");
  webSocketCurrent.onEvent(handleWebSocketEvent);
  webSocketCurrent.setReconnectInterval(5000);
}

void loop() {
  // Keep the WebSocket connections active
  webSocketRelay.loop();
  webSocketCurrent.loop();

  // Send current readings periodically
  sendCurrentReading();
}
