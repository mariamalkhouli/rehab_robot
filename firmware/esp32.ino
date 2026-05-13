#include <WiFi.h>
#include <WiFiUdp.h>

// ⬇ Change these to your WiFi network
const char* ssid     = "Mariam iPhone";
const char* password = "mariam2003";

// ⬇ Change this to your Raspberry Pi's IP address
const char* PI_IP = "172.20.10.6";
const int   UDP_PORT = 5005;
const int   BUTTON_PIN = 2;

WiFiUDP udp;

void setup() {
  // ESP32-C3 Serial fix
  Serial.begin(115200);
  delay(3000);  // wait for Serial to wake up

  Serial.println("=== ESP32-C3 Emergency Stop ===");

  // Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Button ready on GPIO 9");

  // WiFi setup
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;

    // After 20 seconds, print a clear error
    if (attempts > 40) {
      Serial.println("");
      Serial.println("❌ Could not connect to WiFi!");
      Serial.println("Check your WiFi name and password.");
      Serial.println("Restarting in 5 seconds...");
      delay(5000);
      ESP.restart();
    }
  }

  Serial.println("");
  Serial.println("✅ WiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Sending STOP to Pi at: ");
  Serial.println(PI_IP);
  Serial.println("==============================");
  Serial.println("Ready. Press button to test.");

  udp.begin(UDP_PORT);
}

void loop() {
  // Check WiFi still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠ WiFi lost! Reconnecting...");
    WiFi.reconnect();
    delay(3000);
    return;
  }

  // Button pressed (active LOW because INPUT_PULLUP)
if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // wait 50ms
    if (digitalRead(BUTTON_PIN) == LOW) {  // confirm still pressed
        Serial.println("BUTTON PRESSED — Sending STOP...");
        udp.beginPacket(PI_IP, UDP_PORT);
        udp.print("STOP");
        udp.endPacket();
        Serial.println("STOP sent!");
        
        while (digitalRead(BUTTON_PIN) == LOW);  // wait for release
        delay(300);  // debounce after release
    }
}
}