#include "ESP8266WiFi.h"

#define SSID "ssid"
#define PASSWORD "password"
#define HOSTNAME "fetch-runstop"
#define PORT 1337

#define BUTTON_PIN 2

WiFiServer wifiServer(PORT);

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.hostname(HOSTNAME);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting...");
    delay(1000);
  }
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());

  wifiServer.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  WiFiClient client = wifiServer.available();
  if (client) {
    while (client.connected()) {
      client.write(!digitalRead(BUTTON_PIN));
      delay(100);
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}