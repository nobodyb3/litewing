#ifndef WIFI_CRTP_H
#define WIFI_CRTP_H

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "PID.h"
#include "motors.h"

// ---- Network config (mirrors components/drivers/general/wifi/wifi_esp32.c
//      and main/Kconfig.projbuild from the original ESP-IDF/ESP32-S3
//      firmware, so the app's hard-coded defaults still work) -----------
#define WIFI_BASE_SSID  "LiteWing"
#define WIFI_PASSWORD   "12345678"   // "" for open network
#define WIFI_CHANNEL    6
#define UDP_SERVER_PORT 2390
#define UDP_BUFSIZE     64

// CRTP port/header layout, from components/core/crazyflie/modules/interface/crtp.h
#define CRTP_PORT_SETPOINT 0x03

WiFiUDP udp;
IPAddress apIP(192, 168, 43, 42);
IPAddress apMask(255, 255, 255, 0);

IPAddress clientIP;
uint16_t  clientPort = 0;
bool      clientKnown = false;
unsigned long lastPacketMs = 0;
#define LINK_TIMEOUT_MS 1000  // disarm if the app goes quiet this long

// Legacy CRTP RPYT setpoint payload, from crtp_commander_rpyt.c
struct __attribute__((packed)) CommanderCrtpLegacyValues {
  float roll;
  float pitch;
  float yaw;
  uint16_t thrust;
};

uint8_t calcChecksum(uint8_t *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return sum;
}

void initWiFiCRTP() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char ssid[33];
  snprintf(ssid, sizeof(ssid), "%s_%02X%02X%02X", WIFI_BASE_SSID, mac[3], mac[4], mac[5]);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, apMask);
  WiFi.softAP(ssid, WIFI_PASSWORD, WIFI_CHANNEL);

  udp.begin(UDP_SERVER_PORT);

  Serial.print("AP SSID: "); Serial.println(ssid);
  Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());
  Serial.print("UDP port:"); Serial.println(UDP_SERVER_PORT);
}

// Call every loop() iteration. Non-blocking.
void handleWiFiCRTP() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0 && packetSize <= UDP_BUFSIZE) {
    uint8_t buf[UDP_BUFSIZE];
    int len = udp.read(buf, packetSize);
    if (len < 2) return;

    uint8_t receivedChecksum = buf[len - 1];
    if (calcChecksum(buf, len - 1) != receivedChecksum) {
      return; // corrupt packet, drop it (matches wifi_esp32.c behaviour)
    }

    clientIP = udp.remoteIP();
    clientPort = udp.remotePort();
    clientKnown = true;
    lastPacketMs = millis();

    uint8_t header = buf[0];
    uint8_t port = (header >> 4) & 0x0F;
    // channel = header & 0x03; // not needed for the single legacy setpoint channel

    if (port == CRTP_PORT_SETPOINT && (len - 1 - 1) >= (int)sizeof(CommanderCrtpLegacyValues)) {
      CommanderCrtpLegacyValues sp;
      memcpy(&sp, &buf[1], sizeof(sp));

      Desired_Roll_Angle  = sp.roll;
      Desired_Pitch_Angle = sp.pitch;
      Desired_Yaw_Rate    = sp.yaw;      // legacy format sends yaw as a rate
      throttle = map(constrain(sp.thrust, 0, 60000), 0, 60000, 1000, 2000);
    }
    // Other CRTP ports (LOG=5, PARAM=2, CONSOLE=0, PLATFORM=0x0D...) are not
    // implemented yet - see the README "Phase 2" notes. The app may still
    // show a disconnect warning or grey-out telemetry widgets until those
    // are added; flight control itself only needs the setpoint port.
  }

  // Safety: if the app stops sending packets, cut the throttle.
  if (clientKnown && (millis() - lastPacketMs > LINK_TIMEOUT_MS)) {
    throttle = 0;
  }
}

#endif // WIFI_CRTP_H
