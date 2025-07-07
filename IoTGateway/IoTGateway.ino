#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <map>
#include <string>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <HTTPUpdate.h>
#include "esp_wifi.h"
#include "config.h"

#define WIFI_ANT_CONFIG 14
#define AP_TIMEOUT_MS 2 * 60 * 1000
#define DNS_PORT 53
#define UART_RX 17
#define UART_TX 18
#define UART_BAUD 38400
#define MQTT_BUFFER_SIZE 4096
#define MAX_SENSOR_CACHE_SIZE 200

String licenseKey = "";
String deviceId = "";
String firmwareVersion = FIRMWARE_VERSION;
int intervalMinutes = INTERVAL_MINUTES;

Preferences prefs;
WebServer server(80);
DNSServer dns;
WiFiClientSecure net;
MQTTClient mqtt(MQTT_BUFFER_SIZE);

unsigned long bootTime;
unsigned long readingsSent = 0;
bool inApMode = false;
unsigned long lastPublishTime = 0;
unsigned long lastMqttReconnectAttempt = 0; // Track last MQTT reconnect

struct SensorReading {
  std::string serialNumber;
  String pressureHex;
  String temperatureHex;
  uint8_t rssi;
  uint8_t ambient;
};

std::map<std::string, SensorReading> sensorCache;

String mqttBaseTopic() {
  return "pressurepro/devices/WifiSensorGateway/" + licenseKey + "/";
}

String getHardwareId() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  char id[13];
  sprintf(id, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(id);
}

void handleCommandMessage(String &topic, String &payload) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload)) return;
  if (doc.containsKey("action")) {
    String action = doc["action"].as<String>();
    if (action == "reboot") ESP.restart();
    if (action == "reset_wifi") {
      Serial.println("🔄 Resetting WiFi configuration...");
      prefs.begin("wifi", false);
      prefs.clear();
      prefs.end();
      
      // Reset global variables to defaults
      licenseKey = "";
      deviceId = "";
      intervalMinutes = INTERVAL_MINUTES;
      
      Serial.println("✅ WiFi preferences cleared. Rebooting in 2 seconds...");
      delay(2000); // Give time for preferences to be written to flash
      ESP.restart();
    }
    if (action == "firmware_update") {
      if (doc.containsKey("url")) {
        String url = doc["url"].as<String>();
        Serial.println("📦 Starting firmware update from: " + url);
        performFirmwareUpdate(url);
      } else {
        Serial.println("❌ Firmware update action missing URL");
      }
    }
    if (action == "update_certificates") {
      if (doc.containsKey("rootca") && doc.containsKey("devicecert") && doc.containsKey("privatekey")) {
        Serial.println("🔐 Updating certificates...");
        prefs.begin("certs", false);
        prefs.putString("rootca", doc["rootca"].as<String>());
        prefs.putString("devicecert", doc["devicecert"].as<String>());
        prefs.putString("privatekey", doc["privatekey"].as<String>());
        prefs.end();
        Serial.println("✅ Certificates updated. Rebooting in 2 seconds...");
        delay(2000);
        ESP.restart();
      } else {
        Serial.println("❌ Certificate update missing required fields (rootca, devicecert, privatekey)");
      }
    }
    if (action == "reset_certificates") {
      Serial.println("🔄 Resetting to embedded certificates...");
      prefs.begin("certs", false);
      prefs.clear();
      prefs.end();
      Serial.println("✅ Certificate preferences cleared. Rebooting in 2 seconds...");
      delay(2000);
      ESP.restart();
    }
    if (action == "clear_cache") {
      Serial.println("🗑️ Clearing sensor cache...");
      sensorCache.clear();
      Serial.println("✅ Sensor cache cleared");
    }
  }
}

void performFirmwareUpdate(String url) {
  Serial.println("📦 Downloading firmware from: " + url);
  
  // Disconnect MQTT to free up memory and prevent interference
  if (mqtt.connected()) {
    mqtt.disconnect();
    Serial.println("🔌 Disconnected MQTT for update");
  }
  
  // Set longer timeout for large files
  httpUpdate.setLedPin(LED_BUILTIN, LOW);
  httpUpdate.rebootOnUpdate(true);
  
  // Try with HTTPClient first (better redirect handling)
  HTTPClient http;
  http.begin(url);
  http.setTimeout(30000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  Serial.println("🔗 Starting HTTP update with redirects...");
  int httpCode = http.GET();
  Serial.printf("HTTP Response code: %d\n", httpCode);
  
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    Serial.printf("Content length: %d bytes\n", contentLength);
    
    if (contentLength > 0) {
      if (Update.begin(contentLength)) {
        size_t written = Update.writeStream(http.getStream());
        if (written == contentLength) {
          if (Update.end()) {
            Serial.println("✅ Update completed successfully");
            http.end();
            delay(3000);
            ESP.restart();
          } else {
            Serial.printf("❌ Update end failed: %s\n", Update.errorString());
            http.end();
          }
        } else {
          Serial.printf("❌ Written size mismatch. Expected: %d, Got: %d\n", contentLength, written);
          http.end();
        }
      } else {
        Serial.printf("❌ Update begin failed: %s\n", Update.errorString());
        http.end();
      }
    } else {
      Serial.println("❌ Content length is 0");
      http.end();
    }
  } else {
    Serial.printf("❌ HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
  }
  
  // If we get here, the HTTPClient method failed, so try the alternative method
  Serial.println("🔄 HTTPClient method failed, trying alternative...");
  
  // Try alternative method with HTTPClient
  if (tryAlternativeUpdate(url)) {
    Serial.println("✅ Alternative update successful!");
    delay(3000);
    ESP.restart();
  } else {
    Serial.println("❌ Alternative update also failed");
    // Try to reconnect MQTT after failed update
    if (WiFi.status() == WL_CONNECTED) {
      connectMQTT();
    }
  }
}

void handleFirmwareMessage(String &topic, String &payload) {
  Serial.println("⬇️ Firmware update message received");
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println("❌ Failed to parse firmware JSON");
    return;
  }
  if (!doc.containsKey("url")) return;

  String url = doc["url"].as<String>();
  performFirmwareUpdate(url);
}

void messageReceived(String &topic, String &payload) {
  if (topic.endsWith("commands")) handleCommandMessage(topic, payload);
  else if (topic.endsWith("firmware")) handleFirmwareMessage(topic, payload);
}

void connectMQTT() {
  mqtt.begin(AWS_IOT_ENDPOINT, 8883, net);
  mqtt.onMessage(messageReceived);

  while (!mqtt.connect("esp32c6-client")) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ MQTT connected!");
  mqtt.subscribe(mqttBaseTopic() + "commands");
  mqtt.subscribe(mqttBaseTopic() + "firmware");
  mqtt.subscribe("pressurepro/devices/WifiSensorGateway/firmware");
}

bool tryWiFiConnection() {
  Serial.println("🔍 Attempting WiFi connection...");
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("password", "");
  licenseKey = prefs.getString("license", "");
  deviceId = prefs.getString("device_id", "");
  intervalMinutes = prefs.getInt("interval", INTERVAL_MINUTES);
  prefs.end();

  Serial.println("📋 Loaded config - SSID: " + (ssid.isEmpty() ? "EMPTY" : ssid) + 
                 ", License: " + (licenseKey.isEmpty() ? "EMPTY" : licenseKey.substring(0, 8) + "..."));
  
  if (ssid.isEmpty() || pass.isEmpty() || licenseKey.isEmpty()) {
    Serial.println("❌ Missing WiFi credentials, starting AP mode");
    return false;
  }

  pinMode(WIFI_ANT_CONFIG, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG, HIGH);

  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < AP_TIMEOUT_MS) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi connection failed after " + String((millis() - startAttemptTime) / 1000) + " seconds");
  } else {
    Serial.println("✅ WiFi connected successfully");
  }

  return WiFi.status() == WL_CONNECTED;
}

void startAPMode() {
  Serial.println("📶 Starting AP mode...");
  
  // Disconnect from any existing WiFi connection
  WiFi.disconnect();
  delay(100);
  
  // Set WiFi mode to AP only
  WiFi.mode(WIFI_AP);
  delay(100);
  
  inApMode = true;
  String ssid = "PressurePro-Gateway" + getHardwareId();
  WiFi.softAP(ssid.c_str(), "configureme");
  dns.start(DNS_PORT, "*", WiFi.softAPIP());

  server.onNotFound([]() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
  });

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<form method='POST'>SSID: <input name='ssid'><br>Password: <input name='password'><br>License Key: <input name='license'><br>Device ID: <input name='device_id'><br>Interval (min): <input name='interval'><br><input type='submit'></form>");
  });

  server.on("/", HTTP_POST, []() {
    prefs.begin("wifi", false);
    prefs.putString("ssid", server.arg("ssid"));
    prefs.putString("password", server.arg("password"));
    prefs.putString("license", server.arg("license"));
    prefs.putString("device_id", server.arg("device_id"));
    prefs.putInt("interval", server.arg("interval").toInt());
    prefs.end();
    server.send(200, "text/html", "Saved. Rebooting...");
    delay(2000);
    ESP.restart();
  });

  server.begin();
  Serial.println("📶 AP Mode Started: " + ssid);
}

void publishSensorCache() {
  StaticJsonDocument<1024> doc;  // Adjust size based on actual needs
  time_t now;
  time(&now);

  doc["timestamp"] = String(ctime(&now)).c_str();
  doc["unix_time"] = now * 1000UL;
  JsonArray data = doc.createNestedArray("sensor_data");
  for (auto& kv : sensorCache) {
    JsonObject obj = data.createNestedObject();
    obj["serialNumber"] = kv.second.serialNumber.c_str();
    obj["pressure"] = kv.second.pressureHex;
    obj["temperature"] = kv.second.temperatureHex;
    obj["rx_rssi"] = kv.second.rssi;
    obj["amb_rssi"] = kv.second.ambient;
  }

  JsonObject info = doc.createNestedObject("device_info");
  info["device_id"] = deviceId;
  info["firmware_version"] = firmwareVersion;
  info["license_key"] = licenseKey;

  String payload;
  serializeJson(doc, payload);
  mqtt.publish(mqttBaseTopic() + "sensor-readings", payload);

  // Diagnostics
  StaticJsonDocument<512> diag;
  JsonObject conn = diag.createNestedObject("connection");
  conn["signal_strength"] = WiFi.RSSI();
  conn["network_type"] = "WiFi";
  conn["ssid"] = WiFi.SSID();
  conn["ip_address"] = WiFi.localIP().toString();
  conn["connection_duration"] = (millis() - bootTime) / 1000;
  conn["readings_sent"] = ++readingsSent;

  String diagPayload;
  serializeJson(diag, diagPayload);
  mqtt.publish(mqttBaseTopic() + "diagnostics", diagPayload);

  sensorCache.clear();
  lastPublishTime = millis();
}

bool checkCertificateExpiry() {
  // This is a simplified check - you'd need to parse the certificate properly
  // For now, we'll just check if certificates exist and are recent
  prefs.begin("certs", true);
  String deviceCertStr = prefs.getString("devicecert", "");
  prefs.end();
  
  if (deviceCertStr.isEmpty()) {
    Serial.println("⚠️ No certificates in preferences, using embedded");
    return false;
  }
  
  // Here you could add proper certificate parsing to check expiry
  // For now, we'll assume certificates are valid if they exist
  Serial.println("✅ Certificates appear valid");
  return true;
}

void provisionCertificates() {
  // Placeholder for certificate provisioning
  // In a real implementation, this might fetch certificates from a server
  // For now, we'll just check if we have valid certificates
  Serial.println("🔐 Checking certificate status...");
  checkCertificateExpiry();
}

void loadCertificates() {
  Serial.println("🔐 Loading certificates for MQTT connection...");
  
  prefs.begin("certs", true);
  String rootCAStr = prefs.getString("rootca", "");
  String deviceCertStr = prefs.getString("devicecert", "");
  String privateKeyStr = prefs.getString("privatekey", "");
  prefs.end();
  
  if (!rootCAStr.isEmpty() && !deviceCertStr.isEmpty() && !privateKeyStr.isEmpty()) {
    Serial.println("📋 Using certificates from preferences");
    net.setCACert(rootCAStr.c_str());
    net.setCertificate(deviceCertStr.c_str());
    net.setPrivateKey(privateKeyStr.c_str());
  } else {
    Serial.println("📋 Using embedded certificates");
    net.setCACert(rootCA);
    net.setCertificate(deviceCert);
    net.setPrivateKey(privateKey);
  }
  
  Serial.println("✅ Certificates loaded");
}

bool tryAlternativeUpdate(String url) {
  Serial.println("🔄 Attempting alternative update method...");
  
  // For GitHub releases, try using HTTPClient with redirect handling
  if (url.indexOf("github.com") != -1 && url.indexOf("releases/download") != -1) {
    Serial.println("🔄 Trying HTTPClient with redirect handling...");
    
    HTTPClient http;
    http.begin(url);
    http.setTimeout(30000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    
    int httpCode = http.GET();
    Serial.printf("HTTP Response code: %d\n", httpCode);
    
    if (httpCode == HTTP_CODE_OK) {
      int contentLength = http.getSize();
      Serial.printf("Content length: %d bytes\n", contentLength);
      
      if (contentLength > 0) {
        if (Update.begin(contentLength)) {
          size_t written = Update.writeStream(http.getStream());
          if (written == contentLength) {
            if (Update.end()) {
              Serial.println("✅ Alternative update completed successfully");
              http.end();
              return true;
            } else {
              Serial.printf("❌ Update end failed: %s\n", Update.errorString());
            }
          } else {
            Serial.printf("❌ Written size mismatch. Expected: %d, Got: %d\n", contentLength, written);
          }
        } else {
          Serial.printf("❌ Update begin failed: %s\n", Update.errorString());
        }
      } else {
        Serial.println("❌ Content length is 0");
      }
    } else {
      Serial.printf("❌ HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    
    http.end();
    return false;
  } else {
    // For non-GitHub URLs, use HTTPClient
    HTTPClient http;
    http.begin(url);
    http.setTimeout(30000);
    int httpCode = http.GET();
    Serial.printf("HTTP Response code: %d\n", httpCode);
    
    if (httpCode == HTTP_CODE_OK) {
      int contentLength = http.getSize();
      Serial.printf("Content length: %d bytes\n", contentLength);
      
      if (contentLength > 0) {
        if (Update.begin(contentLength)) {
          size_t written = Update.writeStream(http.getStream());
          if (written == contentLength) {
            if (Update.end()) {
              Serial.println("✅ Alternative update completed successfully");
              http.end();
              return true;
            } else {
              Serial.printf("❌ Update end failed: %s\n", Update.errorString());
            }
          } else {
            Serial.printf("❌ Written size mismatch. Expected: %d, Got: %d\n", contentLength, written);
          }
        } else {
          Serial.printf("❌ Update begin failed: %s\n", Update.errorString());
        }
      } else {
        Serial.println("❌ Content length is 0");
      }
    } else {
      Serial.printf("❌ HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    
    http.end();
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  esp_log_level_set("*", ESP_LOG_NONE); // Stops verbose serial logging, if it happens
  bootTime = millis();
  
  Serial.println("🚀 IoT Gateway starting up...");

  if (!tryWiFiConnection()) {
    startAPMode();
    return;
  }

  // Try to provision certificates if we have a license key
  if (!licenseKey.isEmpty()) {
    provisionCertificates();
  }
  
  loadCertificates();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  time_t now;
  while (time(&now) < 100000) delay(100);

  connectMQTT();
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("🛰️  Listening for TPMS packets...");
}

void loop() {
  if (inApMode) {
    dns.processNextRequest();
    server.handleClient();
    return;
  }

  mqtt.loop();

  // MQTT reconnect logic
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > 5000) { // 5 second backoff
      Serial.println("🔄 Attempting MQTT reconnect...");
      connectMQTT();
      lastMqttReconnectAttempt = now;
    }
  }

  static uint8_t packet[9];
  static int index = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    // Debug: Print first few bytes to see what we're receiving
    static int debugCount = 0;
    
    if (debugCount < 20) {
      Serial.printf("%02X", b);
      debugCount++;
      if (debugCount % 10 == 0) Serial.println();
    }
    
    if (index == 0 && (b != 0x80 && b != 0x84 && b != 0x88 && b != 0x90 && b != 0x98)) continue;
    packet[index++] = b;
    if (index == 9) {
      index = 0;
      
      // Validate packet - check for reasonable values
      if (packet[1] == 0x00 && packet[2] == 0x00 && packet[3] == 0x00) {
        // Skip packets with all-zero serial numbers (likely corrupted)
        continue;
      }
      
      // Check for reasonable pressure/temperature values
      if (packet[4] == 0x00 && packet[5] == 0x00) {
        // Skip packets with all-zero readings (likely corrupted)
        continue;
      }
      
      char serialHex[7];
      sprintf(serialHex, "%02X%02X%02X", packet[1], packet[2], packet[3]);
      
      // Check if we need to remove oldest sensor to make room
      if (sensorCache.size() >= MAX_SENSOR_CACHE_SIZE && sensorCache.find(serialHex) == sensorCache.end()) {
        // Remove the oldest sensor (first in map)
        String oldestSensor = sensorCache.begin()->second.serialNumber.c_str();
        sensorCache.erase(sensorCache.begin());
        Serial.println("🗑️ Removed sensor " + oldestSensor + " from cache (limit: " + String(MAX_SENSOR_CACHE_SIZE) + ")");
      }
      
      SensorReading reading = {
        .serialNumber = serialHex,
        .pressureHex = String(packet[4], HEX),
        .temperatureHex = String(packet[5], HEX),
        .rssi = packet[6],
        .ambient = packet[7]
      };
      sensorCache[serialHex] = reading;
    }
  }

  if (millis() - lastPublishTime >= intervalMinutes * 60UL * 1000UL) {
    Serial.println("Sending Cache!");
    publishSensorCache();
  }
}


