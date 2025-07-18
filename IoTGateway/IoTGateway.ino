#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
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
#define AP_TIMEOUT_MS 15 * 60 * 1000
#define DNS_PORT 53
#define UART_RX 17
#define UART_TX 18
#define UART_BAUD 38400
#define MQTT_BUFFER_SIZE 16384
#define MAX_SENSOR_CACHE_SIZE 200
#define MAX_PACKETS_PER_MINUTE 1000  // Rate limiting
#define MAX_INVALID_PACKETS_PER_MINUTE 500  // Invalid packet rate limiting
#define EMERGENCY_SHUTDOWN_INVALID_RATIO 0.8  // 80% invalid packets triggers shutdown
#define RECOVERY_PIN 0  // GPIO0 - Boot button on most ESP32 boards
#define MAX_EMERGENCY_SHUTDOWNS 3  // Max emergency shutdowns before permanent disable

String licenseKey = "";
String deviceId = "";
String firmwareVersion = FIRMWARE_VERSION;
int intervalMinutes = INTERVAL_MINUTES;

Preferences prefs;
WebServer server(80);
DNSServer dns;
WiFiClientSecure net;
PubSubClient mqtt(net);

unsigned long bootTime;
unsigned long readingsSent = 0;
bool inApMode = false;
unsigned long lastPublishTime = 0;
unsigned long lastMqttReconnectAttempt = 0; // Track last MQTT reconnect
bool mqttConnecting = false; // Prevent multiple simultaneous connection attempts
bool deviceProvisioned = false; // Track if device has been provisioned with unique certificate
bool provisioningAttempted = false; // Track if we've already attempted provisioning this session
bool debugMode = false; // Enable serial output to MQTT
String debugBuffer = ""; // Buffer for debug messages
unsigned long lastDebugPublish = 0; // Track last debug publish time
unsigned long packetCount = 0; // Total packets received
unsigned long invalidPacketCount = 0; // Invalid packets received
unsigned long lastValidPacket = 0; // Last valid packet timestamp
unsigned long packetsThisMinute = 0; // Rate limiting counter
unsigned long invalidPacketsThisMinute = 0; // Invalid rate limiting counter
unsigned long lastMinuteReset = 0; // Last time we reset minute counters
bool emergencyShutdown = false; // Emergency shutdown flag
unsigned long totalBytesReceived = 0; // Total bytes received (for debugging)
unsigned long emergencyShutdownCount = 0; // Count of emergency shutdowns
bool emergencyProtectionDisabled = false; // Permanent disable flag
unsigned long lastInternetCheck = 0; // Track last internet connectivity check
unsigned long internetCheckInterval = 5 * 60 * 1000; // Check every 5 minutes

struct SensorReading {
  std::string serialNumber;
  String pressureHex;
  String temperatureHex;
  uint8_t rssi;
  uint8_t ambient;
};

std::map<std::string, SensorReading> sensorCache;

// Function declarations
void startAPMode();
void saveEmergencyShutdownCount();
bool tryAlternativeUpdate(String url);
void loadCertificates();
void flushDebugBuffer();

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
    if (action == "reset_stats") {
      Serial.println("📊 Resetting packet statistics...");
      packetCount = 0;
      invalidPacketCount = 0;
      lastValidPacket = millis();
      packetsThisMinute = 0;
      invalidPacketsThisMinute = 0;
      totalBytesReceived = 0;
      emergencyShutdown = false;
      Serial.println("✅ Packet statistics reset");
    }
    if (action == "debug_mode") {
      if (doc.containsKey("enabled")) {
        debugMode = doc["enabled"].as<bool>();
        Serial.println(debugMode ? "🐛 Debug mode ENABLED" : "🐛 Debug mode DISABLED");
        // Send immediate confirmation
        StaticJsonDocument<256> debugMsg;
        debugMsg["type"] = "debug_mode_change";
        debugMsg["enabled"] = debugMode;
        debugMsg["timestamp"] = String(millis());
        String debugPayload;
        serializeJson(debugMsg, debugPayload);
        String topic = mqttBaseTopic() + "diagnostics";
        Serial.println("📤 Publishing to topic: " + topic);
        mqtt.publish(topic.c_str(), debugPayload.c_str());
      } else {
        Serial.println("❌ Debug mode command missing 'enabled' field");
      }
    }
    if (action == "emergency_shutdown") {
      if (doc.containsKey("enabled")) {
        emergencyShutdown = doc["enabled"].as<bool>();
        Serial.println(emergencyShutdown ? "🚨 Emergency shutdown ENABLED" : "🚨 Emergency shutdown DISABLED");
        if (emergencyShutdown) {
          // Send emergency alert
          StaticJsonDocument<512> emergency;
          emergency["type"] = "emergency_shutdown";
          emergency["reason"] = "manual_trigger";
          emergency["timestamp"] = String(millis());
          
          String emergencyPayload;
          serializeJson(emergency, emergencyPayload);
          String topic = mqttBaseTopic() + "diagnostics";
          Serial.println("📤 Publishing to topic: " + topic);
          mqtt.publish(topic.c_str(), emergencyPayload.c_str());
          
          // Disconnect MQTT
          mqtt.disconnect();
          
          // Clear WiFi credentials and disconnect
          Serial.println("🔌 Disconnecting WiFi and clearing credentials...");
          prefs.begin("wifi", false);
          prefs.clear();
          prefs.end();
          
          // Reset global variables
          licenseKey = "";
          deviceId = "";
          intervalMinutes = INTERVAL_MINUTES;
          
          // Disconnect from WiFi
          WiFi.disconnect();
          WiFi.mode(WIFI_OFF);
          
          Serial.println("✅ WiFi credentials cleared and disconnected");
          
          // Start AP mode for reconfiguration
          startAPMode();
        }
      } else {
        Serial.println("❌ Emergency shutdown command missing 'enabled' field");
      }
    }

    if (action == "disable_emergency_protection") {
      emergencyProtectionDisabled = true;
      saveEmergencyShutdownCount();
      Serial.println("🚨 Emergency protection DISABLED permanently");
      
      // Send alert
      StaticJsonDocument<512> disableAlert;
      disableAlert["type"] = "emergency_protection_disabled";
      disableAlert["reason"] = "manual_disable";
      disableAlert["timestamp"] = String(millis());
      
      String disablePayload;
      serializeJson(disableAlert, disablePayload);
      String topic = mqttBaseTopic() + "diagnostics";
      Serial.println("📤 Publishing to topic: " + topic);
      mqtt.publish(topic.c_str(), disablePayload.c_str());
    }
    if (action == "debug_packets") {
      if (doc.containsKey("enabled")) {
        bool debugPackets = doc["enabled"].as<bool>();
        if (debugPackets) {
          Serial.println("🔍 PACKET DEBUG ENABLED - Will show all packet details");
          // This will be handled in the packet processing loop
        } else {
          Serial.println("🔍 PACKET DEBUG DISABLED");
        }
      }
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

void handleProvisioningResponse(String &topic, String &payload) {
  Serial.println("📥 Received provisioning response");
  
  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, payload)) {
    Serial.println("❌ Failed to parse provisioning response");
    return;
  }
  
  if (doc.containsKey("status") && doc["status"] == "success") {
    if (doc.containsKey("certificates")) {
      JsonObject certs = doc["certificates"];
      
      if (certs.containsKey("rootca") && certs.containsKey("devicecert") && certs.containsKey("privatekey")) {
        String rootCA = certs["rootca"].as<String>();
        String deviceCert = certs["devicecert"].as<String>();
        String privateKey = certs["privatekey"].as<String>();
        
        // Store the new certificates
        prefs.begin("certs", false);
        prefs.putString("rootca", rootCA);
        prefs.putString("devicecert", deviceCert);
        prefs.putString("privatekey", privateKey);
        prefs.end();
        
        Serial.println("✅ New certificates stored successfully");
        
        // Disconnect and reconnect with new certificates
        mqtt.disconnect();
        delay(1000);
        
        // Load the new certificates
        loadCertificates();
        
        // Give AWS IoT time to activate the new certificates
        Serial.println("⏳ Waiting 5 seconds for AWS IoT to activate new certificates...");
        delay(5000);
        
        // Test connection with new certificates
        Serial.println("🧪 Testing connection with new certificates...");
        connectMQTT();
        
        // If connection fails, try with provisioning certificates to verify the issue
        if (!mqtt.connected()) {
          Serial.println("❌ Connection with new certificates failed, testing with provisioning certificates...");
          
          // Temporarily switch back to provisioning certificates
          net.setCACert(::rootCA);  // Use global embedded certificates
          net.setCertificate(::deviceCert);
          net.setPrivateKey(::privateKey);
          
          // Try connecting with provisioning certificates
          String testClientId = "esp32c6-" + getHardwareId();
          Serial.println("🧪 Testing with provisioning certificates, client ID: " + testClientId);
          
          mqtt.setServer(AWS_IOT_ENDPOINT, 8883);
          mqtt.setCallback(messageReceived);
          mqtt.setKeepAlive(60);
          mqtt.setBufferSize(MQTT_BUFFER_SIZE);
          mqtt.setSocketTimeout(30);
          
          bool testConnect = mqtt.connect(testClientId.c_str());
          Serial.println("🧪 Provisioning certificate test result: " + String(testConnect ? "SUCCESS" : "FAILED"));
          Serial.println("🧪 MQTT state: " + String(mqtt.state()));
          
          // Switch back to provisioned certificates
          loadCertificates();
        }
        
        deviceProvisioned = true;
        Serial.println("🎉 Device provisioning completed successfully!");
      } else {
        Serial.println("❌ Provisioning response missing certificate data");
      }
    } else {
      Serial.println("❌ Provisioning response missing certificates");
    }
  } else {
    Serial.println("❌ Provisioning failed: " + (doc.containsKey("error") ? doc["error"].as<String>() : "Unknown error"));
  }
}

void messageReceived(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String payloadStr = "";
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }
  
  Serial.println("📥 MQTT message received:");
  Serial.println("   Topic: " + topicStr);
  Serial.println("   Payload length: " + String(length) + " bytes");
  Serial.println("   Payload: " + payloadStr.substring(0, min(200, (int)payloadStr.length())) + (payloadStr.length() > 200 ? "..." : ""));
  
  if (topicStr.endsWith("commands")) handleCommandMessage(topicStr, payloadStr);
  else if (topicStr.endsWith("firmware")) handleFirmwareMessage(topicStr, payloadStr);
  else if (topicStr.indexOf("/provisioning/") != -1) handleProvisioningResponse(topicStr, payloadStr);
}

void connectMQTT() {
  Serial.println("🔗 ========================================");
  Serial.println("🔗 Attempting MQTT Connection");
  Serial.println("🔗 ========================================");
  Serial.println("🌐 MQTT Broker: " + String(AWS_IOT_ENDPOINT));
  
  // Clear any existing connection
  if (mqtt.connected()) {
    Serial.println("🔌 Disconnecting existing MQTT connection...");
    mqtt.disconnect();
    delay(1000);
  }
  
  // Set up PubSubClient
  Serial.println("⚙️ Configuring MQTT client...");
  mqtt.setServer(AWS_IOT_ENDPOINT, 8883);
  mqtt.setCallback(messageReceived);
  mqtt.setKeepAlive(60); // 60 second keepalive (more stable)
  mqtt.setBufferSize(MQTT_BUFFER_SIZE);
  mqtt.setSocketTimeout(30); // 30 second socket timeout (more generous)

  // Generate unique client ID using MAC address
  String clientId = "esp32c6-" + getHardwareId();
  Serial.println("🆔 Client ID: " + clientId);
  
  Serial.println("🔐 Certificate status: " + (deviceProvisioned ? "PROVISIONED" : "PROVISIONING"));
  
  Serial.print("🔗 Connecting to MQTT broker");
  int attempts = 0;
  while (!mqtt.connect(clientId.c_str()) && attempts < 10) {
    Serial.print(".");
    delay(1000);
    attempts++;
  }
  Serial.println();
  
  if (mqtt.connected()) {
    Serial.println("✅ MQTT connected successfully!");
    Serial.println("📊 MQTT Connection Details:");
    Serial.println("   Client ID: " + clientId);
    Serial.println("   Broker: " + String(AWS_IOT_ENDPOINT));
    Serial.println("   Port: 8883 (TLS)");
    Serial.println("   Keepalive: 60 seconds");
    Serial.println("   Buffer Size: " + String(MQTT_BUFFER_SIZE) + " bytes");
    
    // Subscribe to topics
    Serial.println("📡 Subscribing to MQTT topics...");
    mqtt.subscribe((mqttBaseTopic() + "commands").c_str());
    mqtt.subscribe((mqttBaseTopic() + "firmware").c_str());
    mqtt.subscribe("pressurepro/devices/WifiSensorGateway/firmware");
    Serial.println("✅ MQTT subscriptions active");
    
    // If using provisioning certificates, send provisioning request
    if (!deviceProvisioned && !provisioningAttempted) {
      Serial.println("🔐 ========================================");
      Serial.println("🔐 Starting Certificate Provisioning");
      Serial.println("🔐 ========================================");
      provisioningAttempted = true; // Mark that we've attempted provisioning
      
      // Subscribe to provisioning response topic
      String provisioningTopic = "pressurepro/devices/WifiSensorGateway/provisioning/" + licenseKey + "/response";
      Serial.println("📡 Subscribing to provisioning response: " + provisioningTopic);
      mqtt.subscribe(provisioningTopic.c_str());
      
      // Send provisioning request
      StaticJsonDocument<512> request;
      request["type"] = "provisioning_request";
      request["device_id"] = licenseKey;
      request["hardware_model"] = "ESP32-C6";
      request["firmware_version"] = firmwareVersion;
      request["client_id"] = "esp32c6-" + getHardwareId();
      request["timestamp"] = String(millis());
      
      String requestPayload;
      serializeJson(request, requestPayload);
      
      String requestTopic = "pressurepro/devices/WifiSensorGateway/provisioning/request";
      Serial.println("📤 Sending provisioning request to: " + requestTopic);
      bool publishResult = mqtt.publish(requestTopic.c_str(), requestPayload.c_str());
      
      if (publishResult) {
        Serial.println("✅ Provisioning request sent successfully");
        Serial.println("⏳ Waiting for provisioning response (30 second timeout)...");
        
        // Wait for response (with timeout)
        unsigned long startTime = millis();
        while (millis() - startTime < 30000) { // 30 second timeout
          mqtt.loop();
          
          if (deviceProvisioned) {
            Serial.println("🎉 Device successfully provisioned!");
            Serial.println("🔐 ========================================");
            Serial.println("🔐 Provisioning completed successfully");
            Serial.println("🔐 ========================================");
            return; // Exit the function to reconnect with new certificates
          }
          
          delay(100);
        }
        
        Serial.println("❌ Provisioning timeout - no response received");
        Serial.println("🔐 ========================================");
        Serial.println("🔐 Provisioning failed - continuing with provisioning certificates");
        Serial.println("🔐 ========================================");
      } else {
        Serial.println("❌ Failed to send provisioning request");
        Serial.println("🔐 ========================================");
        Serial.println("🔐 Provisioning failed - continuing with provisioning certificates");
        Serial.println("🔐 ========================================");
      }
    } else if (deviceProvisioned) {
      Serial.println("✅ Device already provisioned with unique certificates");
    }
    
    Serial.println("🔗 ========================================");
    Serial.println("🔗 MQTT Connection Established Successfully");
    Serial.println("🔗 ========================================");
  } else {
    Serial.println("❌ MQTT connection failed after " + String(attempts) + " attempts");
    Serial.println("🔍 Debug info:");
    Serial.println("   WiFi status: " + String(WiFi.status()) + 
                  " (" + (WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED") + ")");
    Serial.println("   WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
    Serial.println("   IP Address: " + WiFi.localIP().toString());
    Serial.println("   MQTT State: " + String(mqtt.state()));
    Serial.println("🔗 ========================================");
    Serial.println("🔗 MQTT Connection Failed");
    Serial.println("🔗 ========================================");
  }
}

bool checkInternetConnectivity() {
  Serial.println("🌐 ========================================");
  Serial.println("🌐 Checking Internet Connectivity");
  Serial.println("🌐 ========================================");
  
  // Try to connect to a reliable external service
  HTTPClient http;
  http.setTimeout(10000); // 10 second timeout
  
  // Try multiple endpoints for redundancy
  const char* endpoints[] = {
    "http://www.google.com",
    "http://www.cloudflare.com", 
    "http://www.amazon.com"
  };
  
  for (int i = 0; i < 3; i++) {
    Serial.println("🔍 Testing connectivity to: " + String(endpoints[i]));
    
    http.begin(endpoints[i]);
    int httpCode = http.GET();
    http.end();
    
    if (httpCode == HTTP_CODE_OK) {
      Serial.println("✅ Internet connectivity confirmed via " + String(endpoints[i]));
      Serial.println("🌐 ========================================");
      Serial.println("🌐 Internet connectivity: AVAILABLE");
      Serial.println("🌐 ========================================");
      return true;
    } else {
      Serial.println("❌ Failed to reach " + String(endpoints[i]) + " (HTTP: " + String(httpCode) + ")");
    }
    
    delay(1000); // Wait between attempts
  }
  
  Serial.println("❌ All connectivity tests failed");
  Serial.println("🌐 ========================================");
  Serial.println("🌐 Internet connectivity: UNAVAILABLE");
  Serial.println("🌐 ========================================");
  return false;
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

  Serial.println("📡 Configuring WiFi antenna...");
  pinMode(WIFI_ANT_CONFIG, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG, HIGH);

  Serial.println("🔗 Connecting to WiFi network: " + ssid);
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long startAttemptTime = millis();
  
  Serial.print("⏳ Waiting for connection");
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < AP_TIMEOUT_MS) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi connection failed after " + String((millis() - startAttemptTime) / 1000) + " seconds");
    Serial.println("🔍 WiFi status: " + String(WiFi.status()));
    return false;
  } else {
    Serial.println("✅ WiFi connected successfully");
    Serial.println("📊 Connection details:");
    Serial.println("   SSID: " + WiFi.SSID());
    Serial.println("   IP Address: " + WiFi.localIP().toString());
    Serial.println("   Gateway: " + WiFi.gatewayIP().toString());
    Serial.println("   DNS: " + WiFi.dnsIP().toString());
    Serial.println("   Signal Strength: " + String(WiFi.RSSI()) + " dBm");
  }
  
  // Check internet connectivity
  Serial.println("🌐 Checking internet connectivity...");
  if (!checkInternetConnectivity()) {
    Serial.println("⚠️ WiFi connected but no internet access - starting AP mode");
    return false;
  }

  Serial.println("✅ Internet connectivity confirmed");
  return true;
}

void startAPMode() {
  Serial.println("📶 ========================================");
  Serial.println("📶 Starting AP (Access Point) Mode");
  Serial.println("📶 ========================================");
  
  // Disconnect from any existing WiFi connection
  Serial.println("🔌 Disconnecting from existing WiFi...");
  WiFi.disconnect();
  delay(100);
  
  // Set WiFi mode to AP only
  Serial.println("📡 Configuring WiFi for AP mode...");
  WiFi.mode(WIFI_AP);
  delay(100);
  
  inApMode = true;
  String ssid = "PressurePro-Gateway" + getHardwareId();
  Serial.println("📶 Creating access point: " + ssid);
  WiFi.softAP(ssid.c_str(), "configureme");
  
  Serial.println("🌐 Starting DNS server...");
  dns.start(DNS_PORT, "*", WiFi.softAPIP());

  Serial.println("🌐 Setting up web server...");
  server.onNotFound([]() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
  });

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<form method='POST'>SSID: <input name='ssid'><br>Password: <input name='password'><br>License Key: <input name='license'><br>Device ID: <input name='device_id'><br>Interval (min): <input name='interval'><br><input type='submit'></form>");
  });

  server.on("/", HTTP_POST, []() {
    Serial.println("📝 Configuration received from web interface");
    prefs.begin("wifi", false);
    prefs.putString("ssid", server.arg("ssid"));
    prefs.putString("password", server.arg("password"));
    prefs.putString("license", server.arg("license"));
    prefs.putString("device_id", server.arg("device_id"));
    prefs.putInt("interval", server.arg("interval").toInt());
    prefs.end();
    server.send(200, "text/html", "Saved. Rebooting...");
    Serial.println("✅ Configuration saved - rebooting in 2 seconds...");
    delay(2000);
    ESP.restart();
  });

  server.begin();
  Serial.println("✅ AP Mode Started Successfully");
  Serial.println("📊 AP Details:");
  Serial.println("   SSID: " + ssid);
  Serial.println("   Password: configureme");
  Serial.println("   IP Address: " + WiFi.softAPIP().toString());
  Serial.println("   DNS Server: Active on port 53");
  Serial.println("   Web Server: Active on port 80");
  Serial.println("📶 ========================================");
  Serial.println("📶 Operating in AP CONFIGURATION mode");
  Serial.println("📶 Connect to WiFi network above to configure");
  Serial.println("📶 ========================================");
}

void publishSensorCache() {
  StaticJsonDocument<1024> doc;  // Adjust size based on actual needs
  time_t now;
  time(&now);

  String timestamp = String(ctime(&now));
  timestamp.trim(); // Remove trailing newline
  doc["timestamp"] = timestamp.c_str();
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
  String sensorTopic = mqttBaseTopic() + "sensor-readings";
  Serial.println("📤 Publishing to topic: " + sensorTopic);
  mqtt.publish(sensorTopic.c_str(), payload.c_str());

  // Diagnostics
  StaticJsonDocument<512> diag;
  
  // Add device identification
  JsonObject device = diag.createNestedObject("device");
  device["license_key"] = licenseKey;
  device["device_id"] = deviceId;
  device["firmware_version"] = firmwareVersion;
  device["hardware_id"] = getHardwareId();
  
  JsonObject conn = diag.createNestedObject("connection");
  conn["signal_strength"] = WiFi.RSSI();
  conn["network_type"] = "WiFi";
  conn["ssid"] = WiFi.SSID();
  conn["ip_address"] = WiFi.localIP().toString();
  conn["connection_duration"] = (millis() - bootTime) / 1000;
  conn["readings_sent"] = ++readingsSent;
  
  // Add packet statistics
  JsonObject packets = diag.createNestedObject("packets");
  packets["total_received"] = packetCount;
  packets["total_bytes"] = totalBytesReceived;
  packets["invalid_count"] = invalidPacketCount;
  packets["valid_sensors"] = sensorCache.size();
  packets["last_valid_packet"] = (millis() - lastValidPacket) / 1000; // seconds ago
  packets["rate_limit"] = packetsThisMinute;
  packets["invalid_rate"] = invalidPacketsThisMinute;
  packets["emergency_shutdown"] = emergencyShutdown;
  packets["emergency_shutdown_count"] = emergencyShutdownCount;
  packets["emergency_protection_disabled"] = emergencyProtectionDisabled;

  String diagPayload;
  serializeJson(diag, diagPayload);
  String diagTopic = mqttBaseTopic() + "diagnostics";
  Serial.println("📤 Publishing to topic: " + diagTopic);
  mqtt.publish(diagTopic.c_str(), diagPayload.c_str());

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
  if (deviceProvisioned) {
    Serial.println("✅ Device already provisioned with unique certificate");
    return;
  }
  
  Serial.println("🔐 Device needs provisioning - will request during MQTT connection");
}



void loadCertificates() {
  Serial.println("🔐 ========================================");
  Serial.println("🔐 Loading Certificates");
  Serial.println("🔐 ========================================");
  
  prefs.begin("certs", true);
  String rootCAStr = prefs.getString("rootca", "");
  String deviceCertStr = prefs.getString("devicecert", "");
  String privateKeyStr = prefs.getString("privatekey", "");
  prefs.end();
  
  if (!rootCAStr.isEmpty() && !deviceCertStr.isEmpty() && !privateKeyStr.isEmpty()) {
    Serial.println("📋 Using provisioned device certificates");
    Serial.println("🔍 Certificate details:");
    Serial.println("   Root CA: " + String(rootCAStr.length()) + " characters");
    Serial.println("   Device Cert: " + String(deviceCertStr.length()) + " characters");
    Serial.println("   Private Key: " + String(privateKeyStr.length()) + " characters");
    
    Serial.println("🔐 Setting certificate chain...");
    net.setCACert(rootCAStr.c_str());
    net.setCertificate(deviceCertStr.c_str());
    net.setPrivateKey(privateKeyStr.c_str());
    deviceProvisioned = true;
    
    Serial.println("✅ Provisioned certificates loaded successfully");
  } else {
    Serial.println("📋 Using provisioning certificates (for JIT setup)");
    Serial.println("🔍 Certificate details:");
    Serial.println("   Root CA: " + String(strlen(rootCA)) + " characters");
    Serial.println("   Device Cert: " + String(strlen(deviceCert)) + " characters");
    Serial.println("   Private Key: " + String(strlen(privateKey)) + " characters");
    
    Serial.println("🔐 Setting certificate chain...");
    net.setCACert(rootCA);
    net.setCertificate(deviceCert);
    net.setPrivateKey(privateKey);
    deviceProvisioned = false;
    
    Serial.println("✅ Provisioning certificates loaded successfully");
  }
  
  Serial.println("🔐 ========================================");
  Serial.println("🔐 Certificates loaded successfully");
  Serial.println("🔐 ========================================");
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

void sendDebugMessage(const String& message) {
  if (!debugMode || !mqtt.connected()) return;
  
  // Add message to buffer
  debugBuffer += message;
  
  // Send buffer if it's getting large or enough time has passed
  unsigned long now = millis();
  if (debugBuffer.length() > 500 || (now - lastDebugPublish > 5000)) {
    flushDebugBuffer();
  }
}

void flushDebugBuffer() {
  if (debugBuffer.isEmpty() || !mqtt.connected()) return;
  
  StaticJsonDocument<1024> debugMsg;
  debugMsg["type"] = "serial_output";
  debugMsg["message"] = debugBuffer;
  debugMsg["timestamp"] = String(millis());
  debugMsg["device_id"] = deviceId;
  
  String debugPayload;
  serializeJson(debugMsg, debugPayload);
  String topic = mqttBaseTopic() + "diagnostics";
  Serial.println("📤 Publishing to topic: " + topic);
  mqtt.publish(topic.c_str(), debugPayload.c_str());
  
  debugBuffer = "";
  lastDebugPublish = millis();
}

// Override Serial.print to capture debug output
void debugPrint(const String& message) {
  Serial.print(message);
  sendDebugMessage(message);
}

void debugPrintln(const String& message) {
  Serial.println(message);
  sendDebugMessage(message + "\n");
}

void checkHardwareRecovery() {
  // Check if recovery pin is held down (GPIO0 pulled low)
  pinMode(RECOVERY_PIN, INPUT_PULLUP);
  delay(100); // Debounce
  
  if (digitalRead(RECOVERY_PIN) == LOW) {
    Serial.println("🔧 HARDWARE RECOVERY TRIGGERED - Holding boot button detected");
    
    // Flash LED to indicate recovery mode
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
    
    // Clear all preferences
    prefs.begin("wifi", false);
    prefs.clear();
    prefs.end();
    
    prefs.begin("certs", false);
    prefs.clear();
    prefs.end();
    
    // Reset all counters and flags
    emergencyShutdownCount = 0;
    emergencyProtectionDisabled = false;
    emergencyShutdown = false;
    packetsThisMinute = 0;
    invalidPacketsThisMinute = 0;
    packetCount = 0;
    invalidPacketCount = 0;
    totalBytesReceived = 0;
    
    Serial.println("✅ FACTORY RESET COMPLETE - Rebooting...");
    delay(2000);
    ESP.restart();
  }
}

void loadEmergencyShutdownCount() {
  prefs.begin("emergency", true);
  emergencyShutdownCount = prefs.getULong("shutdown_count", 0);
  emergencyProtectionDisabled = prefs.getBool("protection_disabled", false);
  prefs.end();
}

void saveEmergencyShutdownCount() {
  prefs.begin("emergency", false);
  prefs.putULong("shutdown_count", emergencyShutdownCount);
  prefs.putBool("protection_disabled", emergencyProtectionDisabled);
  prefs.end();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  esp_log_level_set("*", ESP_LOG_NONE); // Stops verbose serial logging, if it happens
  bootTime = millis();
  
  Serial.println("🚀 ========================================");
  Serial.println("🚀 IoT Gateway Starting Up");
  Serial.println("🚀 ========================================");
  
  // Print hardware info
  Serial.println("🔧 Hardware Info:");
  Serial.println("   Chip: " + String(ESP.getChipModel()));
  Serial.println("   Cores: " + String(ESP.getChipCores()));
  Serial.println("   Flash: " + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB");
  Serial.println("   Free heap: " + String(ESP.getFreeHeap()) + " bytes");
  Serial.println("   SDK version: " + String(ESP.getSdkVersion()));
  
  // Simple memory test
  Serial.println("🧪 Memory test:");
  void* testAlloc = malloc(1024);
  if (testAlloc) {
    Serial.println("   ✅ 1KB allocation successful");
    free(testAlloc);
  } else {
    Serial.println("   ❌ 1KB allocation failed");
  }
  
  // Load emergency shutdown count from persistent storage
  loadEmergencyShutdownCount();
  
  // Check for hardware recovery (boot button held down)
  checkHardwareRecovery();
  
  // Reset emergency shutdown state and counters on boot
  emergencyShutdown = false;
  packetsThisMinute = 0;
  invalidPacketsThisMinute = 0;
  lastMinuteReset = millis();
  provisioningAttempted = false; // Reset provisioning attempt flag on boot
  
  Serial.println("🔍 ========================================");
  Serial.println("🔍 Starting Device Initialization");
  Serial.println("🔍 ========================================");

  Serial.println("📡 Attempting WiFi connection...");
  if (!tryWiFiConnection()) {
    Serial.println("❌ WiFi connection failed - entering AP mode");
    startAPMode();
    return;
  }

  // Try to provision certificates if we have a license key
  Serial.println("🔍 License key check: " + (licenseKey.isEmpty() ? "EMPTY" : licenseKey.substring(0, 8) + "..."));
  if (!licenseKey.isEmpty()) {
    Serial.println("🔐 Certificate provisioning check...");
    provisionCertificates();
  } else {
    Serial.println("⚠️ No license key found - will use provisioning certificates");
  }
  
  Serial.println("🔐 Loading certificates...");
  loadCertificates();
  
  Serial.println("⏰ Setting up time synchronization...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  time_t now;
  while (time(&now) < 100000) delay(100);
  Serial.println("✅ Time synchronized");

  Serial.println("🔗 Attempting MQTT connection...");
  connectMQTT();
  
  Serial.println("🛰️ Initializing TPMS sensor interface...");
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("✅ TPMS interface ready - listening for packets...");
  
  // Give system time to stabilize before processing packets
  delay(2000);
  
  Serial.println("✅ ========================================");
  Serial.println("✅ Device initialization complete");
  Serial.println("✅ Operating in NORMAL mode");
  Serial.println("✅ ========================================");
}

void loop() {
  if (inApMode) {
    dns.processNextRequest();
    server.handleClient();
    return;
  }

  mqtt.loop();
  
  // Debug MQTT connection status (only every 60 seconds to avoid spam)
  static unsigned long lastDebugTime = 0;
  unsigned long now = millis();
  if (now - lastDebugTime > 60000) { // 60 seconds
    Serial.println("📊 ========================================");
    Serial.println("📊 Device Status Report");
    Serial.println("📊 ========================================");
    Serial.println("🔧 Operating Mode: NORMAL");
    Serial.println("📡 WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED"));
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("   SSID: " + WiFi.SSID());
      Serial.println("   IP: " + WiFi.localIP().toString());
      Serial.println("   RSSI: " + String(WiFi.RSSI()) + " dBm");
    }
    Serial.println("🔗 MQTT Status: " + String(mqtt.connected() ? "CONNECTED" : "DISCONNECTED"));
    if (mqtt.connected()) {
      Serial.println("   Broker: " + String(AWS_IOT_ENDPOINT));
      Serial.println("   Client ID: esp32c6-" + getHardwareId());
    }
    Serial.println("🔐 Certificate Status: " + String(deviceProvisioned ? "PROVISIONED" : "PROVISIONING"));
    Serial.println("📦 Sensor Cache: " + String(sensorCache.size()) + " sensors");
    Serial.println("📊 Packet Stats: " + String(packetCount) + " total, " + String(invalidPacketCount) + " invalid");
    Serial.println("💾 Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("⏱️ Uptime: " + String((millis() - bootTime) / 1000) + " seconds");
    Serial.println("📊 ========================================");
    lastDebugTime = now;
  }

  // MQTT reconnect logic - only reconnect if actually disconnected
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected() && !mqttConnecting) {
    if (now - lastMqttReconnectAttempt > 30000) { // 30 second backoff to avoid rate limiting
      mqttConnecting = true;
      Serial.println("🔄 MQTT disconnected, attempting reconnect...");
      Serial.println("🔍 Last known state - WiFi: " + String(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED") + 
                    ", MQTT: DISCONNECTED, Heap: " + String(ESP.getFreeHeap()));
      connectMQTT();
      lastMqttReconnectAttempt = now;
      mqttConnecting = false;
      delay(5000); // Give MQTT time to establish connection
    }
  }

  static uint8_t packet[9];
  static int index = 0;
  
  // Reset minute counters every minute
  if (now - lastMinuteReset >= 60000) { // 1 minute
    packetsThisMinute = 0;
    invalidPacketsThisMinute = 0;
    lastMinuteReset = now;
  }
  
  // Emergency shutdown check - TEMPORARILY DISABLED FOR DEPLOYMENT
  // TODO: Re-enable after packet validation is fixed
  /*
  if (!emergencyProtectionDisabled && 
      packetsThisMinute >= 10 && 
      (float)invalidPacketsThisMinute / packetsThisMinute > EMERGENCY_SHUTDOWN_INVALID_RATIO &&
      (millis() - bootTime) > 30000) { // 30 second grace period after boot
  */
  if (false) { // TEMPORARILY DISABLED
    if (debugMode) {
      debugPrintln("⚠️ Emergency shutdown conditions met: " + String(packetsThisMinute) + " packets, " + 
                  String(invalidPacketsThisMinute) + " invalid, ratio: " + 
                  String((float)invalidPacketsThisMinute / packetsThisMinute, 2));
    }
    if (!emergencyShutdown) {
      emergencyShutdown = true;
      emergencyShutdownCount++;
      saveEmergencyShutdownCount();
      
      // Check if we've exceeded the maximum emergency shutdowns
      if (emergencyShutdownCount >= MAX_EMERGENCY_SHUTDOWNS) {
        emergencyProtectionDisabled = true;
        saveEmergencyShutdownCount();
        debugPrintln("🚨 EMERGENCY PROTECTION PERMANENTLY DISABLED - Too many shutdowns (" + String(emergencyShutdownCount) + ")");
        
        // Send alert about permanent disable
        StaticJsonDocument<512> disableAlert;
        disableAlert["type"] = "emergency_protection_disabled";
        disableAlert["reason"] = "too_many_shutdowns";
        disableAlert["shutdown_count"] = emergencyShutdownCount;
        disableAlert["timestamp"] = String(millis());
        
        String disablePayload;
        serializeJson(disableAlert, disablePayload);
        String topic = mqttBaseTopic() + "diagnostics";
        Serial.println("📤 Publishing to topic: " + topic);
        mqtt.publish(topic.c_str(), disablePayload.c_str());
        
        // Don't actually shutdown, just disable protection
        emergencyShutdown = false;
        return;
      }
      debugPrintln("🚨 EMERGENCY SHUTDOWN: Too many invalid packets (" + String(invalidPacketsThisMinute) + "/" + String(packetsThisMinute) + ")");
      
      // Send emergency alert
      StaticJsonDocument<512> emergency;
      emergency["type"] = "emergency_shutdown";
      emergency["reason"] = "invalid_packet_flood";
      emergency["invalid_ratio"] = (float)invalidPacketsThisMinute / packetsThisMinute;
      emergency["invalid_count"] = invalidPacketsThisMinute;
      emergency["total_count"] = packetsThisMinute;
      emergency["timestamp"] = String(now);
      
      String emergencyPayload;
      serializeJson(emergency, emergencyPayload);
      String topic = mqttBaseTopic() + "diagnostics";
      Serial.println("📤 Publishing to topic: " + topic);
      mqtt.publish(topic.c_str(), emergencyPayload.c_str());
      
      // Disconnect MQTT to stop flooding
      mqtt.disconnect();
      
      // Clear WiFi credentials and disconnect
      debugPrintln("🔌 Disconnecting WiFi and clearing credentials...");
      prefs.begin("wifi", false);
      prefs.clear();
      prefs.end();
      
      // Reset global variables
      licenseKey = "";
      deviceId = "";
      intervalMinutes = INTERVAL_MINUTES;
      
      // Disconnect from WiFi
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      
      debugPrintln("✅ WiFi credentials cleared and disconnected");
      
      // Start AP mode for reconfiguration
      startAPMode();
    }
    return; // Stop processing packets
  }
  
  // Rate limiting check
  if (packetsThisMinute >= MAX_PACKETS_PER_MINUTE) {
    if (debugMode) {
      debugPrintln("⚠️ Rate limit reached: " + String(packetsThisMinute) + " packets/minute");
    }
    return; // Stop processing packets this minute
  }
  
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    totalBytesReceived++; // Count total bytes for debugging
    
    // Debug: Print first few bytes to see what we're receiving
    static int debugCount = 0;
    
    if (debugCount < 20) {
      debugPrint(String(b, HEX));
      debugCount++;
      if (debugCount % 10 == 0) debugPrintln("");
    }
    
    // Look for packet start
    if (index == 0) {
      if (b != 0x80 && b != 0x84 && b != 0x88 && b != 0x90 && b != 0x98) {
        continue; // Not a valid start byte
      }
    }
    
    packet[index++] = b;
    
    if (index == 9) {
      index = 0;
      packetCount++; // Only count complete 9-byte packets
      packetsThisMinute++;
      invalidPacketCount++;
      invalidPacketsThisMinute++;
      
      // Enhanced packet validation
      bool isValidPacket = true;
      String validationErrors = "";
      
      // 1. Check for all-zero serial numbers
      if (packet[1] == 0x00 && packet[2] == 0x00 && packet[3] == 0x00) {
        isValidPacket = false;
        validationErrors += "zero_serial ";
      }
      
      // 2. Check for all-zero readings
      if (packet[4] == 0x00 && packet[5] == 0x00) {
        isValidPacket = false;
        validationErrors += "zero_readings ";
      }
      
      // 3. Check for all-FF values (common corruption pattern)
      if (packet[1] == 0xFF && packet[2] == 0xFF && packet[3] == 0xFF) {
        isValidPacket = false;
        validationErrors += "ff_serial ";
      }
      
      // 4. Check for reasonable pressure range (0-255 PSI is valid)
      // Removed overly strict pressure validation - let all values through
      
      // 5. Check for reasonable temperature range (0-255°C is valid)
      // Removed overly strict temperature validation - let all values through
      
      // 6. Check RSSI range (should be 0-255)
      if (packet[6] > 255) {
        isValidPacket = false;
        validationErrors += "invalid_rssi ";
      }
      
      // 7. Check ambient range (should be 0-255)
      if (packet[7] > 255) {
        isValidPacket = false;
        validationErrors += "invalid_ambient ";
      }
      
      // 8. Check for heartbeat messages (FA3000 pattern with all-zero values)
      if (packet[1] == 0xFA && packet[2] == 0x30 && packet[3] == 0x00) {
        isValidPacket = false;
        validationErrors += "heartbeat_message ";
      }
      
      // 9. Check for all-zero readings (pressure, temperature, RSSI, ambient)
      if (packet[4] == 0x00 && packet[5] == 0x00 && packet[6] == 0x00 && packet[7] == 0x00) {
        isValidPacket = false;
        validationErrors += "all_zero_readings ";
      }
      
      // Apply validation
      if (!isValidPacket) {
        if (debugMode) {
          debugPrintln("❌ Invalid packet: " + validationErrors + " | " + 
                      String(packet[0], HEX) + " " + String(packet[1], HEX) + " " + 
                      String(packet[2], HEX) + " " + String(packet[3], HEX) + " " + 
                      String(packet[4], HEX) + " " + String(packet[5], HEX) + " " + 
                      String(packet[6], HEX) + " " + String(packet[7], HEX) + " " + 
                      String(packet[8], HEX));
        }
        continue;
      }
      
      // Packet is valid!
      lastValidPacket = millis();
      invalidPacketCount = 0; // Reset invalid count on valid packet
      
      char serialHex[7];
      sprintf(serialHex, "%02X%02X%02X", packet[1], packet[2], packet[3]);
      
      // Check if we need to remove oldest sensor to make room
      if (sensorCache.size() >= MAX_SENSOR_CACHE_SIZE && sensorCache.find(serialHex) == sensorCache.end()) {
        // Remove the oldest sensor (first in map)
        String oldestSensor = sensorCache.begin()->second.serialNumber.c_str();
        sensorCache.erase(sensorCache.begin());
        debugPrintln("🗑️ Removed sensor " + oldestSensor + " from cache (limit: " + String(MAX_SENSOR_CACHE_SIZE) + ")");
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
    debugPrintln("Sending Cache!");
    publishSensorCache();
  }
  
  // Periodic internet connectivity check
  if (millis() - lastInternetCheck >= internetCheckInterval) {
    lastInternetCheck = millis();
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("🔍 Periodic internet connectivity check...");
      if (!checkInternetConnectivity()) {
        Serial.println("⚠️ Internet connectivity lost - attempting to reconnect...");
        
        // Try to reconnect WiFi first
        Serial.println("🔄 Disconnecting WiFi to attempt reconnection...");
        WiFi.disconnect();
        delay(2000);
        
        Serial.println("🔄 Attempting WiFi reconnection...");
        if (!tryWiFiConnection()) {
          Serial.println("❌ Failed to reconnect with internet - restarting device");
          Serial.println("🔄 Device restart in 2 seconds...");
          delay(2000);
          ESP.restart();
        } else {
          Serial.println("✅ WiFi reconnection successful");
        }
      } else {
        Serial.println("✅ Periodic internet check passed");
      }
    } else {
      Serial.println("⚠️ WiFi disconnected during periodic check");
    }
  }
}


