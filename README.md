# IoT Gateway - TPMS Sensor Gateway for AWS IoT

A robust ESP32-C6 based IoT gateway that reads TPMS (Tire Pressure Monitoring System) sensor packets via UART, processes them, and transmits data to AWS IoT Core via MQTT. Features Just-In-Time (JIT) certificate provisioning, OTA updates, and comprehensive device management.

## 🚀 Features

### **Core Functionality**
- **TPMS Sensor Processing**: Reads and processes 9-byte TPMS sensor packets via UART
- **AWS IoT Integration**: Secure MQTT communication with AWS IoT Core
- **JIT Certificate Provisioning**: Automatic device provisioning with unique certificates
- **OTA Updates**: Remote firmware updates via MQTT commands
- **WiFi Configuration**: AP mode for initial WiFi setup

### **Advanced Features**
- **Emergency Shutdown Protection**: Automatic shutdown on sensor data flooding
- **Hardware Recovery**: Boot button recovery for emergency shutdowns
- **Rate Limiting**: Configurable packet rate limiting (1000 packets/minute)
- **Sensor Caching**: Intelligent sensor data caching and management
- **Debug Mode**: Remote debug message publishing via MQTT

### **Security**
- **TLS 1.2 Encryption**: Secure MQTT communication
- **Certificate Management**: Secure certificate storage with embedded fallback
- **Device Authentication**: License key-based device identification
- **Policy Enforcement**: AWS IoT policy-based access control

## 📋 Requirements

### **Hardware**
- **ESP32-C6** development board (tested with XIAO ESP32-C6)
- **UART connection** for TPMS sensor data (RX: GPIO17, TX: GPIO18)
- **WiFi antenna** (GPIO14 for antenna configuration)
- **Boot button** (GPIO0) for emergency recovery

### **Software**
- **Arduino IDE 2.0+** or **PlatformIO**
- **ESP32 Arduino Core** v2.0.0+
- **Required Libraries**:
  - `WiFi.h` (included with ESP32 core)
  - `WiFiClientSecure.h` (included with ESP32 core)
  - `PubSubClient.h` (v2.8+)
  - `Preferences.h` (included with ESP32 core)
  - `WebServer.h` (included with ESP32 core)
  - `DNSServer.h` (included with ESP32 core)
  - `ArduinoJson.h` (v6.0+)
  - `HTTPClient.h` (included with ESP32 core)
  - `Update.h` (included with ESP32 core)

### **AWS Services**
- **AWS IoT Core** for MQTT communication
- **AWS Lambda** for JIT provisioning (see setup guide)
- **AWS IoT Rules** for message routing
- **CloudWatch** for logging and monitoring

## 🔧 Installation

### **1. Clone the Repository**
```bash
git clone <repository-url>
cd IoTGateway
```

### **2. Install Dependencies**
Install the required libraries through Arduino IDE Library Manager or PlatformIO.

### **3. Configure AWS IoT**
1. Create an AWS IoT Core endpoint
2. Set up JIT provisioning Lambda function
3. Configure IoT rules for message routing
4. Create provisioning certificates and policies

### **4. Update Configuration**
Edit `config.h` and `config.cpp`:
- Update `AWS_IOT_ENDPOINT` with your AWS IoT endpoint
- Replace embedded certificates with your provisioning certificates
- Adjust `INTERVAL_MINUTES` for data transmission frequency

### **5. Compile and Upload**
1. Select your ESP32-C6 board in Arduino IDE
2. Set correct port and upload settings
3. Compile and upload the firmware

## ⚙️ Configuration

### **Initial Setup**
1. **Power on the device** - it will start in AP mode
2. **Connect to WiFi** - SSID: `PressurePro-Gateway{MAC}`
3. **Configure settings** - Visit `http://192.168.4.1`
4. **Enter credentials**:
   - WiFi SSID and password
   - License key (required for device identification)
   - Device ID (optional)
   - Data transmission interval

### **AWS IoT Setup**
The device uses a two-tier certificate system:

1. **Provisioning Certificates** (embedded in firmware)
   - Used for initial connection and JIT provisioning
   - Limited permissions for provisioning topics only

2. **Device Certificates** (provisioned via JIT)
   - Unique per device
   - Full permissions for device-specific topics
   - Automatically generated and deployed

### **Topic Structure**
```
pressurepro/devices/WifiSensorGateway/
├── provisioning/
│   ├── request                    # Provisioning requests
│   └── {license_key}/response     # Provisioning responses
└── {license_key}/
    ├── sensor-readings            # TPMS sensor data
    ├── diagnostics                # Device diagnostics
    ├── commands                   # Remote commands
    └── firmware                   # Firmware update commands
```

## 📊 Data Format

### **Sensor Readings**
```json
{
  "timestamp": "Wed Jul 10 15:30:45 2024",
  "unix_time": 1720629045000,
  "sensor_data": [
    {
      "serialNumber": "A1B2C3",
      "pressure": "32",
      "temperature": "25",
      "rx_rssi": 45,
      "amb_rssi": 38
    }
  ],
  "device_info": {
    "device_id": "gateway-001",
    "firmware_version": "1.0.8",
    "license_key": "1234567"
  }
}
```

### **Diagnostics**
```json
{
  "device": {
    "license_key": "1234567",
    "device_id": "gateway-001",
    "firmware_version": "1.0.8",
    "hardware_id": "E4B323B5A6D8"
  },
  "connection": {
    "signal_strength": -45,
    "network_type": "WiFi",
    "ssid": "MyWiFi",
    "ip_address": "192.168.1.100",
    "connection_duration": 3600,
    "readings_sent": 15
  },
  "packets": {
    "total_received": 1250,
    "total_bytes": 11250,
    "invalid_count": 5,
    "valid_sensors": 8,
    "last_valid_packet": 30,
    "rate_limit": 45,
    "invalid_rate": 2,
    "emergency_shutdown": false,
    "emergency_shutdown_count": 0,
    "emergency_protection_disabled": false
  }
}
```

## 🔌 Commands

### **Remote Commands via MQTT**
Send JSON commands to `pressurepro/devices/WifiSensorGateway/{license_key}/commands`:

```json
{
  "action": "reboot"
}
```

**Available Commands**:
- `reboot` - Restart the device
- `reset_wifi` - Clear WiFi credentials and restart
- `firmware_update` - Update firmware from URL
- `update_certificates` - Update device certificates
- `reset_certificates` - Reset to embedded certificates
- `clear_cache` - Clear sensor cache
- `reset_stats` - Reset packet statistics
- `debug_mode` - Enable/disable debug mode
- `emergency_shutdown` - Manually trigger emergency shutdown

### **Firmware Updates**
```json
{
  "action": "firmware_update",
  "url": "https://example.com/firmware.bin"
}
```

## 🛠️ Troubleshooting

### **Common Issues**

1. **MQTT Connection Failures**
   - Check WiFi connectivity
   - Verify AWS IoT endpoint
   - Ensure certificates are valid
   - Check AWS IoT policies

2. **Provisioning Failures**
   - Verify Lambda function is configured
   - Check IoT rules are active
   - Ensure license key is unique
   - Review CloudWatch logs

3. **Emergency Shutdowns**
   - Check for sensor data flooding
   - Verify packet validation settings
   - Use boot button for recovery
   - Review diagnostic messages

### **Debug Mode**
Enable debug mode to get detailed logs:
```json
{
  "action": "debug_mode",
  "enabled": true
}
```

### **Hardware Recovery**
1. **Emergency shutdown recovery**: Press boot button during startup
2. **Factory reset**: Clear all preferences and restart
3. **AP mode**: Device automatically enters AP mode if WiFi fails

## 📈 Monitoring

### **Key Metrics**
- **Packet reception rate** - Total packets per minute
- **Invalid packet ratio** - Percentage of invalid packets
- **Sensor count** - Number of unique sensors detected
- **Connection stability** - MQTT connection duration
- **Emergency shutdowns** - Count and reasons

### **CloudWatch Integration**
- **AWS IoT logs** - Connection and message logs
- **Lambda logs** - Provisioning function logs
- **Custom metrics** - Device-specific performance data

## 🔒 Security Considerations

1. **Certificate Management**
   - Never commit production certificates to version control
   - Use environment variables for sensitive data
   - Regularly rotate certificates

2. **Network Security**
   - Use WPA3 WiFi when possible
   - Implement network segmentation
   - Monitor for unauthorized access

3. **Device Security**
   - Keep firmware updated
   - Monitor for suspicious activity
   - Implement rate limiting

## 📝 License

[Add your license information here]

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## 📞 Support

For support and questions:
- Create an issue in the repository
- Check the troubleshooting section
- Review AWS IoT documentation
- Contact the development team

---

**Version**: 1.0.8  
**Last Updated**: July 2024  
**Platform**: ESP32-C6  
**AWS IoT Compatible**: Yes 