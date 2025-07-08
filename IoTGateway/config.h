#pragma once

#define FIRMWARE_VERSION "1.0.4"
#define INTERVAL_MINUTES 5
#define AWS_IOT_ENDPOINT "a1vqmzc3f7mie5-ats.iot.us-west-2.amazonaws.com"

// ====== EMBEDDED CERTIFICATES (PEM format) ======
extern const char* rootCA;
extern const char* deviceCert;
extern const char* privateKey;
