#define WifiSSID "WifiSSID"
#define WifiPassword "WifiPassword"
#define MQTTUsername "MQTTUsername"
#define MQTTPassword "MQTTPassword"
#define MQTTHost "MQTTHost"
#define MQTTCACert \
"-----BEGIN CERTIFICATE-----\n" \
"REPLACE_WITH_BROKER_CA_CERT_PEM\n" \
"-----END CERTIFICATE-----\n"
#define OtaUrl "https://OtaHost/lssensor.bin"
#define OtaUser "OtaUser"
#define OtaPassword "OtaPassword"
#define OTA_CHECK_INTERVAL_MS 60000
#define OtaRootCA R"EOF(
-----BEGIN CERTIFICATE-----
REPLACE_WITH_OTA_SERVER_ROOT_CA_PEM
-----END CERTIFICATE-----
)EOF"
