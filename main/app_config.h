#pragma once

// LilyGo T-Vending (ESP32-S3) onboard pins
// Source: https://github.com/Xinyuan-LilyGO/T-Vending examples/RS485_Master
#define BOARD_POWER_ON   4   // peripheral power enable — must be HIGH for RS485 transceiver
#define BOARD_485_RX     38
#define BOARD_485_TX     39
#define BOARD_485_EN     42  // RS485 driver enable — LOW = active (transceiver auto-directions)

#define WIFI_SSID "BT-P2CKZW"
#define WIFI_PASS "nFm6T3M9knXaTN"

#define BLYNK_TEMPLATE_ID  "TMPL4GQQAzx08"
#define BLYNK_AUTH_TOKEN   "M7q-VgyYY7Lpze1fNh80IlLJj0OCWkC4"

#define FIRMWARE_VERSION "1.3.0"

#define MQTT_BROKER_URI "mqtt://fra1.blynk.cloud:1883"

#define UPLOAD_INTERVAL_MS 1800000
#define MQTT_DEVICE_ID     "device1"
#define MQTT_PUB_ID        "device"
#define MQTT_PASSWORD      BLYNK_AUTH_TOKEN

// Blynk MQTT topics — the broker routes by auth token internally;
// device-side topics do NOT include the token prefix.
#define BLYNK_RELAY_TOPIC "downlink/ds/Relay"
#define BLYNK_INFO_TOPIC  "info/mcu"
#define BLYNK_BATCH_TOPIC "batch_ds"

#define SENSOR_SLAVE_ID   1
#define RELAY_SLAVE_ID    2
#define RELAY_CONTROL_REG 1
#define RELAY_VAL_ON      256 // 0x0100
#define RELAY_VAL_OFF     512 // 0x0200
