#pragma once

// LilyGo T-Vending (ESP32-S3) onboard pins
// Source: https://github.com/Xinyuan-LilyGO/T-Vending examples/RS485_Master
#define BOARD_POWER_ON   4   // peripheral power enable — must be HIGH for RS485 transceiver
#define BOARD_485_RX     38
#define BOARD_485_TX     39
#define BOARD_485_EN     42  // RS485 driver enable — LOW = active (transceiver auto-directions)

// #define WIFI_SSID "BT-P2CKZW"
// #define WIFI_PASS "nFm6T3M9knXaTN"
#define WIFI_SSID "Balloon1"
#define WIFI_PASS "Bla123456"

#define BLYNK_TEMPLATE_ID  "TMPL4GQQAzx08"
#define BLYNK_AUTH_TOKEN   "M7q-VgyYY7Lpze1fNh80IlLJj0OCWkC4"

#define FIRMWARE_VERSION "1.3.0"

#define MQTT_BROKER_URI "mqtt://fra1.blynk.cloud:1883"

#define UPLOAD_INTERVAL_MS 1800000
#define MQTT_DEVICE_ID     "device1"
#define MQTT_PUB_ID        "device"
#define MQTT_PASSWORD      BLYNK_AUTH_TOKEN

// Blynk MQTT topics — the broker routes by auth token internally;
// device-side topics do NOT include the token prefix. Device-API topics
// (ping/reboot/redirect/ota/datastream downlink) are defined in blynk_mqtt.c;
// the relay is just datastream "Relay" now, handled generically there.
#define BLYNK_INFO_TOPIC  "info/mcu"
#define BLYNK_BATCH_TOPIC "batch_ds"

#define SENSOR_SLAVE_ID   1
#define RELAY_SLAVE_ID    2
#define RELAY_CONTROL_REG 1
#define RELAY_VAL_ON      256 // 0x0100
#define RELAY_VAL_OFF     512 // 0x0200

// SIM7000 cellular modem — plugs into the T-Vending board's T-PCIe socket.
// Pins per LilyGo's own T-Vending example (examples/Modem_ATDebug.ino).
// Shares BOARD_POWER_ON with the RS485 transceiver; UART_NUM_1 is already
// used for the RS485 Modbus link, so the modem gets UART_NUM_2.
#define MODEM_PWRKEY_PIN 9
#define MODEM_TX_PIN     3  // ESP32 TX -> modem RX
#define MODEM_RX_PIN     46 // ESP32 RX <- modem TX
#define MODEM_DTR_PIN    12 // driven low to keep the modem from auto-sleeping
#define MODEM_UART_NUM   UART_NUM_2

// This card talks at 230400, not the usual SIMCom default of 115200 —
// confirmed via an Arduino baud-scan sketch (T-PCIE_AT_Passthrough) after
// this exact card looked totally dead at 115200 across every prior test
// (this project, two other codebases, two other boards), which had been
// wrongly attributed to a hardware defect. See T-SIM7600E-Blynk-IDF's
// NETWORK_FAILOVER_NOTES.md for the full story.
#define MODEM_BAUD_RATE 230400

// Left blank for now, but blank/auto proved unreliable on both SIMs tested
// on the sibling T-SIM7600E-Blynk-IDF project (intermittent NO CARRIER after
// dialing) — each needed its real APN hardcoded once found via AT+CGNAPN.
// net_manager.c now logs AT+CGDCONT?/AT+CGNAPN on every connection attempt;
// if this SIM shows the same flakiness, check those logs for its real APN
// and set it here explicitly rather than chasing blank/auto further.
#define CELLULAR_APN ""

// How long Wi-Fi must stay down before falling back to cellular. Debounces
// transient Wi-Fi blips so they don't trigger an expensive modem boot +
// network registration cycle (see NETWORK_FAILOVER_NOTES.md's hysteresis note).
#define WIFI_DOWN_FAILOVER_MS 15000
