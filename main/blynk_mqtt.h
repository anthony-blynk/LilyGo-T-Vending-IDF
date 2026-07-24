#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Call once, early in app_main() — before network bring-up. If the running
// image is an unconfirmed OTA update (bootloader state PENDING_VERIFY),
// arms a watchdog that force-rolls-back to the previous image if MQTT
// hasn't connected within the timeout. Harmless no-op on a normal
// (non-OTA) boot. See blynk_mqtt.c's ota_watchdog_cb() for why this is
// needed in addition to esp_ota_mark_app_valid_cancel_rollback() — ESP-IDF's
// automatic rollback only triggers on a crash/reset, not on an update that
// just never manages to connect.
void blynk_mqtt_arm_ota_watchdog(void);

// Starts the Blynk MQTT client. Call once network is up — signature matches
// net_manager_ready_cb_t so it can be passed directly to net_manager_init().
void blynk_mqtt_start(void);

// Force-disconnects an already-running client so auto-reconnect picks up a
// fresh session — signature matches net_manager_changed_cb_t.
void blynk_mqtt_reconnect(void);

bool blynk_mqtt_is_connected(void);

// Publishes a raw payload to an arbitrary topic (e.g. app-specific
// telemetry), QoS 0, no retain. Returns ESP_ERR_INVALID_STATE if the client
// hasn't been started yet.
esp_err_t blynk_mqtt_publish(const char *topic, const char *payload);

// Called for every "downlink/ds/<name>" datastream write, with the topic's
// trailing name (e.g. "Relay") and the raw payload string. Optional — the
// module logs and drops datastream downlinks if no handler is registered.
// Register before net_manager_init() calls blynk_mqtt_start().
typedef void (*blynk_mqtt_ds_handler_t)(const char *ds_name, const char *payload);
void blynk_mqtt_set_ds_handler(blynk_mqtt_ds_handler_t handler);

// Optional notification hooks, both pure notifications (the module works
// fine with neither set): called right after a successful MQTT connect, and
// right before the module forces a network reset after 3 consecutive MQTT
// failures — useful for app-level diagnostic counters.
typedef void (*blynk_mqtt_event_cb_t)(void);
void blynk_mqtt_set_on_connected(blynk_mqtt_event_cb_t cb);
void blynk_mqtt_set_on_network_reset(blynk_mqtt_event_cb_t cb);
