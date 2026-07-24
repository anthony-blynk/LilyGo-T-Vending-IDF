// Blynk MQTT Device API — info/mcu, ping, reboot, redirect, OTA, and
// datastream downlink. Ported from the T-SIM7600E-Blynk-IDF project's
// blynk_mqtt.c/.h (itself ported from C:\Blynk\blynk_p4_mqtt — see that
// project's NETWORK_FAILOVER_NOTES.md Gotcha 4 for why
// esp_mqtt_client_disconnect() is used below instead of
// esp_mqtt_client_reconnect()), with two additions neither source project
// had:
//
//   - App rollback safety: the running image is only marked "valid" after
//     MQTT actually reconnects post-OTA (confirm_rollback_if_pending()), so
//     a bad update automatically reverts to the previous slot on next boot
//     instead of bricking the device. Requires CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
//     and the two-OTA-slot partition table in partitions.csv.
//   - Basic OTA status reporting back over MQTT (start/success/failure).
//
// Also adds, on top of the T-SIM7600E version, two small hooks this
// project's main.c actually needs: a datastream-downlink handler (so
// "downlink/ds/Relay" can drive the real RS485 relay instead of just being
// logged) and connect/network-reset notification callbacks (so main.c can
// keep its Reconnects/WifiResets telemetry counters).
//
// Kept as a real, documented constraint rather than an oversight:
// MQTT_EVENT_DATA only handles payloads that arrive in a single fragment
// and fit the local buffer — larger/fragmented control messages are logged
// and dropped.
#include "blynk_mqtt.h"

#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "app_config.h"
#include "net_manager.h"

static const char *TAG = "blynk_mqtt";

#define BLYNK_TOPIC_DS_DOWNLINK "downlink/ds/#"
#define BLYNK_TOPIC_DS_PREFIX   "downlink/ds/"
#define BLYNK_TOPIC_PING        "downlink/ping"
#define BLYNK_TOPIC_REBOOT      "downlink/reboot"
#define BLYNK_TOPIC_REDIRECT    "downlink/redirect"
#define BLYNK_TOPIC_OTA         "downlink/ota/json"
#define BLYNK_TOPIC_OTA_STATUS  "ota/status"

#define MAX_CONTROL_PAYLOAD 512

static esp_mqtt_client_handle_t s_client;
static volatile bool s_connected = false;
static bool s_rollback_confirmed = false;
static uint32_t s_fail_count = 0;
static blynk_mqtt_ds_handler_t s_ds_handler;
static blynk_mqtt_event_cb_t s_on_connected;
static blynk_mqtt_event_cb_t s_on_network_reset;

// Cooldown on the network-reset escalation below, independent of how fast
// MQTT itself is retrying — a genuinely-down network can trip 3 consecutive
// failures in seconds, and forcing a reset that often just interrupts
// Wi-Fi/cellular's own in-progress reconnection instead of helping it.
#define NETWORK_RESET_COOLDOWN_MS 60000
static int64_t s_last_network_reset_ms = -NETWORK_RESET_COOLDOWN_MS;

static const struct {
    const char *topic;
    int qos;
} s_control_subs[] = {
    {BLYNK_TOPIC_DS_DOWNLINK, 1},
    {BLYNK_TOPIC_PING, 1}, // Blynk always publishes this at QoS 1
    {BLYNK_TOPIC_REBOOT, 1},
    {BLYNK_TOPIC_REDIRECT, 1},
    {BLYNK_TOPIC_OTA, 1},
};

#define BLYNK_PARAM_KV(k, v) k "\0" v "\0"

// Blynk.Cloud scans this tag out of the flashed binary itself (not over
// MQTT) to catalog the firmware version for its OTA-eligibility UI.
// -ffunction-sections/-fdata-sections put it in its own section, and this
// toolchain's default --gc-sections link can drop it despite
// __attribute__((used)) if nothing genuinely references it — logging its
// address in blynk_mqtt_start() is a real reference that keeps it.
volatile const char firmwareTag[] __attribute__((used)) = "blnkinf\0"
    BLYNK_PARAM_KV("mcu", FIRMWARE_VERSION)
    BLYNK_PARAM_KV("fw-type", BLYNK_TEMPLATE_ID)
    BLYNK_PARAM_KV("build", __DATE__ " " __TIME__)
    BLYNK_PARAM_KV("blynk", "0.1.0")
    BLYNK_PARAM_KV("hw", "T-Vending")
    "\0";

bool blynk_mqtt_is_connected(void)
{
    return s_connected;
}

esp_err_t blynk_mqtt_publish(const char *topic, const char *payload)
{
    if (!s_client) {
        return ESP_ERR_INVALID_STATE;
    }
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, 0, 0);
    return msg_id >= 0 ? ESP_OK : ESP_FAIL;
}

void blynk_mqtt_set_ds_handler(blynk_mqtt_ds_handler_t handler)
{
    s_ds_handler = handler;
}

void blynk_mqtt_set_on_connected(blynk_mqtt_event_cb_t cb)
{
    s_on_connected = cb;
}

void blynk_mqtt_set_on_network_reset(blynk_mqtt_event_cb_t cb)
{
    s_on_network_reset = cb;
}

static void publish_info(void)
{
    char info[192];
    snprintf(info, sizeof(info),
             "{\"tmpl\":\"%s\",\"ver\":\"%s\",\"build\":\"%s\",\"type\":\"%s\",\"rxbuff\":1024}",
             BLYNK_TEMPLATE_ID, FIRMWARE_VERSION, __DATE__ " " __TIME__, BLYNK_TEMPLATE_ID);
    blynk_mqtt_publish(BLYNK_INFO_TOPIC, info);
}

// How long a freshly-OTA'd image gets to prove itself (MQTT connected)
// before blynk_mqtt_arm_ota_watchdog()'s timer force-rolls-back. Generous
// enough to cover a worst-case cellular fallback boot (~15s Wi-Fi-down
// debounce + tens of seconds of modem registration) with real margin.
#define OTA_CONFIRM_TIMEOUT_MS 120000

static esp_timer_handle_t s_ota_watchdog_timer;

// Confirms the currently-running app image is good, canceling the
// bootloader's automatic rollback-on-next-reset. Only meaningful right after
// an OTA reboot (esp_ota_get_state_partition() reports PENDING_VERIFY);
// harmless no-op otherwise. Called once, on the first successful MQTT
// connect after boot — reaching that point means Wi-Fi/cellular *and* MQTT
// are both working, a reasonable bar for "this update is good."
static void confirm_rollback_if_pending(void)
{
    if (s_rollback_confirmed) {
        return;
    }
    s_rollback_confirmed = true;

    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "Post-OTA connectivity confirmed, marking app valid");
        esp_ota_mark_app_valid_cancel_rollback();
    }
    if (s_ota_watchdog_timer) {
        esp_timer_stop(s_ota_watchdog_timer); // no-op if it already fired/was never started
    }
}

// ESP-IDF's automatic app-rollback only triggers on a crash/reset while
// still PENDING_VERIFY — a bad update that boots fine but can never reach
// MQTT (wrong broker/auth, or a network-stack bug) doesn't crash, so it
// would otherwise sit in PENDING_VERIFY forever and never actually revert.
// This timer is the explicit "or roll back anyway" half of that safety net.
static void ota_watchdog_cb(void *arg)
{
    if (s_rollback_confirmed) {
        return; // confirm_rollback_if_pending() already handled it
    }
    ESP_LOGE(TAG, "OTA not confirmed within %d s, rolling back to the previous image...",
              OTA_CONFIRM_TIMEOUT_MS / 1000);
    esp_ota_mark_app_invalid_rollback_and_reboot(); // does not return on success
    ESP_LOGE(TAG, "Rollback call returned — no valid previous image to revert to?");
}

void blynk_mqtt_arm_ota_watchdog(void)
{
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &state) != ESP_OK || state != ESP_OTA_IMG_PENDING_VERIFY) {
        return; // normal boot, not an unconfirmed OTA image — nothing to arm
    }
    ESP_LOGW(TAG, "Booted a pending-verify OTA image — arming %d s confirm-or-rollback watchdog",
             OTA_CONFIRM_TIMEOUT_MS / 1000);
    const esp_timer_create_args_t timer_args = {
        .callback = &ota_watchdog_cb,
        .name = "ota_watchdog",
    };
    esp_timer_create(&timer_args, &s_ota_watchdog_timer);
    esp_timer_start_once(s_ota_watchdog_timer, (int64_t)OTA_CONFIRM_TIMEOUT_MS * 1000);
}

static void ota_task(void *pvParameter)
{
    char *url = pvParameter;
    ESP_LOGI(TAG, "Starting OTA update from %s", url);
    blynk_mqtt_publish(BLYNK_TOPIC_OTA_STATUS, "{\"status\":\"starting\"}");

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_err_t err = esp_https_ota(&ota_cfg);
    free(url);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA succeeded, rebooting");
        blynk_mqtt_publish(BLYNK_TOPIC_OTA_STATUS, "{\"status\":\"success\"}");
        vTaskDelay(pdMS_TO_TICKS(500)); // let the publish flush before reset
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
        char status[96];
        snprintf(status, sizeof(status), "{\"status\":\"failed\",\"error\":\"%s\"}", esp_err_to_name(err));
        blynk_mqtt_publish(BLYNK_TOPIC_OTA_STATUS, status);
    }
    vTaskDelete(NULL);
}

static void handle_ota_request(const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    if (!root) {
        ESP_LOGW(TAG, "OTA payload is not valid JSON, ignoring: %s", payload);
        return;
    }
    const cJSON *url_item = cJSON_GetObjectItemCaseSensitive(root, "url");
    if (!cJSON_IsString(url_item) || url_item->valuestring[0] == '\0') {
        ESP_LOGW(TAG, "OTA payload missing 'url', ignoring: %s", payload);
        cJSON_Delete(root);
        return;
    }
    char *url_copy = strdup(url_item->valuestring);
    cJSON_Delete(root);
    if (!url_copy) {
        ESP_LOGE(TAG, "OTA: out of memory");
        return;
    }
    if (xTaskCreate(ota_task, "blynk_ota", 8192, url_copy, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OTA task");
        free(url_copy);
    }
}

static void handle_redirect(const char *payload)
{
    char new_uri[160];
    if (strstr(payload, "://")) {
        strlcpy(new_uri, payload, sizeof(new_uri));
    } else {
        cJSON *root = cJSON_Parse(payload);
        if (!root) {
            ESP_LOGW(TAG, "Redirect payload is not valid JSON or a URI, ignoring: %s", payload);
            return;
        }
        const cJSON *host_item = cJSON_GetObjectItemCaseSensitive(root, "host");
        if (!cJSON_IsString(host_item) || host_item->valuestring[0] == '\0') {
            ESP_LOGW(TAG, "Redirect payload missing 'host', ignoring: %s", payload);
            cJSON_Delete(root);
            return;
        }
        const cJSON *port_item = cJSON_GetObjectItemCaseSensitive(root, "port");
        int port = cJSON_IsNumber(port_item) ? port_item->valueint : 8883;
        snprintf(new_uri, sizeof(new_uri), "mqtts://%s:%d", host_item->valuestring, port);
        cJSON_Delete(root);
    }
    ESP_LOGW(TAG, "Server redirect to %s", new_uri);
    if (esp_mqtt_client_set_uri(s_client, new_uri) == ESP_OK) {
        // Per NETWORK_FAILOVER_NOTES.md Gotcha 4: esp_mqtt_client_reconnect()
        // only does anything if the client is already WAIT_RECONNECT;
        // disconnect() is what actually forces a CONNECTED client to drop
        // and pick up the new URI.
        esp_mqtt_client_disconnect(s_client);
    } else {
        ESP_LOGE(TAG, "Invalid redirect URI: %s", new_uri);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_connected = true;
        s_fail_count = 0;
        publish_info();
        for (size_t i = 0; i < sizeof(s_control_subs) / sizeof(s_control_subs[0]); i++) {
            esp_mqtt_client_subscribe(s_client, s_control_subs[i].topic, s_control_subs[i].qos);
        }
        confirm_rollback_if_pending();
        if (s_on_connected) {
            s_on_connected();
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "MQTT disconnected");
        // Escalate to a full network reset after repeated failures.
        // Cooldown-gated: a genuinely-down network can trip this every few
        // seconds, and resetting that often just fights Wi-Fi/cellular's own
        // recovery instead of helping it (see NETWORK_RESET_COOLDOWN_MS).
        if (++s_fail_count >= 3) {
            s_fail_count = 0;
            int64_t now_ms = esp_timer_get_time() / 1000;
            if (now_ms - s_last_network_reset_ms >= NETWORK_RESET_COOLDOWN_MS) {
                s_last_network_reset_ms = now_ms;
                ESP_LOGW(TAG, "3 consecutive MQTT failures, forcing a network reset...");
                if (s_on_network_reset) {
                    s_on_network_reset();
                }
                net_manager_reset();
            } else {
                ESP_LOGI(TAG, "3 consecutive MQTT failures, but network reset is on cooldown");
            }
        }
        break;

    case MQTT_EVENT_DATA: {
        // Only handle payloads that arrived in a single fragment and fit —
        // a real, documented constraint carried over from the source
        // project, not an oversight (see this file's header comment).
        if (event->current_data_offset != 0 || event->data_len != event->total_data_len ||
            event->data_len >= MAX_CONTROL_PAYLOAD) {
            ESP_LOGW(TAG, "Ignoring oversized/fragmented MQTT message on topic %.*s",
                     event->topic_len, event->topic);
            break;
        }
        char payload[MAX_CONTROL_PAYLOAD];
        memcpy(payload, event->data, event->data_len);
        payload[event->data_len] = '\0';

        if (event->topic_len == strlen(BLYNK_TOPIC_PING) &&
            strncmp(event->topic, BLYNK_TOPIC_PING, event->topic_len) == 0) {
            ESP_LOGI(TAG, "Received server ping"); // QoS 1 PUBACK is the implicit reply; no app-level pong needed
        } else if (event->topic_len == strlen(BLYNK_TOPIC_REBOOT) &&
                   strncmp(event->topic, BLYNK_TOPIC_REBOOT, event->topic_len) == 0) {
            ESP_LOGW(TAG, "Reboot requested by server, restarting...");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        } else if (event->topic_len == strlen(BLYNK_TOPIC_REDIRECT) &&
                   strncmp(event->topic, BLYNK_TOPIC_REDIRECT, event->topic_len) == 0) {
            handle_redirect(payload);
        } else if (event->topic_len == strlen(BLYNK_TOPIC_OTA) &&
                   strncmp(event->topic, BLYNK_TOPIC_OTA, event->topic_len) == 0) {
            handle_ota_request(payload);
        } else if (event->topic_len > (int)strlen(BLYNK_TOPIC_DS_PREFIX) &&
                   strncmp(event->topic, BLYNK_TOPIC_DS_PREFIX, strlen(BLYNK_TOPIC_DS_PREFIX)) == 0) {
            int name_len = event->topic_len - (int)strlen(BLYNK_TOPIC_DS_PREFIX);
            const char *name_ptr = event->topic + strlen(BLYNK_TOPIC_DS_PREFIX);
            ESP_LOGI(TAG, "Datastream downlink [%.*s] = %s", name_len, name_ptr, payload);
            if (s_ds_handler) {
                char ds_name[64];
                int copy_len = name_len < (int)sizeof(ds_name) - 1 ? name_len : (int)sizeof(ds_name) - 1;
                memcpy(ds_name, name_ptr, copy_len);
                ds_name[copy_len] = '\0';
                s_ds_handler(ds_name, payload);
            }
        }
        break;
    }

    default:
        break;
    }
}

void blynk_mqtt_start(void)
{
    ESP_LOGI(TAG, "firmware tag @ %p (%d bytes)", (const void *)firmwareTag, sizeof(firmwareTag));

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = MQTT_DEVICE_ID,
        .credentials.username = MQTT_PUB_ID,
        .credentials.authentication.password = MQTT_PASSWORD,
        .session.keepalive = 60,
        .network.timeout_ms = 15000,
        // Deliberately left at esp-mqtt's default (~10s) reconnect backoff —
        // see the T-SIM7600E-Blynk-IDF NETWORK_FAILOVER_NOTES.md for why a
        // tighter timeout (1000ms) caused a real crash there (concurrent
        // cellular_start() calls under net_manager.c's mutex-less version at
        // the time). This project's net_manager.c now has the same mutex
        // fix, but there's no reason to reintroduce the aggressive timeout.
    };
    s_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
}

void blynk_mqtt_reconnect(void)
{
    if (!s_client) {
        return;
    }
    ESP_LOGI(TAG, "Active network interface changed, forcing MQTT reconnect");
    esp_mqtt_client_disconnect(s_client);
}
