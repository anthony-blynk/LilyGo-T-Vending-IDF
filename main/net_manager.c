// Wi-Fi bring-up, structured per NETWORK_FAILOVER_NOTES.md so a second
// interface (SIM7000 cellular) can be added later without reshaping the app's
// on_ready/on_changed contract. Only Wi-Fi exists today, so the multi-interface
// arbitration (route_prio, default-netif polling backstop — Gotchas 1-2 in
// those notes) isn't implemented yet; it becomes necessary once a second
// netif exists and needs verifying against this project's actual esp_modem
// behavior rather than assumed.
#include "net_manager.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "app_config.h"

static const char *TAG = "net_manager";

static net_manager_ready_cb_t s_on_ready;
static net_manager_changed_cb_t s_on_changed;
static volatile bool s_connected = false;
static bool s_ready_fired = false;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
        if (!s_ready_fired) {
            s_ready_fired = true;
            if (s_on_ready) {
                s_on_ready();
            }
        } else if (s_on_changed) {
            s_on_changed();
        }
    }
}

void net_manager_init(net_manager_ready_cb_t on_ready, net_manager_changed_cb_t on_changed)
{
    s_on_ready = on_ready;
    s_on_changed = on_changed;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

bool net_manager_is_connected(void)
{
    return s_connected;
}

void net_manager_reset(void)
{
    ESP_LOGW(TAG, "Forcing Wi-Fi reset...");
    esp_wifi_disconnect();
    s_connected = false;
    esp_wifi_connect();
}
