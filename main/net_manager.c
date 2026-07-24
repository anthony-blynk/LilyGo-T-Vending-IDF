// Wi-Fi + SIM7000 cellular bring-up, structured per NETWORK_FAILOVER_NOTES.md.
//
// Wi-Fi is preferred: its default netif route_prio (100) already beats PPP's
// default (20), so no priority override is needed (Gotcha 1 in the notes).
// Cellular is on-demand: the modem stays powered off until Wi-Fi has been
// down for WIFI_DOWN_FAILOVER_MS (debounced, so a brief Wi-Fi blip doesn't
// trigger an expensive modem boot + network registration cycle), then boots
// and dials in; it's torn down again once Wi-Fi recovers.
//
// on_ready/on_changed both fire from each interface's own GOT_IP/PPP_GOT_IP
// event, never from esp_netif_get_default_netif() — so unlike the
// Ethernet/Wi-Fi case in the notes, there's no default-netif staleness to
// poll for (Gotcha 2 doesn't apply here) and Gotcha 3 (gate on real GOT_IP)
// is satisfied directly.
//
// Cellular bring-up robustness below (registration wait, connect-timeout
// watchdog, CEER, cellular mutex) ported from the T-SIM7600E-Blynk-IDF
// project's net_manager.c after real hardware testing there found this
// exact SIM7000 card — previously assumed defective — was actually fine,
// just needed the right baud rate and these same timing/robustness fixes.
// See that project's NETWORK_FAILOVER_NOTES.md for the full story.
#include "net_manager.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "app_config.h"

static const char *TAG = "net_manager";

#define WIFI_DISCONNECT_LOG_THROTTLE_MS 30000

static net_manager_ready_cb_t s_on_ready;
static net_manager_changed_cb_t s_on_changed;
static volatile bool s_wifi_connected = false;
static bool s_ready_fired = false;
static int64_t s_last_wifi_log_ms = -WIFI_DISCONNECT_LOG_THROTTLE_MS; // log the first disconnect immediately

static esp_netif_t *s_ppp_netif = NULL;
static esp_modem_dce_t *s_dce = NULL;
static bool s_cellular_active = false;
static bool s_cellular_ip_notified = false; // guards against a duplicate PPP GOT_IP re-notifying mid-session
static esp_timer_handle_t s_wifi_down_timer = NULL;
static esp_timer_handle_t s_ppp_debug_quiet_timer = NULL;
static esp_timer_handle_t s_ppp_connect_timeout_timer = NULL;

// Catches the network dropping the data call after dial (e.g. "NO CARRIER"
// mid-PPP-negotiation) — esp_modem_set_mode(DATA) only confirms the modem
// accepted the AT command, not that a PPP session actually comes up, and
// lwIP's PPP task just silently keeps retransmitting LCP forever against a
// dead call with no event of its own. If we don't get an IP within this
// window, force a teardown-and-retry instead of hanging until someone
// notices and resets by hand. 45s comfortably covers the couple-of-seconds-
// to-tens-of-seconds range actually observed for successful connects on the
// sibling project, without indefinitely hanging on a truly dead call either.
#define PPP_CONNECT_TIMEOUT_MS 45000

// Serializes cellular_start()/cellular_stop() — they can be invoked from
// different tasks (the esp_timer service task via wifi_down_timer_cb(), and
// whatever task calls net_manager_reset(), e.g. the MQTT client's own task).
// Without this, two concurrent calls can both install/destroy the modem's
// UART driver at once, which aborts the whole app ("UART driver already
// installed" inside esp_modem's C++ exception path).
static SemaphoreHandle_t s_cellular_mutex;

// Debounces on_changed() specifically — protects against spurious repeat
// GOT_IP events (e.g. a DHCP lease renewal on an already-connected STA,
// or a delayed/queued event surviving a netif teardown) forcing an
// unnecessary MQTT reconnect, without needing to perfectly identify which
// GOT_IP re-fire caused it. Real Wi-Fi/cellular transitions in this system
// are inherently much slower than this window (15s failover debounce plus
// tens of seconds of modem boot), so this doesn't mask genuine failover.
#define NETWORK_CHANGED_DEBOUNCE_MS 60000
static int64_t s_last_changed_notify_ms = -NETWORK_CHANGED_DEBOUNCE_MS;

static void notify_got_ip(void)
{
    if (!s_ready_fired) {
        s_ready_fired = true;
        if (s_on_ready) {
            s_on_ready();
        }
        return;
    }
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms - s_last_changed_notify_ms < NETWORK_CHANGED_DEBOUNCE_MS) {
        ESP_LOGI(TAG, "Network-changed notification suppressed (debounced, %lldms since last)",
                 (long long)(now_ms - s_last_changed_notify_ms));
        return;
    }
    s_last_changed_notify_ms = now_ms;
    if (s_on_changed) {
        s_on_changed();
    }
}

static void cellular_stop_locked(void)
{
    if (!s_cellular_active) {
        return;
    }
    ESP_LOGI(TAG, "Tearing down cellular fallback...");
    esp_timer_stop(s_ppp_debug_quiet_timer); // no-op if it already fired/was never started
    esp_timer_stop(s_ppp_connect_timeout_timer); // no-op if it already fired/was never started
    esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);
    // Extended Error Report — the actual network-side cause for the last
    // call/session ending (e.g. "no network service", "activation rejected"),
    // instead of just the bare "NO CARRIER" we'd see on the wire. Only
    // meaningful right after a session actually ended, but harmless to ask
    // on every teardown (a clean Wi-Fi-recovered teardown just logs nothing
    // interesting here).
    char ceer_buf[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
    if (esp_modem_at(s_dce, "AT+CEER", ceer_buf, 5000) == ESP_OK) {
        ESP_LOGI(TAG, "Extended error report (CEER): %s", ceer_buf);
    }
    esp_modem_power_down(s_dce); // graceful AT-command shutdown (e.g. AT+CPOWD)
    esp_modem_destroy(s_dce);
    s_dce = NULL;
    esp_netif_destroy(s_ppp_netif);
    s_ppp_netif = NULL;
    s_cellular_active = false;
}

// PWRKEY is a toggle, not a level — so it only behaves predictably from a
// known starting state. If the ESP32 resets while cellular was active
// (crash, brownout, watchdog, USB replug) the modem never gets the graceful
// esp_modem_power_down() in cellular_stop() and is left running its old PPP
// session; blindly pulsing PWRKEY next time either does nothing or leaves it
// in a worse state, and plain "AT" sent into a live PPP stream just looks
// like silence/noise. Before touching PWRKEY at all, check via raw UART
// whether the modem is already alive: try AT as-is, and if that's silent,
// assume it might be stuck in data mode and try the PPP escape sequence
// ("+++" with guard silences) to drop it back to command mode, then retry.
static bool modem_already_responsive(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = MODEM_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(MODEM_UART_NUM, 512, 512, 0, NULL, 0) != ESP_OK) {
        return false;
    }
    uart_param_config(MODEM_UART_NUM, &uart_cfg);
    uart_set_pin(MODEM_UART_NUM, MODEM_TX_PIN, MODEM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    bool alive = false;
    for (int attempt = 0; attempt < 2 && !alive; attempt++) {
        if (attempt == 1) {
            ESP_LOGI(TAG, "Modem silent on plain AT, trying PPP escape sequence...");
            vTaskDelay(pdMS_TO_TICKS(1100));
            uart_write_bytes(MODEM_UART_NUM, "+++", 3);
            vTaskDelay(pdMS_TO_TICKS(1100));
        }
        uart_flush_input(MODEM_UART_NUM);
        uart_write_bytes(MODEM_UART_NUM, "AT\r", 3);
        uint8_t buf[64] = {0};
        int len = uart_read_bytes(MODEM_UART_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(500));
        if (len > 0) {
            buf[len] = '\0';
            if (strstr((const char *)buf, "OK") != NULL) {
                alive = true;
            }
        }
    }
    uart_driver_delete(MODEM_UART_NUM);
    return alive;
}

// A modem that answers plain AT is only "alive," not necessarily attached to
// the network — RSSI reflects hearing a cell tower, not having completed NAS
// registration, and dialing before registration finishes is a reliable way
// to get "CONNECT" followed immediately by "NO CARRIER". Poll AT+CEREG?
// until it reports registered (stat 1 or 5) or this gives up and lets the
// dial attempt proceed anyway — the connect-timeout watchdog below still
// catches an eventual failure.
#define REGISTRATION_WAIT_TIMEOUT_MS 30000
static bool wait_for_registration(void)
{
    char resp[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
    int attempts = REGISTRATION_WAIT_TIMEOUT_MS / 1000;
    for (int attempt = 0; attempt < attempts; attempt++) {
        if (esp_modem_at(s_dce, "AT+CEREG?", resp, 5000) == ESP_OK) {
            const char *p = strstr(resp, "+CEREG:");
            p = p ? p + strlen("+CEREG:") : resp;
            int mode = -1, stat = -1;
            if (sscanf(p, " %d,%d", &mode, &stat) == 2 && (stat == 1 || stat == 5)) {
                ESP_LOGI(TAG, "Network registration confirmed (CEREG stat=%d) after %d s", stat, attempt);
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGW(TAG, "Network registration not confirmed within %d s, dialing anyway",
             REGISTRATION_WAIT_TIMEOUT_MS / 1000);
    return false;
}

static void cellular_start_locked(void)
{
    if (s_cellular_active || s_wifi_connected) {
        return;
    }
    ESP_LOGW(TAG, "Wi-Fi down for %d s, starting cellular fallback...", WIFI_DOWN_FAILOVER_MS / 1000);

    gpio_set_level(MODEM_DTR_PIN, 0); // keep the modem from auto-sleeping

    // Full raw AT traffic during the connect attempt (requires
    // CONFIG_ESP_MODEM_ADD_DEBUG_LOGS + CONFIG_LOG_MAXIMUM_LEVEL_DEBUG) —
    // quieted back down once in PPP data mode below, since dumping every
    // encoded IP packet forever isn't useful, just noisy.
    esp_log_level_set("uart-tx", ESP_LOG_DEBUG);
    esp_log_level_set("uart-rx", ESP_LOG_DEBUG);

    if (modem_already_responsive()) {
        ESP_LOGI(TAG, "Modem already on and responsive — skipping PWRKEY pulse");
    } else {
        // PWRKEY boot pulse. LilyGo's T-Vending Modem_ATDebug.ino example uses
        // a 1000ms hold (the SIM7000 datasheet's documented minimum); bumped
        // to 2000ms here for extra margin.
        gpio_set_level(MODEM_PWRKEY_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(MODEM_PWRKEY_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_set_level(MODEM_PWRKEY_PIN, 0);
    }

    esp_netif_config_t ppp_netif_config = ESP_NETIF_DEFAULT_PPP();
    s_ppp_netif = esp_netif_new(&ppp_netif_config);
    if (!s_ppp_netif) {
        ESP_LOGE(TAG, "Failed to create PPP netif");
        return;
    }

    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.baud_rate = MODEM_BAUD_RATE;
    dte_config.uart_config.port_num = MODEM_UART_NUM;
    dte_config.uart_config.tx_io_num = MODEM_TX_PIN;
    dte_config.uart_config.rx_io_num = MODEM_RX_PIN;
    dte_config.uart_config.rts_io_num = -1;
    dte_config.uart_config.cts_io_num = -1;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;

    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CELLULAR_APN);

    s_dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7000, &dte_config, &dce_config, s_ppp_netif);
    if (!s_dce) {
        ESP_LOGE(TAG, "Failed to create modem DCE");
        esp_netif_destroy(s_ppp_netif);
        s_ppp_netif = NULL;
        return;
    }

    // Give the modem time to boot after the PWRKEY pulse before it responds to AT.
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 20; attempt++) {
        err = esp_modem_sync(s_dce);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "esp_modem_sync attempt %d/20 failed: %s", attempt + 1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Modem did not respond after boot, aborting cellular fallback");
        esp_modem_destroy(s_dce);
        s_dce = NULL;
        esp_netif_destroy(s_ppp_netif);
        s_ppp_netif = NULL;
        return;
    }

    int rssi = 0, ber = 0;
    if (esp_modem_get_signal_quality(s_dce, &rssi, &ber) == ESP_OK) {
        ESP_LOGI(TAG, "Cellular signal: rssi=%d ber=%d", rssi, ber);
    }

    // Modem/SIM identity — only queryable in COMMAND mode, so this has to
    // happen before switching to DATA below.
    char info_buf[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
    // Verbose registration reporting — any registration state change from
    // here on shows up as a "+CEREG: ..." URC in the raw uart-rx debug dump,
    // instead of us only ever seeing point-in-time AT+COPS? snapshots.
    esp_modem_at(s_dce, "AT+CEREG=2", info_buf, 5000);
    wait_for_registration();
    if (esp_modem_get_imsi(s_dce, info_buf) == ESP_OK) {
        ESP_LOGI(TAG, "SIM IMSI: %s", info_buf);
    }
    if (esp_modem_at(s_dce, "AT+CCID", info_buf, 5000) == ESP_OK) {
        ESP_LOGI(TAG, "SIM ICCID: %s", info_buf);
    }
    int act = 0;
    if (esp_modem_get_operator_name(s_dce, info_buf, &act) == ESP_OK) {
        ESP_LOGI(TAG, "Network operator: %s (act=%d)", info_buf, act);
    }
    ESP_LOGI(TAG, "APN: %s", CELLULAR_APN[0] ? CELLULAR_APN : "(blank/auto)");

    // Diagnostic only: log whatever context profile is already provisioned
    // (which our own CGDCONT below may be about to overwrite) and ask the
    // network directly for its suggested default APN, in case that saves
    // hunting for it manually if blank/auto ever proves unreliable here too
    // (as it did on both SIMs tested on the sibling T-SIM7600E-Blynk-IDF project).
    if (esp_modem_at(s_dce, "AT+CGDCONT?", info_buf, 5000) == ESP_OK) {
        ESP_LOGI(TAG, "Existing PDP context profile(s): %s", info_buf);
    }
    if (esp_modem_at(s_dce, "AT+CGNAPN", info_buf, 5000) == ESP_OK) {
        ESP_LOGI(TAG, "Network-suggested APN (CGNAPN): %s", info_buf);
    }

    err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DATA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_set_mode(DATA) failed: %s", esp_err_to_name(err));
        esp_modem_destroy(s_dce);
        s_dce = NULL;
        esp_netif_destroy(s_ppp_netif);
        s_ppp_netif = NULL;
        return;
    }
    s_cellular_active = true;
    s_cellular_ip_notified = false;
    // Keep the AT/PPP dump alive for a short window after entering DATA mode
    // so a stalled LCP/IPCP handshake would actually be visible — then quiet
    // it down once the session should be established, since dumping every
    // encoded IP packet forever isn't useful, just noisy.
    esp_timer_start_once(s_ppp_debug_quiet_timer, 5000 * 1000);
    // Cancelled from IP_EVENT_PPP_GOT_IP below once the session actually has
    // an address; fires ppp_connect_timeout_cb() otherwise.
    esp_timer_start_once(s_ppp_connect_timeout_timer, (int64_t)PPP_CONNECT_TIMEOUT_MS * 1000);
    // notify_got_ip() fires from IP_EVENT_PPP_GOT_IP once the PPP session
    // actually has an address.
}

static void cellular_start(void)
{
    xSemaphoreTake(s_cellular_mutex, portMAX_DELAY);
    cellular_start_locked();
    xSemaphoreGive(s_cellular_mutex);
}

static void cellular_stop(void)
{
    xSemaphoreTake(s_cellular_mutex, portMAX_DELAY);
    cellular_stop_locked();
    xSemaphoreGive(s_cellular_mutex);
}

static void wifi_down_timer_cb(void *arg)
{
    if (!s_wifi_connected) {
        cellular_start();
    }
}

static void ppp_debug_quiet_timer_cb(void *arg)
{
    esp_log_level_set("uart-tx", ESP_LOG_WARN);
    esp_log_level_set("uart-rx", ESP_LOG_WARN);
}

static void ppp_connect_timeout_cb(void *arg)
{
    if (s_cellular_ip_notified) {
        return; // connected fine, this fire is stale (shouldn't normally happen — stopped on GOT_IP)
    }
    ESP_LOGE(TAG, "No cellular IP within %d s of dialing (modem likely dropped the call, "
                  "e.g. NO CARRIER) — resetting and retrying...",
             PPP_CONNECT_TIMEOUT_MS / 1000);
    cellular_stop();
    cellular_start();
}

static void net_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        s_wifi_connected = false;
        // Throttled — retry attempts against an AP that's genuinely down can
        // repeat every couple of seconds indefinitely, which floods the log.
        // esp_wifi_connect() below still runs every time regardless.
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - s_last_wifi_log_ms >= WIFI_DISCONNECT_LOG_THROTTLE_MS) {
            s_last_wifi_log_ms = now_ms;
            ESP_LOGW(TAG, "Wi-Fi disconnected from \"%s\" (reason=%d), reconnecting...", WIFI_SSID, event->reason);
        }
        esp_wifi_connect();
        // Debounce off the FIRST disconnect only — retry attempts that keep
        // failing every couple of seconds must not keep pushing this back,
        // or it would never reach WIFI_DOWN_FAILOVER_MS and cellular would
        // never take over.
        if (!esp_timer_is_active(s_wifi_down_timer)) {
            esp_timer_start_once(s_wifi_down_timer, (int64_t)WIFI_DOWN_FAILOVER_MS * 1000);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // GOT_IP can re-fire on an already-connected STA (e.g. a routine DHCP
        // lease renewal from the AP) — only treat it as a real
        // disconnected->connected transition, not every occurrence, or a
        // harmless renewal ends up forcing an unnecessary MQTT reconnect.
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        bool was_connected = s_wifi_connected;
        ESP_LOGI(TAG, "Wi-Fi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        esp_timer_stop(s_wifi_down_timer);
        if (!was_connected) {
            cellular_stop();
            notify_got_ip();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_PPP_GOT_IP) {
        // Same guard as above, in case PPP ever re-fires GOT_IP within an
        // already-established session.
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Cellular connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        esp_timer_stop(s_ppp_connect_timeout_timer); // no-op if it already fired/was never started
        if (!s_cellular_ip_notified) {
            s_cellular_ip_notified = true;
            notify_got_ip();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGW(TAG, "Cellular lost IP");
    }
}

void net_manager_init(net_manager_ready_cb_t on_ready, net_manager_changed_cb_t on_changed)
{
    s_on_ready = on_ready;
    s_on_changed = on_changed;

    s_cellular_mutex = xSemaphoreCreateMutex();

    gpio_config_t modem_ctrl_conf = {
        .pin_bit_mask = (1ULL << MODEM_PWRKEY_PIN) | (1ULL << MODEM_DTR_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&modem_ctrl_conf));
    gpio_set_level(MODEM_PWRKEY_PIN, 0);
    gpio_set_level(MODEM_DTR_PIN, 0);

    const esp_timer_create_args_t timer_args = {
        .callback = &wifi_down_timer_cb,
        .name = "wifi_down",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_wifi_down_timer));

    const esp_timer_create_args_t ppp_quiet_timer_args = {
        .callback = &ppp_debug_quiet_timer_cb,
        .name = "ppp_debug_quiet",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ppp_quiet_timer_args, &s_ppp_debug_quiet_timer));

    const esp_timer_create_args_t ppp_connect_timeout_timer_args = {
        .callback = &ppp_connect_timeout_cb,
        .name = "ppp_connect_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ppp_connect_timeout_timer_args, &s_ppp_connect_timeout_timer));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &net_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &net_event_handler, NULL, NULL));

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
    return s_wifi_connected || s_cellular_active;
}

void net_manager_reset(void)
{
    // Non-blocking: if a cellular connect/teardown is already in flight
    // (e.g. still inside wait_for_registration()'s poll loop, before
    // s_cellular_active gets set), reading s_cellular_active here would race
    // it — seen in practice reading stale "false" and taking the Wi-Fi-reset
    // branch mid-cellular-connect. Just skip; the in-progress attempt will
    // resolve on its own (success, or its own connect-timeout watchdog).
    if (xSemaphoreTake(s_cellular_mutex, 0) != pdTRUE) {
        ESP_LOGI(TAG, "Cellular busy (connect/teardown in progress), skipping reset request");
        return;
    }
    bool was_cellular_active = s_cellular_active;
    xSemaphoreGive(s_cellular_mutex);

    if (was_cellular_active) {
        ESP_LOGW(TAG, "Forcing cellular reset...");
        cellular_stop();
        cellular_start();
    } else {
        ESP_LOGW(TAG, "Forcing Wi-Fi reset...");
        esp_wifi_disconnect();
        s_wifi_connected = false;
        esp_wifi_connect();
    }
}
