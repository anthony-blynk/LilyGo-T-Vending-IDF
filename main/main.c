/*
 *******************************************************************************
 * T-Vending — Modbus to MQTT to Blynk over WiFi/cellular
 *
 * Blynk's MQTT Device API (info/mcu, ping, reboot, redirect, OTA, datastream
 * downlink) lives in blynk_mqtt.c/.h, ported from the T-SIM7600E-Blynk-IDF
 * project's module of the same shape (itself ported from blynk_p4_mqtt).
 * The relay is driven via the generic "Relay" datastream, through
 * blynk_mqtt_set_ds_handler() below, instead of a hardcoded relay-only
 * MQTT topic subscription.
 *******************************************************************************
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mbcontroller.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "blynk_mqtt.h"
#include "net_manager.h"

#define FIRMWARE_BUILD_DATE __DATE__ " " __TIME__

// Standard Modbus function codes (fixed by the Modbus spec). esp-modbus only
// exposes these to its own internal transports (modbus/mb_objects/include is a
// private include dir of the espressif/esp-modbus component), not to app code,
// so they're restated here for use with mbc_master_send_request().
#define MB_FUNC_READ_INPUT_REGISTER 0x04
#define MB_FUNC_WRITE_REGISTER      0x06

static const char *TAG = "t_vending";

static void *s_mb_master_handle;

static uint32_t s_reconnect_count = 0;
static uint32_t s_wifi_reset_count = 0;
static bool s_relay_state = false;

static bool set_modbus_relay_state(bool turn_on)
{
    uint16_t value = turn_on ? RELAY_VAL_ON : RELAY_VAL_OFF;
    mb_param_request_t req = {
        .slave_addr = RELAY_SLAVE_ID,
        .command = MB_FUNC_WRITE_REGISTER,
        .reg_start = RELAY_CONTROL_REG,
        .reg_size = 1,
    };
    ESP_LOGI(TAG, "Relay %s -> Slave %d", turn_on ? "ON" : "OFF", RELAY_SLAVE_ID);
    esp_err_t err = mbc_master_send_request(s_mb_master_handle, &req, &value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Relay OK");
        return true;
    }
    ESP_LOGW(TAG, "Relay error: %s", esp_err_to_name(err));
    return false;
}

static void poll_sensor_and_publish(void)
{
    uint16_t regs[2];
    mb_param_request_t req = {
        .slave_addr = SENSOR_SLAVE_ID,
        .command = MB_FUNC_READ_INPUT_REGISTER,
        .reg_start = 0x0001,
        .reg_size = 2,
    };
    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Polling XY-MD02...");
    for (int attempt = 0; attempt < 3; attempt++) {
        err = mbc_master_send_request(s_mb_master_handle, &req, regs);
        // Some XY-MD02 units start temp/humidity at register 0x0002 instead of
        // 0x0001. esp-modbus maps an illegal-data-address slave exception to
        // ESP_ERR_NOT_SUPPORTED, so use that as the signal to retry at the
        // other base address — matching the original ModbusMaster-based
        // firmware's fallback (it checked for raw exception code 0x02).
        if (err == ESP_ERR_NOT_SUPPORTED) {
            req.reg_start = 0x0002;
            err = mbc_master_send_request(s_mb_master_handle, &req, regs);
        }
        if (err == ESP_OK) {
            break;
        }
        req.reg_start = 0x0001;
        if (attempt < 2) {
            ESP_LOGW(TAG, "Modbus retry %d: %s", attempt + 1, esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    if (err == ESP_OK) {
        float temperature = (int16_t)regs[0] / 10.0f;
        float humidity = (int16_t)regs[1] / 10.0f;
        ESP_LOGI(TAG, "--> Temp: %.1fC  Hum: %.1f%%", temperature, humidity);

        char payload[200];
        snprintf(payload, sizeof(payload),
                 "{\"Temperature\":%.1f,\"Humidity\":%.1f,\"Reconnects\":%lu,\"WifiResets\":%lu}",
                 temperature, humidity, (unsigned long)s_reconnect_count, (unsigned long)s_wifi_reset_count);
        blynk_mqtt_publish(BLYNK_BATCH_TOPIC, payload);
    } else {
        ESP_LOGW(TAG, "Modbus error: %s", esp_err_to_name(err));
    }
}

// Drives the real RS485 relay off the generic "Relay" datastream downlink —
// replaces the old hardcoded BLYNK_RELAY_TOPIC subscription now that
// blynk_mqtt.c handles all datastream downlinks generically.
static void on_datastream(const char *ds_name, const char *payload)
{
    if (strcmp(ds_name, "Relay") == 0) {
        bool on = (strcmp(payload, "true") == 0 || strcmp(payload, "1") == 0);
        s_relay_state = on;
        set_modbus_relay_state(on);
    }
}

static void on_mqtt_connected(void)
{
    s_reconnect_count++;
}

static void on_mqtt_network_reset(void)
{
    s_wifi_reset_count++;
}

static esp_err_t modbus_master_init(void)
{
    mb_communication_info_t comm = {
        .ser_opts.port = UART_NUM_1,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = 9600,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1,
        .ser_opts.uid = 0,
        .ser_opts.response_tout_ms = 500,
    };

    ESP_LOGI(TAG, "Initializing UART for Modbus...");
    esp_err_t err = mbc_master_create_serial(&comm, &s_mb_master_handle);
    if (err != ESP_OK || s_mb_master_handle == NULL) {
        ESP_LOGE(TAG, "mbc_master_create_serial failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, BOARD_485_TX, BOARD_485_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // esp-modbus requires a non-empty descriptor table before mbc_master_start(),
    // but this app talks to both slaves via raw mbc_master_send_request() calls
    // (to reproduce the address-fallback quirk in poll_sensor_and_publish()), so
    // this entry is only a placeholder to satisfy that requirement.
    static const mb_parameter_descriptor_t placeholder_descriptor[] = {
        {0, "RelayControl", "", RELAY_SLAVE_ID, MB_PARAM_HOLDING,
         RELAY_CONTROL_REG, 1, 0, PARAM_TYPE_U16, 2, {{0, 0, 0}}, PAR_PERMS_READ_WRITE},
    };
    ESP_ERROR_CHECK(mbc_master_set_descriptor(
        s_mb_master_handle, placeholder_descriptor,
        sizeof(placeholder_descriptor) / sizeof(placeholder_descriptor[0])));

    ESP_ERROR_CHECK(mbc_master_start(s_mb_master_handle));
    ESP_LOGI(TAG, "Modbus ready on RS485.");
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, ">>T-Vending  FW:%s  Built:%s", FIRMWARE_VERSION, FIRMWARE_BUILD_DATE);

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    // Before any network bring-up: if this is an unconfirmed OTA image,
    // arm the confirm-or-rollback watchdog so a bad update that never
    // reaches MQTT reverts on its own instead of hanging forever.
    blynk_mqtt_arm_ota_watchdog();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpio_config_t power_conf = {
        .pin_bit_mask = (1ULL << BOARD_POWER_ON) | (1ULL << BOARD_485_EN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&power_conf));
    gpio_set_level(BOARD_POWER_ON, 1); // power up RS485 transceiver
    gpio_set_level(BOARD_485_EN, 0);   // transceiver auto-directions, keep enabled

    blynk_mqtt_set_ds_handler(on_datastream);
    blynk_mqtt_set_on_connected(on_mqtt_connected);
    blynk_mqtt_set_on_network_reset(on_mqtt_network_reset);

    ESP_ERROR_CHECK(modbus_master_init());
    net_manager_init(blynk_mqtt_start, blynk_mqtt_reconnect);

    int64_t poll_at_ms = (esp_timer_get_time() / 1000) + 10000; // give XY-MD02 10s to finish startup
    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms >= poll_at_ms) {
            if (blynk_mqtt_is_connected()) {
                poll_sensor_and_publish();
                poll_at_ms = now_ms + UPLOAD_INTERVAL_MS;
            } else {
                poll_at_ms = now_ms + 30000; // MQTT down — retry poll in 30 s
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
