extern "C" {
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "nvs_flash.h"

#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "esp_https_ota.h"
} /* end extern "C" */

#include "FastLED.h"
#include "FX.h"
#include "palettes.h"

extern "C" {
#define GOT_IPV4_BIT BIT(0)

#define CONNECTED_BITS (GOT_IPV4_BIT)

static const char *connname;
static esp_netif_t *netif = NULL;

static const char *TAG = "2812weather";

bool mqtt_conn = false;

static void on_wifi_disconnect(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_wifi_connect();
    mqtt_conn = false;
}

static void on_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    static esp_ip4_addr_t ip_addr;

    ESP_LOGI(TAG, "Got event!");
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    memcpy(&ip_addr, &event->ip_info.ip, sizeof(ip_addr));

    ESP_LOGI(TAG, "Connected to %s", connname);
    ESP_LOGI(TAG, "IPv4 address: " IPSTR, IP2STR(&ip_addr));
}

static void start(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();

    netif = esp_netif_new(&netif_config);

    assert(netif);

    esp_netif_attach_wifi_station(netif);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config;
    strcpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD);

    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    connname = CONFIG_WIFI_SSID;
}

static void stop(void) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) return;
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(netif));
    esp_netif_destroy(netif);
    netif = NULL;
}

#define SUB_TOPIC_WILDCARD "cmd/2812weather/+"

esp_mqtt_client_handle_t client;

bool ota = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    client = event->client;
    int msg_id;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, CONFIG_MQTT_CMD_BASE "/+", 0);
	    esp_mqtt_client_publish(client, CONFIG_MQTT_TLM_BASE "/avail", "ON", 0, 1, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            mqtt_conn = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_conn = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            event->data[event->data_len] = '\0';
            ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
            if(!strncmp(event->topic, CONFIG_MQTT_CMD_BASE "/ota", strlen(CONFIG_MQTT_CMD_BASE "/ota"))) {
                ota = true;
                ESP_LOGI(TAG, "setting OTA");
                break;
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = { .uri = CONFIG_BROKER_URL, };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static void animate_task(void *pvParameters) {
    CRGB leds[CONFIG_LED_STRIP_COUNT];

    FastLED.addLeds<WS2812, CONFIG_LED_STRIP_GPIO>(leds, CONFIG_LED_STRIP_COUNT);

    while (true) {
        for(int i = 0; i < 10; i++)
            leds[random() % CONFIG_LED_STRIP_COUNT] = CHSV(180, 200, 32 + random() % 150);
        yield();
        FastLED.show();
        delay(300);
    }
}

static void blink_task(void *pvParameters) {
    gpio_pad_select_gpio((gpio_num_t)CONFIG_BLINK_GPIO);
    gpio_set_direction((gpio_num_t)CONFIG_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CONFIG_BLINK_GPIO, 0);

    while (true) {
        gpio_set_level((gpio_num_t)CONFIG_BLINK_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level((gpio_num_t)CONFIG_BLINK_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    start();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&stop));

    mqtt_app_start();

    if(xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL) == pdPASS)
        ESP_LOGI(TAG, "[APP] Created blink task.");
    else ESP_LOGE(TAG, "[APP] Unable to create blink task.");

    if(xTaskCreate(animate_task, "animate_task", 8192, NULL, 5, NULL) == pdPASS)
        ESP_LOGI(TAG, "[APP] Created animate task.");
    else ESP_LOGE(TAG, "[APP] Unable to create animate task.");

    while (true) {
        if (ota) {
            esp_http_client_config_t config = {
                .url = "http://ota.lan/parlor.bin",
                .cert_pem = "MIID0TCCArmgAwIBAgIUIb5AgNFSFJLtCfRfjHnlJArS4rowDQYJKoZIhvcNAQELBQAweDELMAkGA1UEBhMCVVMxCzAJBgNVBAgMAkNBMREwDwYDVQQHDAhTYW4gSm9zZTETMBEGA1UECgwKaG90Y2F0Lm9yZzEQMA4GA1UEAwwHbnVjLmxhbjEiMCAGCSqGSIb3DQEJARYTY2RrbmlnaHRAaG90Y2F0Lm9yZzAeFw0yMDA5MjEwMjM4MDlaFw00ODAyMDcwMjM4MDlaMHgxCzAJBgNVBAYTAlVTMQswCQYDVQQIDAJDQTERMA8GA1UEBwwIU2FuIEpvc2UxEzARBgNVBAoMCmhvdGNhdC5vcmcxEDAOBgNVBAMMB251Yy5sYW4xIjAgBgkqhkiG9w0BCQEWE2Nka25pZ2h0QGhvdGNhdC5vcmcwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDEH2pK2QqG7eGHPyPTlCq1DSZdxH/7uKSyN63DLwuqpaOZNNsd7Ac8/uGO/c2bc9fkyKX2PsTxKfrcKQ/ubhRsR4Qk07neTZ0OLuBFOGiz3sxxo7BlvdG3cf00lfnZiPzLxFmW3tsEYagRG3vw3UkvwOM5kIkZC/8j/nIjw6Wad49qCM/xNegmUdEWheFSm+HTfQs7Zo4y6xk439Ta0neq1+B5ik3x5nsAJkdIBgiHgZxpoFqAXxdP9aY4T80mqZ75laBYZOFtEdeuBbrRQL5PiQ1HZjvOjqdjhLO6zVQ8XSbFAE7YUoVc0bGfO5G+Jy3uj6eIJZwMhigvH15owgr7AgMBAAGjUzBRMB0GA1UdDgQWBBQaQgOohRnx5wV5cgMP0D9AvRVkRzAfBgNVHSMEGDAWgBQaQgOohRnx5wV5cgMP0D9AvRVkRzAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3DQEBCwUAA4IBAQBoFMc+uQGIAm45MXUiaXDRM+v3zEX5cqUUUJ3kXnZa0yWO8Fr6whY/9JyGKmuXegXw+NACF+GOIFJpZJx7JlPlhhpawfWTb/Ys6UmIyq7CEk6/X36VAdmVcQxP7zP1x9Z0XQNJOQ5oHpgRwws0c/uoTaT7f6nlkoA27aM2oRyNk/29XUCA1pjds7ZBgzXWsmYvHaA7EAXNvrgVbYg8vb8m2/O6v+iFFNALMS/+ogTT8aPpOWpiC6q3xLbu+IfYACA2iPgGhxAQI9c5WvEQWWpqC3ucAyvStXNIxeGmIiiT52QSOlg40fvkGRILCxbEXtAb2BE/Xx/xJdCAiKqZP8ch"
            };

            ESP_LOGI(TAG, "Starting OTA");
            esp_err_t ret = esp_https_ota(&config);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Restarting...");
                esp_restart();
            }
            else {
                ESP_LOGE(TAG, "Unable to OTA");
            }

            ota = false;
        }
        if(mqtt_conn) {
            ESP_LOGI(TAG, "Sending heartbeat.");
            esp_mqtt_client_publish(client, CONFIG_MQTT_TLM_BASE "/heartbeat", "beat", 0, 1, 0);
        }

        ESP_LOGI(TAG, "Sleeping 5s.");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
} /* end extern "C" */
