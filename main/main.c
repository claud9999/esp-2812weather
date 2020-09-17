/* HSV->RGB code adapted from https://www.vagrearg.org/content/hsvrgb */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "sdkconfig.h"

#include "led_strip.h"

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
#include "driver/rmt.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_log.h"
#include "esp_netif.h"

#define GOT_IPV4_BIT BIT(0)

#define CONNECTED_BITS (GOT_IPV4_BIT)

static const char *connname;
static esp_netif_t *netif = NULL;

static const char *TAG = "2812weather";

bool mqtt_conn = false;

#include <stdint.h>

#define HSV_HUE_SEXTANT     256
#define HSV_HUE_STEPS       (6 * HSV_HUE_SEXTANT)

#define HSV_HUE_MIN     0
#define HSV_HUE_MAX     (HSV_HUE_STEPS - 1)
#define HSV_SAT_MIN     0
#define HSV_SAT_MAX     255
#define HSV_VAL_MIN     0
#define HSV_VAL_MAX     255

#define HSV_MONOCHROMATIC_TEST(s,v,r,g,b) \
    do { \
        if(!(s)) { \
             *(r) = *(g) = *(b) = (v); \
            return; \
        } \
    } while(0)

#define HSV_SEXTANT_TEST(sextant) \
    do { \
        if((sextant) > 5) { \
            (sextant) = 5; \
        } \
    } while(0)

/*
 * Pointer swapping:
 *  sext.   r g b   r<>b    g<>b    r <> g  result
 *  0 0 0   v u c           !u v c  u v c
 *  0 0 1   d v c               d v c
 *  0 1 0   c v u   u v c           u v c
 *  0 1 1   c d v   v d c       d v c   d v c
 *  1 0 0   u c v       u v c       u v c
 *  1 0 1   v c d       v d c   d v c   d v c
 *
 * if(sextant & 2)
 *  r <-> b
 *
 * if(sextant & 4)
 *  g <-> b
 *
 * if(!(sextant & 6) {
 *  if(!(sextant & 1))
 *      r <-> g
 * } else {
 *  if(sextant & 1)
 *      r <-> g
 * }
 */
#define HSV_SWAPPTR(a,b)    do { uint8_t *tmp = (a); (a) = (b); (b) = tmp; } while(0)
#define HSV_POINTER_SWAP(sextant,r,g,b) \
    do { \
        if((sextant) & 2) { \
            HSV_SWAPPTR((r), (b)); \
        } \
        if((sextant) & 4) { \
            HSV_SWAPPTR((g), (b)); \
        } \
        if(!((sextant) & 6)) { \
            if(!((sextant) & 1)) { \
                HSV_SWAPPTR((r), (g)); \
            } \
        } else { \
            if((sextant) & 1) { \
                HSV_SWAPPTR((r), (g)); \
            } \
        } \
    } while(0)

// hue is 0..(6 * 256 - 1) which each of the six sextants being 0..255
void fast_hsv2rgb_8bit(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    HSV_MONOCHROMATIC_TEST(s, v, r, g, b);  // Exit with grayscale if s == 0

    uint8_t sextant = h >> 8;

    HSV_SEXTANT_TEST(sextant);      // Optional: Limit hue sextants to defined space

    HSV_POINTER_SWAP(sextant, r, g, b); // Swap pointers depending which sextant we are in

    *g = v;     // Top level

    // Perform actual calculations
    uint8_t bb;
    uint16_t ww;

    /*
     * Bottom level: v * (1.0 - s)
     * --> (v * (255 - s) + error_corr) / 256
     */
    bb = ~s;
    ww = v * bb;
    ww += 1;        // Error correction
    ww += ww >> 8;      // Error correction
    *b = ww >> 8;

    uint8_t h_fraction = h & 0xff;  // 0...255

    if(!(sextant & 1)) {
        // *r = ...slope_up...;
        /*
         * Slope up: v * (1.0 - s * (1.0 - h))
         * --> (v * (255 - (s * (256 - h) + error_corr1) / 256) + error_corr2) / 256
         */
        ww = !h_fraction ? ((uint16_t)s << 8) : (s * (uint8_t)(-h_fraction));
        ww += ww >> 8;      // Error correction 1
        bb = ww >> 8;
        bb = ~bb;
        ww = v * bb;
        ww += v >> 1;       // Error correction 2
        *r = ww >> 8;
    } else {
        // *r = ...slope_down...;
        /*
         * Slope down: v * (1.0 - s * h)
         * --> (v * (255 - (s * h + error_corr1) / 256) + error_corr2) / 256
         */
        ww = s * h_fraction;
        ww += ww >> 8;      // Error correction 1
        bb = ww >> 8;
        bb = ~bb;
        ww = v * bb;
        ww += v >> 1;       // Error correction 2
        *r = ww >> 8;

        /*
         * A perfect match for h_fraction == 0 implies:
         *  *r = (ww >> 8) + (h_fraction ? 0 : 1)
         * However, this is an extra calculation that may not be required.
         */
    }
}

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
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
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

led_strip_t *strip = NULL;

static void set_color(uint32_t r, uint32_t g, uint32_t b) {
    if (!strip) return;

    ESP_LOGI(TAG, "setting color %d %d %d", r, g, b);
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    for (int i = 0; i < CONFIG_STRIP_LED_NUMBER; i++) ESP_ERROR_CHECK(strip->set_pixel(strip, i, r, g, b));

    ESP_ERROR_CHECK(strip->refresh(strip, 100));
}

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
            if(!strncmp(event->topic, CONFIG_MQTT_CMD_BASE "/rgb", strlen(CONFIG_MQTT_CMD_BASE "/rgb")))
            {
                char *end;
                uint32_t r = strtol(event->data, &end, 10);
                uint32_t g = strtol(end + 1, &end, 10);
                uint32_t b = strtol(end + 1, &end, 10);
                set_color(r, g, b);
            }
            if(!strncmp(event->topic, CONFIG_MQTT_CMD_BASE "/hsv", strlen(CONFIG_MQTT_CMD_BASE "/rgb")))
            {
                event->data[event->data_len] = '\0';
                char *end;
                uint32_t h = strtol(event->data, &end, 10);
                uint32_t s = strtol(end + 1, &end, 10);
                uint32_t v = strtol(end + 1, &end, 10);
                uint8_t r, g, b;
                fast_hsv2rgb_8bit(h, s, v, &r, &g, &b);
                set_color(r, g, b);
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
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static void blink_task(void *pvParameters)
{
    gpio_pad_select_gpio(CONFIG_BLINK_GPIO);
    gpio_set_direction(CONFIG_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_BLINK_GPIO, 0);

    while (true) {
        gpio_set_level(CONFIG_BLINK_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(CONFIG_BLINK_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void animate_task(void *pvParameters) {
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    uint8_t b = 0;
    while (true) {
        for (int j = 0; j < CONFIG_STRIP_LED_NUMBER; j++) {
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, b, b, b));
        }
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        vTaskDelay(pdMS_TO_TICKS(100));
        b += 1;
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

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_RMT_TX_GPIO, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) ESP_LOGE(TAG, "install WS2812 driver failed");
    else ESP_LOGI(TAG, "WS2812 driver installed");

    if(xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL) == pdPASS)
        ESP_LOGI(TAG, "[APP] Created blink task.");
    else ESP_LOGE(TAG, "[APP] Unable to create blink task.");
/*    if(xTaskCreate(animate_task, "animate_task", 8192, NULL, 5, NULL) == pdPASS)
        ESP_LOGI(TAG, "[APP] Created animate task.");
    else ESP_LOGE(TAG, "[APP] Unable to create animate task.");
    */

    while (true) {
        if(mqtt_conn) {
            ESP_LOGI(TAG, "Sending heartbeat.");
            esp_mqtt_client_publish(client, CONFIG_MQTT_TLM_BASE "/heartbeat", "beat", 0, 1, 0);
        }

        ESP_LOGI(TAG, "Sleeping 5s.");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
