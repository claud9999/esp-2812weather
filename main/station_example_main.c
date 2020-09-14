/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "driver/adc.h"

static const char *TAG = "MQTT_EXAMPLE";

#define GPIO_LED GPIO_NUM_2

#define SUB_TOPIC_WILDCARD "cmnd/spr/+"
#define PUB_TOPIC_PREFIX "stat/spr"
#define MOISTURE_TOPIC_PREFIX "stat/moisture"
#define AVAIL_TOPIC "tele/spr"

#define SPR_CNT 6
int spr_gpios[SPR_CNT] = {
    GPIO_NUM_26,
    GPIO_NUM_25,
    GPIO_NUM_17,
    GPIO_NUM_16,
    GPIO_NUM_27,
    GPIO_NUM_14
};
int spr_gpio_state[SPR_CNT];

#define MOISTURE_CNT 1
int moisture_gpios[MOISTURE_CNT] = {
    ADC1_CHANNEL_7
};

void send_status(esp_mqtt_client_handle_t client) {
    int i = 0;
    for(i = 0; i < SPR_CNT; i++) {
	char topic[32];
	snprintf(topic, 32, "%s/%d", PUB_TOPIC_PREFIX, i);
 
        esp_mqtt_client_publish(client, topic,
            spr_gpio_state[i] ? "ON" : "OFF", 0, 1, 0);
    }
}

void report_moisture(esp_mqtt_client_handle_t client, int m) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(moisture_gpios[m], ADC_ATTEN_DB_0);
    int val = adc1_get_raw(moisture_gpios[m]);
    char topic[32];
    char valstr[32];
    snprintf(topic, 32, "%s/%d", MOISTURE_TOPIC_PREFIX, m);
    snprintf(valstr, 32, "%d", val);
    esp_mqtt_client_publish(client, topic, valstr, 0, 1, 0);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, SUB_TOPIC_WILDCARD, 0);
	    esp_mqtt_client_publish(client, AVAIL_TOPIC, "ON", 0, 1, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
	    send_status(client);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
	    esp_mqtt_client_publish(client, AVAIL_TOPIC, "OFF", 0, 1, 0);
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
            ESP_LOGI(TAG, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s\r\n", event->data_len, event->data);

            char *numpart = rindex(event->topic, '/');
	    if(!numpart) break;
	    numpart++;

	    if(*numpart == 'm') {
                /* this is a request for a moisture sensor reading */
                int m = atoi(numpart + 1);
                if(m >= 0 && m < MOISTURE_CNT) {
                    report_moisture(client, m);
                }
	    }

            int sw = atoi(numpart);
            if(sw >= 0 && sw < SPR_CNT) {
                if(!strncmp(event->data, "ON", 2)) {
                    if (gpio_set_level(spr_gpios[sw], 0) == ESP_OK) {
                        spr_gpio_state[sw] = 1;
                    } else {
                        ESP_LOGE(TAG, "unable to set level for %d", sw);
                    }
                } else {
                    if(gpio_set_level(spr_gpios[sw], 1) == ESP_OK) {
                        spr_gpio_state[sw] = 0;
                    } else {
                        ESP_LOGE(TAG, "unable to set level for %d", sw);
                    }
                }
	    }
	    send_status(client);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    int i = 0;
    gpio_pad_select_gpio(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED, 0);

    for(i = 0; i < SPR_CNT; i++) {
        gpio_pad_select_gpio(spr_gpios[i]);
        gpio_set_direction(spr_gpios[i], GPIO_MODE_OUTPUT);
        gpio_set_level(spr_gpios[i], 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(spr_gpios[i], 1);
    }

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
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    gpio_set_level(GPIO_LED, 1);

    mqtt_app_start();
}
