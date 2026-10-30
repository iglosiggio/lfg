/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"

#include "display.h"
#include "buttons.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue = NULL;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq = 0;

#define MAX_PEERS 32
#define MAX_PEER_NAME 22
struct known_peer {
    uint8_t addr[ESP_NOW_ETH_ALEN];
    char name[MAX_PEER_NAME];
    int8_t rssi;
    TickType_t last_seen;
} peers[MAX_PEERS];
#define PEER_SILENCE_CUTOFF (10000 / portTICK_PERIOD_MS)

char own_name[MAX_PEER_NAME] = "NEW!!";

static void peer_update(uint8_t peer[ESP_NOW_ETH_ALEN], int8_t rssi, char name[MAX_PEER_NAME])
{
    TickType_t current_tick = xTaskGetTickCount();

    int peer_id;
    for (peer_id = 0; peer_id < MAX_PEERS; peer_id++)
        if (memcmp(peers[peer_id].addr, peer, ESP_NOW_ETH_ALEN) == 0) goto found;
    for (peer_id = 0; peer_id < MAX_PEERS; peer_id++)
        if (PEER_SILENCE_CUTOFF <= current_tick - peers[peer_id].last_seen) break;
    if (peer_id == MAX_PEERS) return;

    memcpy(peers[peer_id].addr, peer, ESP_NOW_ETH_ALEN);
found:
    memcpy(peers[peer_id].name, name, MAX_PEER_NAME);
    peers[peer_id].rssi = rssi;
    peers[peer_id].last_seen = xTaskGetTickCount();
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    int8_t rssi = recv_info->rx_ctrl->rssi;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    recv_cb->rssi = rssi;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, char name[MAX_PEER_NAME])
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *seq = buf->seq_num;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal != crc) {
        return -1;
    }

    memcpy(name, buf->name, MAX_PEER_NAME);

    return 0;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    buf->seq_num = s_example_espnow_seq++;
    buf->crc = 0;
    memcpy(buf->name, own_name, MAX_PEER_NAME);
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, sizeof(example_espnow_data_t));
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(broadcast_mac, send_param->buffer, sizeof(example_espnow_data_t)) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                example_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(broadcast_mac, send_param->buffer, sizeof(example_espnow_data_t)) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                char name[22];
                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, name);
                free(recv_cb->data);

                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d, rssi: %d, name: %s", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len, recv_cb->rssi, name);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    peer_update(recv_cb->mac_addr, recv_cb->rssi, name);
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->buffer = malloc(sizeof(example_espnow_data_t));
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_example_espnow_queue);
    s_example_espnow_queue = NULL;
    esp_now_deinit();
}


static void example_display_task(void *ignored)
{
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;


    int frame = 0;
    for (;;)
    {
        TickType_t current_tick = xTaskGetTickCount();
        char text[32];

        display_clear();
        snprintf(text, sizeof text, "Yo: %s", own_name);
        display_text(10, 0, text, false);
        for (int i = 0; i < MAX_PEERS; i++)
        {
            if (!peers[i].last_seen) continue;
            if (PEER_SILENCE_CUTOFF <= current_tick - peers[i].last_seen) continue;
            snprintf(text, sizeof text, "%d) %s", i, peers[i].name);
            display_text(10, (i+1) * 9, text, false);
            int written = snprintf(text, sizeof text, "%2d", peers[i].rssi);
            written = sizeof text < written ? sizeof text : written;
            display_text(128 - written * 6, (i+1) * 9 + 9, text, false);
        }

        uint8_t glyphs[][5] = {
            {
                0b00000000,
                0b00000000,
                0b00000000,
                0b00000000,
                0b00000000,
            },
            {
                0b00000000,
                0b00000000,
                0b00001000,
                0b00000000,
                0b00000000,
            },
            {
                0b00000000,
                0b00000100,
                0b00001110,
                0b00000100,
                0b00000000,
            },
            {
                0b00000000,
                0b00001110,
                0b00001110,
                0b00001110,
                0b00000000,
            },
            {
                0b00000100,
                0b00001110,
                0b00011111,
                0b00001110,
                0b00000100,
            },
            {
                0b00001110,
                0b00011111,
                0b00011111,
                0b00011111,
                0b00001110,
            },
            {
                0b00000100,
                0b00001110,
                0b00011111,
                0b00001110,
                0b00000100,
            },
            {
                0b00000000,
                0b00001110,
                0b00001110,
                0b00001110,
                0b00000000,
            },
            {
                0b00000000,
                0b00000100,
                0b00001110,
                0b00000100,
                0b00000000,
            },
            {
                0b00000000,
                0b00000000,
                0b00001000,
                0b00000000,
                0b00000000,
            },
        };
        draw_glyph(123, 0, glyphs[frame % 10], true);
        display_flush();
        vTaskDelay(xDelay);
    }
}

static void example_display_init()
{
    ESP_ERROR_CHECK( display_init() );
    display_clear();
    display_text(10, 8, "Booting...", false);
    display_text(10, 24, "ESP32-S3 dev board", false);
    display_flush();
    xTaskCreate(example_display_task, "example_display_task", 2048, NULL, 4, NULL);
}

static void example_buttons_task(void* ignored)
{
    button_event_t evt;
    while (xQueueReceive(buttons_event_queue(), &evt, portMAX_DELAY) == pdTRUE) {
        if (!evt.pressed) continue;
        switch (evt.id) {
            case BTN_KEY1: strcpy(own_name, "Wavelength"); break;
            case BTN_KEY2: strcpy(own_name, "Codenames (Pict)"); break;
            case BTN_KEY3: strcpy(own_name, "Codenames (Word)"); break;
            case BTN_KEY4: strcpy(own_name, "The Mind"); break;
            default: break;
        }
    }
}

static void example_buttons_init()
{

    ESP_ERROR_CHECK(buttons_init());
    xTaskCreate(example_buttons_task, "example_buttons_task", 2048, NULL, 4, NULL);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    example_wifi_init();
    example_espnow_init();
    example_display_init();
    example_buttons_init();
}
