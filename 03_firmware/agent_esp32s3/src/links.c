#include "links.h"
#include <string.h>
#include "atlas_agent.h"
#include "board_b_pins.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

static const char *TAG = "links";
static links_frame_cb frame_cb;
static int sock = -1;
static struct sockaddr_in carrier;
static atlas_rx_t rx_udp, rx_dock, rx_pi;
static uint8_t seq[2];
#define UART_PI   UART_NUM_1
#define UART_DOCK UART_NUM_2

static void on_frame(uint8_t type, uint8_t s, const uint8_t *p, int n, void *ctx)
{
    (void)s; frame_cb((int)(intptr_t)ctx, type, p, n);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) esp_wifi_connect();
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) ESP_LOGI(TAG, "Wi-Fi up");
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t c = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&c));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);
    wifi_config_t w = {0};
    strncpy((char *)w.sta.ssid, CONFIG_ATLAS_WIFI_SSID, sizeof w.sta.ssid);
    strncpy((char *)w.sta.password, CONFIG_ATLAS_WIFI_PASS, sizeof w.sta.password);
    w.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &w));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));          /* latency over power: heartbeat is 10 Hz */
    ESP_ERROR_CHECK(esp_wifi_start());

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in me = {.sin_family = AF_INET, .sin_port = htons(CONFIG_ATLAS_CARRIER_PORT), .sin_addr.s_addr = htonl(INADDR_ANY)};
    bind(sock, (struct sockaddr *)&me, sizeof me);
    struct timeval tv = {0, 20000}; setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    carrier.sin_family = AF_INET; carrier.sin_port = htons(CONFIG_ATLAS_CARRIER_PORT);
    inet_pton(AF_INET, CONFIG_ATLAS_CARRIER_IP, &carrier.sin_addr);
}

static void uart_setup(uart_port_t u, int baud, int tx, int rx)
{
    uart_config_t c = {.baud_rate = baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                       .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(u, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(u, &c));
    ESP_ERROR_CHECK(uart_set_pin(u, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void rx_task(void *arg)
{
    uint8_t buf[256];
    for (;;) {
        if (sock >= 0) {
            struct sockaddr_in from; socklen_t fl = sizeof from;
            int n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &fl);
            if (n > 0) atlas_rx_feed(&rx_udp, buf, (size_t)n);
        }
        int n = uart_read_bytes(UART_DOCK, buf, sizeof buf, 0);
        if (n > 0) atlas_rx_feed(&rx_dock, buf, (size_t)n);
        n = uart_read_bytes(UART_PI, buf, sizeof buf, 0);
        if (n > 0) atlas_rx_feed(&rx_pi, buf, (size_t)n);
        if (sock < 0) vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void links_init(links_frame_cb cb)
{
    frame_cb = cb;
    atlas_rx_init(&rx_udp, on_frame, (void *)(intptr_t)LINK_CARRIER);
    atlas_rx_init(&rx_dock, on_frame, (void *)(intptr_t)LINK_CARRIER);
    atlas_rx_init(&rx_pi, on_frame, (void *)(intptr_t)LINK_PI);
    uart_setup(UART_PI, 921600, PIN_PI_TX, PIN_PI_RX);
    uart_setup(UART_DOCK, 115200, PIN_DOCK_TX, PIN_DOCK_RX);
    wifi_init();
    xTaskCreatePinnedToCore(rx_task, "links_rx", 4096, NULL, 10, NULL, 0);
}

void links_send(int link, uint8_t type, const void *payload, size_t n)
{
    uint8_t f[ATLAS_MAX_FRAME + 4];
    size_t len = atlas_frame_build(type, seq[link]++, payload, n, f);
    if (!len) return;
    if (link == LINK_PI) { uart_write_bytes(UART_PI, f, len); return; }
    if (sock >= 0) sendto(sock, f, len, 0, (struct sockaddr *)&carrier, sizeof carrier);
    uart_write_bytes(UART_DOCK, f, len);       /* harmless when undocked: pad is open */
}
