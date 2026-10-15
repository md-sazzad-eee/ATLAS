/* ATLAS-M carrier firmware, Board A. Decisions in carrier_core (host-tested); this file schedules
 * the 100 Hz control task, the USB link to the ROS 2 computer and the two dock DATA UARTs.
 * The TPS3823 is kicked only by an on-time control loop; a late loop drops MOT_ENABLE in hardware. */
#include <string.h>
#include "atlas_carrier.h"
#include "board_a_pins.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal_board_a.h"

static atlas_carrier_t carrier;
static SemaphoreHandle_t lock;
static atlas_rx_t rx_usb, rx_dock[2];
static uint8_t seq;
static const uart_port_t DOCK_UART[2] = {UART_NUM_1, UART_NUM_2};

static void usb_send(uint8_t type, const void *p, size_t n, void *ctx)
{
    (void)ctx; uint8_t f[ATLAS_MAX_FRAME + 4];
    size_t len = atlas_frame_build(type, seq++, p, n, f);
    if (len) usb_serial_jtag_write_bytes(f, len, 0);
}

static void on_usb(uint8_t type, uint8_t s, const uint8_t *p, int n, void *ctx)
{
    (void)s; (void)ctx;
    if (type == MSG_DOCK_RELAY && n >= 2 && p[0] < 2) {        /* ROS -> agent over the dock pad */
        uint8_t f[ATLAS_MAX_FRAME + 4]; size_t len = atlas_frame_build(p[1], 0, p + 2, (size_t)(n - 2), f);
        if (len) uart_write_bytes(DOCK_UART[p[0]], f, len);
        return;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    atlas_carrier_on_frame(&carrier, type, p, n);
    xSemaphoreGive(lock);
}

static void on_dock(uint8_t type, uint8_t s, const uint8_t *p, int n, void *ctx)
{
    (void)s; uint8_t buf[ATLAS_MAX_PAYLOAD];
    if (n > ATLAS_MAX_PAYLOAD - 2) return;
    buf[0] = (uint8_t)(intptr_t)ctx; buf[1] = type; memcpy(buf + 2, p, (size_t)n);
    usb_send(MSG_DOCK_RELAY, buf, (size_t)n + 2, NULL);        /* agent -> ROS, tagged with the bay */
}

static void link_task(void *arg)
{
    uint8_t buf[256];
    for (;;) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, pdMS_TO_TICKS(2));
        if (n > 0) atlas_rx_feed(&rx_usb, buf, (size_t)n);
        for (int b = 0; b < 2; b++) {
            n = uart_read_bytes(DOCK_UART[b], buf, sizeof buf, 0);
            if (n > 0) atlas_rx_feed(&rx_dock[b], buf, (size_t)n);
        }
    }
}

static void control_task(void *arg)
{
    esp_task_wdt_add(NULL);
    TickType_t wake = xTaskGetTickCount();
    int64_t last = esp_timer_get_time();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CARRIER_DT_MS));
        int64_t t0 = esp_timer_get_time();
        atlas_carrier_in_t in; memset(&in, 0, sizeof in);
        board_a_read(&in); in.now_ms = (uint32_t)(t0 / 1000);
        xSemaphoreTake(lock, portMAX_DELAY);
        const atlas_carrier_out_t *o = atlas_carrier_tick(&carrier, &in);
        board_a_apply(o, in.vbat_mv);
        xSemaphoreGive(lock);
        int64_t t1 = esp_timer_get_time();
        if ((t0 - last) < 3 * CARRIER_DT_MS * 1000 && (t1 - t0) < CARRIER_DT_MS * 1000) board_a_kick_watchdog();
        last = t0;
        esp_task_wdt_reset();
    }
}

void app_main(void)
{
    board_a_init();
    lock = xSemaphoreCreateMutex();
    atlas_carrier_cfg_t cfg; atlas_carrier_default_cfg(&cfg);
    atlas_carrier_init(&carrier, &cfg, usb_send, NULL);
    usb_serial_jtag_driver_config_t u = {.rx_buffer_size = 1024, .tx_buffer_size = 2048};
    usb_serial_jtag_driver_install(&u);
    atlas_rx_init(&rx_usb, on_usb, NULL);
    const int tx[2] = {PIN_DATA_TX_1, PIN_DATA_TX_2}, rx[2] = {PIN_DATA_RX_1, PIN_DATA_RX_2};
    for (int b = 0; b < 2; b++) {
        uart_config_t c = {.baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                           .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
        uart_driver_install(DOCK_UART[b], 512, 512, 0, NULL, 0); uart_param_config(DOCK_UART[b], &c);
        uart_set_pin(DOCK_UART[b], tx[b], rx[b], UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        atlas_rx_init(&rx_dock[b], on_dock, (void *)(intptr_t)b);
    }
    xTaskCreatePinnedToCore(link_task, "link", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}
