/* ATLAS-S agent firmware, Board B. The decisions are in agent_core (host-tested); this file only
 * schedules them: a 100 Hz control task on core 1, the radio/UART receive task on core 0.
 *
 * Safety chain
 *   1. agent_core: carrier heartbeat timeout -> SAFE_STOP; stall/undervoltage -> FAULT (motor rail off)
 *   2. this file : the TPS3823 is kicked only by the control task, and only if the loop ran on time;
 *                  a late or hung loop stops the kicks -> WDT_OK low -> hardware cuts VM_SW within 1.6 s
 *   3. ESP task watchdog (2 s) resets the chip if the control task stops entirely              */
#include <string.h>
#include "atlas_agent.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal_board_b.h"
#include "links.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "atlas";
static atlas_agent_t agent;
static SemaphoreHandle_t lock;

static void send_cb(int link, uint8_t type, const void *p, size_t n, void *ctx) { (void)ctx; links_send(link, type, p, n); }

static void frame_cb(int link, uint8_t type, const uint8_t *p, int n)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    atlas_agent_on_frame(&agent, link, type, p, n);
    xSemaphoreGive(lock);
}

static void control_task(void *arg)
{
    esp_task_wdt_add(NULL);
    TickType_t wake = xTaskGetTickCount();
    int64_t last = esp_timer_get_time();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(AGENT_DT_MS));
        int64_t t0 = esp_timer_get_time();
        atlas_agent_in_t in; memset(&in, 0, sizeof in);
        board_b_read(&in);
        in.now_ms = (uint32_t)(t0 / 1000);
        xSemaphoreTake(lock, portMAX_DELAY);
        const atlas_agent_out_t *o = atlas_agent_tick(&agent, &in);
        board_b_apply(o, in.vbat_mv);
        xSemaphoreGive(lock);
        int64_t t1 = esp_timer_get_time();
        bool on_time = (t0 - last) < 3 * AGENT_DT_MS * 1000 && (t1 - t0) < AGENT_DT_MS * 1000;
        if (on_time) board_b_kick_watchdog();
        else ESP_LOGW(TAG, "control loop late: period %lld us, run %lld us", t0 - last, t1 - t0);
        last = t0;
        esp_task_wdt_reset();
    }
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    board_b_init();
    lock = xSemaphoreCreateMutex();
    atlas_agent_cfg_t cfg;
#ifdef CONFIG_ATLAS_AGENT_SCOUT
    atlas_agent_default_cfg(&cfg, true);
#else
    atlas_agent_default_cfg(&cfg, false);
#endif
    cfg.sens_gain = CONFIG_ATLAS_SENS_GAIN_X1000 / 1000.0f;
    atlas_agent_init(&agent, &cfg, send_cb, NULL);
    links_init(frame_cb);
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    ESP_LOGI(TAG, "ATLAS agent up (%s)", cfg.has_pi ? "scout" : "research");
}
