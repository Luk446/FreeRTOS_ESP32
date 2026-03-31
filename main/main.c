
/*
    *** Includes, configs and macros ***
*/


// ----- Headers -----

// standard C libs

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// freertos headers

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// esp-idf headers

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "monitor.h"

// project constraints

#if configTICK_RATE_HZ != 1000
#error "Assignment 3 requires FreeRTOS tick = 1 ms (configTICK_RATE_HZ == 1000)."
#endif

// ----- Pin Definitions -----

// Inputs
#define PIN_SYNC    13
#define PIN_IN_A    14
#define PIN_IN_B    25
#define PIN_IN_S    26
#define PIN_IN_MODE 27

// Outputs (ACKs)
#define ACK_A   16
#define ACK_B   17
#define ACK_AGG 18
#define ACK_C   19
#define ACK_D   21
#define ACK_S   22

// ----- duration configs -----

// WorkKernel budgets (ESP32 Wroom @ 240 MHz)

#define BUDGET_A_CYCLES     672000u
#define BUDGET_B_CYCLES     960000u
#define BUDGET_AGG_CYCLES   480000u
#define BUDGET_C_CYCLES    1680000u
#define BUDGET_D_CYCLES     960000u
#define BUDGET_S_CYCLES     600000u

#define AGG_FALLBACK_TOKEN  0xDEADBEEFu

// Task periods (ms)

#define PERIOD_A_MS   10
#define PERIOD_B_MS   20
#define PERIOD_AGG_MS 20
#define PERIOD_C_MS   50
#define PERIOD_D_MS   50

// Monitor reporting frequency
#define MONITOR_REPORT_PERIOD_S 2u
#define MONITOR_POLL_PERIOD_MS  100u

// Enable/disable per-task UART logging
#define TASK_UART_LOG_ENABLED 0

// conditional for task logging 
#if TASK_UART_LOG_ENABLED
#define TASK_LOG(...) printf(__VA_ARGS__)
#else
#define TASK_LOG(...) do { } while (0)
#endif

static const char *TAG = "assignment3";

// WorkKernel input
uint32_t WorkKernel(uint32_t budget_cycles, uint32_t seed);

/*
    *** Synchronization objects, shared state ***
*/

// FreeRTOS synchronisation objects 

// Task S is released by the PIN_IN_S ISR, which gives this semaphore
static SemaphoreHandle_t g_sporadic_sem  = NULL;  // ISR → Task S
// generated tokens that AGG reads from A/B, protected by mutex
static SemaphoreHandle_t g_token_mutex   = NULL;  // protects shared token state
// SYNC ISR gives this semaphore to unblock
static SemaphoreHandle_t g_sync_sem      = NULL;  // ISR → app_main for first SYNC

// shared state - global struct that tracks the; execution index of each task, tokens from A/B, and bool flags 
typedef struct {
    uint32_t idx_a;
    uint32_t idx_b;
    uint32_t idx_agg;
    uint32_t idx_c;
    uint32_t idx_d;
    uint32_t idx_s;

    bool     token_a_valid;
    bool     token_b_valid;
    uint32_t token_a;
    uint32_t token_b;
} sched_state_t;

// global state variable
static sched_state_t g_state;

/*
    *** Hardware init (GPIO, PCNT) ***
*/

// ----- PCNT handles -----

// PCNT units for counting edges on PIN_IN_A and PIN_IN_B
static pcnt_unit_handle_t g_pcnt_unit_a = NULL;
static pcnt_unit_handle_t g_pcnt_unit_b = NULL;

// previous counts for delta calc
static int g_pcnt_prev_a = 0;
static int g_pcnt_prev_b = 0;

// ----- SYNC state -----

// SYNC timestamp (microseconds)
static volatile uint32_t g_sync_time_us   = 0;

// SYNC seen flag and schedule started flag 
static volatile bool     g_sync_seen      = false;
static volatile bool     g_schedule_started = false;

// SYNC tick (FreeRTOS ticks)
static TickType_t g_sync_tick = 0;

// ----- GPIO helpers -----

// configures ACK GPIOs
static inline void ack_high(gpio_num_t pin) { gpio_set_level(pin, 1); }
static inline void ack_low(gpio_num_t pin)  { gpio_set_level(pin, 0); }

// Get current time in ms
static inline int64_t now_us(void)          { return esp_timer_get_time(); }

/*
    *** Edge counting and ISRs ***
*/

// ----- Edge counting for A and B O(1) -----

// functions do:
// read current count from PCNT,
// compute delta from previous count
// store current count
// return delta

static uint32_t edge_count_a_last_period(void)
{
    int current = 0;
    pcnt_unit_get_count(g_pcnt_unit_a, &current);
    const uint32_t delta = (uint32_t)(current - g_pcnt_prev_a);
    g_pcnt_prev_a = current;
    return delta;
}

static uint32_t edge_count_b_last_period(void)
{
    int current = 0;
    pcnt_unit_get_count(g_pcnt_unit_b, &current);
    const uint32_t delta = (uint32_t)(current - g_pcnt_prev_b);
    g_pcnt_prev_b = current;
    return delta;
}

// ----- ISRs -----

// grab timestamp of SYNC
// set a flag that SYNC has been seen
// give semaphore to unblock app_main waiting for SYNC
static void IRAM_ATTR isr_in_s(void *arg)
{
    (void)arg;
    if (!__atomic_load_n(&g_schedule_started, __ATOMIC_RELAXED)) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_sporadic_sem, &xHigherPriorityTaskWoken);
    notifySRelease();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// SYNC ISR
// if SYNC already seen, return
// else grab timestamp
// set SYNC seen flag
// give semaphore to unblock app_main waiting for SYNC
static void IRAM_ATTR isr_sync(void *arg)
{
    (void)arg;
    if (__atomic_load_n(&g_sync_seen, __ATOMIC_RELAXED)) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    __atomic_store_n(&g_sync_time_us, (uint32_t)esp_timer_get_time(), __ATOMIC_RELAXED);
    __atomic_store_n(&g_sync_seen, true, __ATOMIC_RELEASE);
    if (g_sync_sem != NULL) {
        xSemaphoreGiveFromISR(g_sync_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// ----- GPIO init -----

static void gpio_init(void)
{
    const gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << PIN_SYNC) |
                        (1ULL << PIN_IN_S) |
                        (1ULL << PIN_IN_MODE),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    const gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << ACK_A) |
                        (1ULL << ACK_B) |
                        (1ULL << ACK_AGG) |
                        (1ULL << ACK_C) |
                        (1ULL << ACK_D) |
                        (1ULL << ACK_S),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&input_conf));
    ESP_ERROR_CHECK(gpio_config(&output_conf));

    ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_SYNC, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_IN_S, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_IN_MODE, GPIO_PULLDOWN_ONLY));

    ack_low(ACK_A);
    ack_low(ACK_B);
    ack_low(ACK_AGG);
    ack_low(ACK_C);
    ack_low(ACK_D);
    ack_low(ACK_S);

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_SYNC, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_IN_S, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_SYNC, isr_sync, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_IN_S, isr_in_s, NULL));
}

// ----- PCNT init (same as Assignment 2) -----

static void pcnt_init(void)
{
    const pcnt_unit_config_t unit_cfg = {
        .low_limit  = -1,
        .high_limit = 30000,
        .flags      = { .accum_count = 1 },
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &g_pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &g_pcnt_unit_b));

    const pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt_unit_a, &filter_cfg));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt_unit_b, &filter_cfg));

    const pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = PIN_IN_A, .level_gpio_num = -1 };
    pcnt_channel_handle_t chan_a;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit_a, &chan_a_cfg, &chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    const pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = PIN_IN_B, .level_gpio_num = -1 };
    pcnt_channel_handle_t chan_b;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit_b, &chan_b_cfg, &chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_unit_b));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit_b));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_unit_b));
}

// ----- SYNC wait -----

static int64_t wait_for_sync_rising_edge(void)
{
    __atomic_store_n(&g_sync_seen, false, __ATOMIC_RELEASE);

    while (gpio_get_level(PIN_SYNC) != 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    while (!__atomic_load_n(&g_sync_seen, __ATOMIC_ACQUIRE)) {
        xSemaphoreTake(g_sync_sem, pdMS_TO_TICKS(100));
    }

    return (int64_t)__atomic_load_n(&g_sync_time_us, __ATOMIC_ACQUIRE);
}

// ----- State reset -----

static void reset_schedule_state(void)
{
    g_state.idx_a   = 0;
    g_state.idx_b   = 0;
    g_state.idx_agg = 0;
    g_state.idx_c   = 0;
    g_state.idx_d   = 0;
    g_state.idx_s   = 0;

    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit_b));
    g_pcnt_prev_a = 0;
    g_pcnt_prev_b = 0;

    g_state.token_a_valid = false;
    g_state.token_b_valid = false;
    g_state.token_a = 0;
    g_state.token_b = 0;
}

// ===================================================================
// FreeRTOS Task Functions
// ===================================================================

// ----- Task A: period 10 ms -----
static void task_a_func(void *arg)
{
    (void)arg;
    TickType_t xLastWake = g_sync_tick;
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_A_MS);

    while (true) {
        const uint32_t count_a = edge_count_a_last_period();
        const uint32_t id   = g_state.idx_a;
        const uint32_t seed = (id << 16) ^ count_a ^ 0xA1u;

        beginTaskA(id);
        ack_high(ACK_A);
        const uint32_t token = WorkKernel(BUDGET_A_CYCLES, seed);
        ack_low(ACK_A);
        endTaskA();

        xSemaphoreTake(g_token_mutex, portMAX_DELAY);
        g_state.token_a = token;
        g_state.token_a_valid = true;
        xSemaphoreGive(g_token_mutex);

        g_state.idx_a++;

        TASK_LOG("A,%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n", id, count_a, token);

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ----- Task B: period 20 ms -----
static void task_b_func(void *arg)
{
    (void)arg;
    TickType_t xLastWake = g_sync_tick;
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_B_MS);

    while (true) {
        const uint32_t count_b = edge_count_b_last_period();
        const uint32_t id   = g_state.idx_b;
        const uint32_t seed = (id << 16) ^ count_b ^ 0xB2u;

        beginTaskB(id);
        ack_high(ACK_B);
        const uint32_t token = WorkKernel(BUDGET_B_CYCLES, seed);
        ack_low(ACK_B);
        endTaskB();

        xSemaphoreTake(g_token_mutex, portMAX_DELAY);
        g_state.token_b = token;
        g_state.token_b_valid = true;
        xSemaphoreGive(g_token_mutex);

        g_state.idx_b++;

        TASK_LOG("B,%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n", id, count_b, token);

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ----- Task AGG: period 20 ms -----
static void task_agg_func(void *arg)
{
    (void)arg;
    TickType_t xLastWake = g_sync_tick;
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_AGG_MS);

    while (true) {
        xSemaphoreTake(g_token_mutex, portMAX_DELAY);
        const uint32_t agg = (g_state.token_a_valid && g_state.token_b_valid)
                                 ? (g_state.token_a ^ g_state.token_b)
                                 : AGG_FALLBACK_TOKEN;
        xSemaphoreGive(g_token_mutex);

        const uint32_t id   = g_state.idx_agg;
        const uint32_t seed = (id << 16) ^ agg ^ 0xD4u;

        beginTaskAGG(id);
        ack_high(ACK_AGG);
        const uint32_t token = WorkKernel(BUDGET_AGG_CYCLES, seed);
        ack_low(ACK_AGG);
        endTaskAGG();

        g_state.idx_agg++;

        TASK_LOG("AGG,%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n", id, agg, token);

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ----- Task C: period 50 ms, mode-controlled -----
static void task_c_func(void *arg)
{
    (void)arg;
    TickType_t xLastWake = g_sync_tick;
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_C_MS);

    while (true) {
        if (gpio_get_level(PIN_IN_MODE) == 0) {
            vTaskDelayUntil(&xLastWake, xPeriod);
            continue;  // mode disabled — skip, don't advance IDX
        }

        const uint32_t id   = g_state.idx_c;
        const uint32_t seed = (id << 16) ^ 0xC3u;

        beginTaskC(id);
        ack_high(ACK_C);
        const uint32_t token = WorkKernel(BUDGET_C_CYCLES, seed);
        ack_low(ACK_C);
        endTaskC();

        g_state.idx_c++;

        TASK_LOG("C,%" PRIu32 ",%" PRIu32 "\n", id, token);

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ----- Task D: period 50 ms, mode-controlled -----
static void task_d_func(void *arg)
{
    (void)arg;
    TickType_t xLastWake = g_sync_tick;
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_D_MS);

    while (true) {
        if (gpio_get_level(PIN_IN_MODE) == 0) {
            vTaskDelayUntil(&xLastWake, xPeriod);
            continue;  // mode disabled — skip, don't advance IDX
        }

        const uint32_t id   = g_state.idx_d;
        const uint32_t seed = (id << 16) ^ 0xD5u;

        beginTaskD(id);
        ack_high(ACK_D);
        const uint32_t token = WorkKernel(BUDGET_D_CYCLES, seed);
        ack_low(ACK_D);
        endTaskD();

        g_state.idx_d++;

        TASK_LOG("D,%" PRIu32 ",%" PRIu32 "\n", id, token);

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ----- Task S: sporadic, semaphore-driven -----
static void task_s_func(void *arg)
{
    (void)arg;

    while (true) {
        xSemaphoreTake(g_sporadic_sem, portMAX_DELAY);

        const uint32_t id   = g_state.idx_s;
        const uint32_t seed = (id << 16) ^ 0x55u;

        beginTaskS(id);
        ack_high(ACK_S);
        const uint32_t token = WorkKernel(BUDGET_S_CYCLES, seed);
        ack_low(ACK_S);
        endTaskS();

        g_state.idx_s++;

        TASK_LOG("S,%" PRIu32 ",%" PRIu32 "\n", id, token);
    }
}

// ----- Monitor polling task -----
static void monitor_task_func(void *arg)
{
    (void)arg;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(MONITOR_REPORT_PERIOD_S * 1000);

    while (true) {
        // Clear screen and move cursor to home
        printf("\033[2J\033[H");
        
        printf("========================================\n");
        printf("      ESP32 FreeRTOS Task Monitor       \n");
        printf("========================================\n");

        monitorReport();
        monitorPollReports();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ===================================================================
// app_main — init, SYNC, create FreeRTOS tasks
// ===================================================================

void app_main(void)
{
    monitorInit();
    gpio_init();
    pcnt_init();

    g_sync_sem      = xSemaphoreCreateBinary();
    g_sporadic_sem = xSemaphoreCreateCounting(32, 0);
    g_token_mutex  = xSemaphoreCreateMutex();
    if ((g_sync_sem == NULL) || (g_sporadic_sem == NULL) || (g_token_mutex == NULL)) {
        ESP_LOGE(TAG, "Failed to create RTOS synchronization objects");
        vTaskSuspend(NULL);
    }

    ESP_LOGI(TAG, "Waiting for SYNC");
    const int64_t t0_us = wait_for_sync_rising_edge();
    reset_schedule_state();
    synch();
    monitorSetPeriodicReportEverySeconds(0);
    monitorSetFinalReportAfterSeconds(0);

    g_sync_tick = xTaskGetTickCount();
    __atomic_store_n(&g_schedule_started, true, __ATOMIC_RELEASE);

    ESP_LOGI(TAG, "Assignment 3 FreeRTOS started at T0=%" PRIi64 " us", t0_us);

    // Task priorities — Rate Monotonic: shorter period = higher priority.
    // Task S at priority 5 so it can preempt C/D but not A.
    #define PRIO_A   6
    #define PRIO_B   5
    #define PRIO_AGG 4
    #define PRIO_S   5
    #define PRIO_C   3
    #define PRIO_D   3
    #define PRIO_MON 1

    BaseType_t rc = pdPASS;
    rc = xTaskCreatePinnedToCore(task_a_func,   "A",   4096, NULL, PRIO_A,   NULL, 0);
    if (rc != pdPASS) { ESP_LOGE(TAG, "Failed to create task A"); vTaskSuspend(NULL); }
    rc = xTaskCreatePinnedToCore(task_b_func,   "B",   4096, NULL, PRIO_B,   NULL, 0);
    if (rc != pdPASS) { ESP_LOGE(TAG, "Failed to create task B"); vTaskSuspend(NULL); }
    rc = xTaskCreatePinnedToCore(task_agg_func, "AGG", 4096, NULL, PRIO_AGG, NULL, 0);
    if (rc != pdPASS) { ESP_LOGE(TAG, "Failed to create task AGG"); vTaskSuspend(NULL); }
    rc = xTaskCreatePinnedToCore(task_c_func,   "C",   4096, NULL, PRIO_C,   NULL, 0);
    if (rc != pdPASS) { ESP_LOGE(TAG, "Failed to create task C"); vTaskSuspend(NULL); }
    rc = xTaskCreatePinnedToCore(task_d_func,   "D",   4096, NULL, PRIO_D,   NULL, 0);
    if (rc != pdPASS) { ESP_LOGE(TAG, "Failed to create task D"); vTaskSuspend(NULL); }
    rc = xTaskCreatePinnedToCore(task_s_func,   "S",   4096, NULL, PRIO_S,   NULL, 0);
    if (rc != pdPASS) { ESP_LOGE(TAG, "Failed to create task S"); vTaskSuspend(NULL); }
    rc = xTaskCreatePinnedToCore(monitor_task_func, "MON", 3072, NULL, PRIO_MON, NULL, 0);
    if (rc != pdPASS) { ESP_LOGE(TAG, "Failed to create monitor task"); vTaskSuspend(NULL); }

    // app_main task is no longer needed
    vTaskDelete(NULL);
}
