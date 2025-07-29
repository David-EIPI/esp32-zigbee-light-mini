#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_cap.h"
#include "triac_dimmer.h"

#define TRIAC_CONTROL_PIN (GPIO_NUM_5)
#define AC_SYNC_PIN       (GPIO_NUM_11)


volatile int32_t dimLevel = DIM_LEVEL_MAX / 2;
volatile int32_t phaseCorrection_us = -300;
volatile int32_t transitionTime = 0;
volatile int stopDimmer = 1;

static uint32_t ac_wave_period = 0;
static uint32_t ac_pulse_width = 0;

static uint32_t cap_timer_resolution = 0;

/* MCPWM timer is 16 bit, and at 6MHz it can provide up to 1/90 s PWM cycle length.
 With AC 60Hz we need PWM cycle length 1/120 s.
 Also 6MHz is an integer divider for APB clock of 48HZ */
static const uint32_t mcpwm_timer_resolution = 6 * 1000 * 1000;
static const uint32_t mcpwm_60Hz_half_period = mcpwm_timer_resolution / 60 / 2;


const static char *CTAG = "capture";
const static char *GTAG = "generator";

/* Capture timer */

static bool ac_wave_detect_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t prev_pos_edge = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        ac_wave_period = edata->cap_value - prev_pos_edge;

        /* store the timestamp when pos edge is detected */
        prev_pos_edge = edata->cap_value;
    } else {
       /* pulse length that marks the top of the sine wave */
        ac_pulse_width = edata->cap_value - prev_pos_edge;
        /* Notify the task about the detected pulse to update the dimmer settings */
        xTaskNotifyFromISR(task_to_notify, ac_pulse_width, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

void setup_capture(void)
{

    ESP_LOGI(CTAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(CTAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = AC_SYNC_PIN,
        .prescale = 1,
        // capture on both edges
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(CTAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = ac_wave_detect_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(CTAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(CTAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TRIAC_CONTROL_PIN,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(TRIAC_CONTROL_PIN, 0));

    ESP_LOGI(CTAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(cap_timer, &cap_timer_resolution));
}

/* Setup MCPWM Generator  */

static mcpwm_timer_handle_t gen_timer = NULL;
static mcpwm_sync_handle_t gpio_sync_source = NULL;
static mcpwm_cmpr_handle_t comparator = NULL;

static void set_pwm_timer_phase(uint32_t phase_ticks)
{
    mcpwm_timer_sync_phase_config_t sync_phase_config = {
        .count_value = phase_ticks,
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .sync_src = gpio_sync_source,
    };

    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(gen_timer, &sync_phase_config));
}

static void setup_generator(void)
{
    ESP_LOGI(GTAG, "Create generator timer");
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = mcpwm_timer_resolution,
        .period_ticks = mcpwm_60Hz_half_period, /* Preliminary setting, will be adjusted from the capture timer */
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &gen_timer));

    ESP_LOGI(GTAG, "Create operator");
    mcpwm_oper_handle_t gen_operator;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator should be in the same group of the above timers
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &gen_operator));


    ESP_LOGI(GTAG, "Connect timer and operator with each other");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(gen_operator, gen_timer));

    ESP_LOGI(GTAG, "Create comparator");
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(gen_operator, &compare_config, &comparator));
        // init compare for each comparator
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, timer_config.period_ticks)); // start with 100% duty

    ESP_LOGI(GTAG, "Create generators");
    mcpwm_gen_handle_t generator;
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = TRIAC_CONTROL_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(gen_operator, &gen_config, &generator));

    ESP_LOGI(GTAG, "Set generator actions on timer and compare event");

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        // when the timer value is zero, and is counting up, set output to high
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        // when compare event happens, and timer is counting up, set output to low
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));


    ESP_LOGI(GTAG, "Start generator timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(gen_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(gen_timer, MCPWM_TIMER_START_NO_STOP));


    ESP_LOGI(GTAG, "Create GPIO sync source");
    mcpwm_gpio_sync_src_config_t gpio_sync_config = {
        .group_id = 0,  // GPIO fault should be in the same group of the above timers
        .gpio_num = AC_SYNC_PIN,
        .flags.pull_up = true,
        .flags.active_neg = false,  // by default, a posedge pulse can trigger a sync event
    };

    ESP_ERROR_CHECK(mcpwm_new_gpio_sync_src(&gpio_sync_config, &gpio_sync_source));


    ESP_LOGI(GTAG, "Set timers to sync on the GPIO");
    set_pwm_timer_phase(0);
}

#if 0
static void test_output(uint32_t d)
{
    gpio_set_level(TRIAC_CONTROL_PIN, 1); // set high
    esp_rom_delay_us(d);
    gpio_set_level(TRIAC_CONTROL_PIN, 0); // set low
}
#endif

#if 0
static void init_test_gpio(void)
{
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << AC_SYNC_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };

    gpio_config(&io_conf);
}
#endif


void triac_dimmer_task(void *arg)
{
    esp_log_level_set(GTAG, ESP_LOG_INFO);
    esp_log_level_set(CTAG, ESP_LOG_INFO);

//    init_test_gpio();
    uint32_t current_dim_level = 0;
    uint32_t accum_duty_timer = 0;

    setup_capture();
    if (!cap_timer_resolution) {
        return;
    }

    setup_generator();

    ESP_LOGI(CTAG, "Capture timer resolution: %lu", cap_timer_resolution);


    unsigned cap_pwm_ratio = cap_timer_resolution / mcpwm_timer_resolution;

    uint32_t pulse_len = 0;
    uint32_t dbg_cnt = 0;
    stopDimmer = 0;

    while (!stopDimmer) {
        // wait for echo done signal

        if (xTaskNotifyWait(0x00, ULONG_MAX, &pulse_len, pdMS_TO_TICKS(1000)) == pdTRUE) {


/* Calculate PWM timer settings using the captured sine wave period */
            uint32_t pwm_width = ac_pulse_width / 2 / cap_pwm_ratio;
            uint32_t pwm_period = ac_wave_period / 2 / cap_pwm_ratio;

            if (current_dim_level != dimLevel) {
                accum_duty_timer += pwm_period * 2;

                uint32_t transition_step = transitionTime * mcpwm_timer_resolution / DIM_LEVEL_MAX / TRANSITION_UNITS;

                while (current_dim_level != dimLevel && accum_duty_timer >= transition_step) {

                    accum_duty_timer -= transition_step;

                    if (current_dim_level < dimLevel)
                        current_dim_level += 1;

                    if (current_dim_level > dimLevel)
                        current_dim_level -= 1;
                }
            }

//            uint32_t pwm_duty = ((uint32_t)dimLevel * pwm_period + DIM_LEVEL_MAX / 2) / DIM_LEVEL_MAX;
            uint32_t pwm_duty = (current_dim_level * pwm_period + DIM_LEVEL_MAX / 2) / DIM_LEVEL_MAX;
            uint32_t pwm_phase = pwm_period / 2 - pwm_width + pwm_duty;
            int32_t phase_correction = phaseCorrection_us * (mcpwm_timer_resolution / 1000000);

/* Validate the timer parameters */
            if (phase_correction > 0)
                pwm_phase += (uint32_t)phase_correction;
            else
                pwm_phase -= (uint32_t)(-phase_correction);

            if (pwm_period >= (1<<16)                            /* MCPWM generator timer is 16-bit, avoid overflow */
               || pwm_period < mcpwm_60Hz_half_period / 2        /* Check that captured period is not too short, possibly due to incorrectly detected edge */
               || pwm_width > pwm_period / 2)                    /* Check that captured pulse width is not too long, possibly due to lost edge */
            {
// out of range
//                ESP_LOGI(CTAG, "OOR:period %lu, width: %lu, phase: %lu", pwm_period, pwm_duty, pwm_phase);
//                ESP_LOGI(CTAG, "OOR:period %lu, width: %lu, phase: %lu", pwm_period, ac_pulse_width / 2 / cap_pwm_ratio, pwm_phase);
                continue;
            }

            if (pwm_phase >= pwm_period) { /* If the computed phase is longer than the period, wrap around */
                pwm_phase = pwm_phase % pwm_period;
//                ESP_LOGI(CTAG, "Period %lu, width: %lu, OOR:phase: %lu pulse: %lu", pwm_period, pwm_duty, pwm_phase, pulse_len / cap_pwm_ratio);
            }


            if (pwm_duty > pwm_period) /* Duty can not exceed period */
                pwm_duty = pwm_period;

#if 0
            dbg_cnt += 1;
            if (dbg_cnt == 120 || pwm_period < 20000 || pwm_period > 60000) {
//                ESP_LOGI(CTAG, "Period %lu, width: %lu, phase: %lu", pwm_period, pwm_duty, pwm_phase);
                ESP_LOGI(CTAG, "Period %lu, width: %lu, phase: %lu", pwm_period, ac_pulse_width / 2 / cap_pwm_ratio, pwm_phase);
                dbg_cnt = 0;
            }
#endif

/* Set the generator timer parameters */
            ESP_ERROR_CHECK(mcpwm_timer_set_period(gen_timer, pwm_period));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pwm_duty));
            set_pwm_timer_phase(pwm_phase);

        } else {

/* Fallback method if pulse input is lost - provide a basic on-off functionality */
            if (0 == dimLevel) {
                mcpwm_comparator_set_compare_value(comparator, 0);
            } else if (DIM_LEVEL_MAX == dimLevel) {
                mcpwm_timer_set_period(gen_timer, mcpwm_timer_resolution / 120);
                mcpwm_comparator_set_compare_value(comparator, mcpwm_timer_resolution / 120);
            }
        }

/* Short break to feed the watchdog */
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (gpio_sync_source)
        mcpwm_del_sync_src(gpio_sync_source);

    if (gen_timer)
        mcpwm_timer_disable(gen_timer);

}

