#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_system.h"
#include "clock.h"
#include "ld2420_comm.h"


/*
* Exported variables
*/

/* Motion detection condition is defined in the software as Distance < Threshold. */
volatile int motionDetected = 0;

volatile uint32_t ld2420_Distance = 0;
volatile int32_t motionThreshold = 200;

volatile int32_t calibrationTime = 0;


/*
* Hardware constants
*/
#define TXD_PIN (GPIO_NUM_10)
#define RXD_PIN (GPIO_NUM_25)

#define NSAMPLES 1

#define MAX_TX_BUF_WORDS 32

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 256

#define NGATE 16

/*
* Static constants
*/
static const char *RX_TASK_TAG = "RX_TASK";


static const uint16_t gate_move_threshold_regname[NGATE] = 
 { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
   0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
  };

static const uint16_t gate_still_threshold_regname[NGATE] = 
 { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
   0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
  };


static const uint16_t system_mode_command = 0x00;
static const uint32_t system_energy_output = 0x04;
static const uint32_t system_basic_output = 0x64;

static const uint16_t moving_detector_timeout_param = 0x04;
static const uint32_t moving_detector_timeout_value = 1; // second

/*
* Forward declarations
*/

static void basic_handler(char *line, size_t linelen);
static void energy_handler(char *line, size_t linelen);
static void command_handler(char *line, size_t linelen);


/*
* Static variables
*/

/* Line processing */
static char handler_line[RX_BUF_SIZE];
static size_t line_length = 0;
static int current_frame_type = -1;
static uint32_t current_test_pattern = 0;
static int last_command_status = 0;
static void (*pending_command)(void) = NULL;

/* Range value */
static uint32_t average_value = 0;
static unsigned measurement_count = 0;

static struct frame_descriptors {
    uint32_t frame_header;
    uint32_t frame_footer;
    uint32_t footer_mask;
    void (* handler)(char *line, size_t linelen);
} frame_descriptors[] = {
    {
    /* Normal output: Range dd */
        .frame_header = ('n' << 24) + ('g' << 16) + ('e' << 8) + ' ',
        .frame_footer = '\r',
        .footer_mask = 0xff,
        .handler = basic_handler,
    },

    {
    /* No motion detected: OFF */
        .frame_header = ('O' << 24) + ('F' << 16) + ('F' << 8) + '\r',
        .frame_footer = 0,
        .footer_mask = 0,
        .handler = basic_handler,
    },

    {
    /* Command header: 0xFDFCFBFA */
        .frame_header = 0xFDFCFBFA,
        .frame_footer = 0x04030201,
        .footer_mask = 0xffffffff,
        .handler = command_handler,
    },

    {
    /* Energy output: 0xF4F3F2F1 */
        .frame_header = 0xF4F3F2F1,
        .frame_footer = 0xF8F7F6F5,
        .footer_mask = 0xffffffff,
        .handler = energy_handler,
    },

 };

/* Gate calibration data */
static uint16_t gate_peak_energy[NGATE] = { 0 };
static uint32_t gate_thresholds[NGATE] = { 0 };
static char in_calibration = 0;


/* Timer and time watch data */

static gptimer_handle_t timer = 0;
static const uint32_t timer_resolution = 10 * 1000;

#define clock(...) gpclock(timer)

/**********
*  UART interface functions
**********/

static void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


static size_t uart_get_chars(char *buffer, int maxsize)
{
    size_t size = 0;
    uart_get_buffered_data_len(UART_NUM_1, &size);

    if (size > maxsize)
        size = maxsize;

    if (size > 0) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, buffer, size, 100 / portTICK_PERIOD_MS);
        if (rxBytes < 0)
            size = 0;
        else if ((size_t)rxBytes < size)
            size = (size_t)rxBytes;
    } else {
/* Pause to allow some data arrive to the buffer */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return size;
}


/*
* Read the stream, detect frames, call handlers.
*/
static int find_frame_header(uint32_t pattern)
{
    size_t i;
    for (i = 0; i < sizeof(frame_descriptors); i++) {
        if (frame_descriptors[i].frame_header == pattern) {
            return (int)i;
        }
    }
    return -1;
}

static void uart_process(void)
{
    static char local_buf[16];

    size_t rx_size = uart_get_chars(&local_buf[0], sizeof(local_buf));

    size_t i;
    for (i = 0; i < rx_size; i++) {
        char ch = local_buf[i];

        current_test_pattern = (current_test_pattern << 8) | ch;
        if (line_length < sizeof(handler_line)) {
            handler_line[line_length] = ch;
            line_length += 1;
        } else {
        /* Line buffer overflow -> restart frame detection. */
            local_buf[sizeof(local_buf)-1] = 0;
            current_frame_type = -1;
            line_length = 0;
        }

        if (current_frame_type < 0) {
            current_frame_type = find_frame_header(current_test_pattern);
            if (current_frame_type >= 0) {
                current_test_pattern = 0;
                line_length = 0;
            }
        }

        if (current_frame_type >= 0 &&
            (current_test_pattern & frame_descriptors[current_frame_type].footer_mask) == frame_descriptors[current_frame_type].frame_footer) {
        /* Frame end detected -> call the handler and reset */
            frame_descriptors[current_frame_type].handler(handler_line, line_length);
            current_test_pattern = 0;
            line_length = 0;
            current_frame_type = -1;
        }

    }
}

/*
* Set motion flag based on the range threshold rather than use sensor's flag.
*/
static void apply_motion_threshold(unsigned range)
{
    int mdetect = 0;

    ld2420_Distance = range;

    mdetect = (range > 11 && range < motionThreshold);

    if (mdetect != motionDetected)
        ESP_LOGI(RX_TASK_TAG, "Motion detected: %s", mdetect ? "On" : "Off");

    motionDetected = mdetect;
}

static void basic_handler(char *line, size_t linelen)
{
    if (linelen == 0) {
/* No data in the OFF line */
        if (motionDetected)
            ESP_LOGI(RX_TASK_TAG, "Motion: OFF");
        motionDetected = 0;
        ld2420_Distance = 0;
        return;
    }

    unsigned range_value = 0;

    size_t i;
    for (i = 0; i < linelen; i++) {
        char ch = line[i];

        if (ch < '0' || ch > '9') {

            ESP_LOGI(RX_TASK_TAG, "Distance: %u", range_value);
    /* Finish reading */
	    if (range_value < 6)
		range_value = 0;

	    average_value += range_value;
	    measurement_count += 1;

	    if (NSAMPLES == measurement_count) {
	        average_value /= NSAMPLES;
                ld2420_Distance = average_value;

                apply_motion_threshold(average_value);

	        average_value = 0;
	        measurement_count = 0;
	    }
	    range_value = 0;
            break;

        } else {
    /* Next digit */
            range_value = (range_value * 10) + (ch - '0');
        }
    }
}

static void energy_handler(char *line, size_t linelen)
{

    size_t framelen = line[0] + (line[1] << 8);

/* Check for incomplete frame */
    if (framelen > linelen - 4)
        return;


    int moving = line[2];

    if (0 == moving) {
        motionDetected = 0;
    } else {
/* Use detected range to detect movement */
        int range_value = line[3] + (line[4] << 8);

        average_value += range_value;
        measurement_count += 1;

        if (NSAMPLES == measurement_count) {
            average_value /= NSAMPLES;
            ld2420_Distance = average_value;

            apply_motion_threshold(average_value);

            average_value = 0;
            measurement_count = 0;

        }
    }


/* Read gate energies and update the peak values */
    size_t n;
    size_t ngate = NGATE;
    if (ngate * 2 + 3 > framelen) {
        ngate = (framelen - 3) / 2;
    }

    for (n = 0; n < ngate; n++) {
        unsigned gate_val = line[n * 2 + 5] + (line[n * 2 + 6] << 8);
        if (gate_val > gate_peak_energy[n])
            gate_peak_energy[n] = gate_val;
    }

    ESP_LOGD(RX_TASK_TAG, "M=%d D=%lu Energies: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
        moving, average_value,
        gate_peak_energy[0], gate_peak_energy[1], gate_peak_energy[2], gate_peak_energy[3],
        gate_peak_energy[4], gate_peak_energy[5], gate_peak_energy[6], gate_peak_energy[7],
        gate_peak_energy[8], gate_peak_energy[9], gate_peak_energy[10], gate_peak_energy[11],
        gate_peak_energy[12], gate_peak_energy[13], gate_peak_energy[14], gate_peak_energy[15] );

}

static void command_handler(char *line, size_t linelen)
{
    size_t framelen = line[0] + (line[1] << 8);

/* Check for incomplete frame */
    if (framelen > linelen - 4)
        return;

    last_command_status = line[4] + (line[5] << 8);
    ESP_LOGI(RX_TASK_TAG, "Return status: %d", last_command_status);

    if (0 == last_command_status) {
        if (pending_command) {
            pending_command();
            pending_command = NULL;
        }
    }
}

static void write_parameters(const uint16_t command, const uint16_t *param_name, const uint32_t *param_value, unsigned count)
{
    uint32_t frame_header = 0xFAFBFCFD;
    uint32_t frame_footer = 0x01020304;

    uint16_t frame_len = count * 6 + 2;

    uart_write_bytes(UART_NUM_1, &frame_header, sizeof(frame_header));
    uart_write_bytes(UART_NUM_1, &frame_len, sizeof(frame_len));
    uart_write_bytes(UART_NUM_1, &command, sizeof(command));

    unsigned i;
    for (i = 0; i < count; i++) {
        uart_write_bytes(UART_NUM_1, &param_name[i], sizeof(param_name[i]));
        uart_write_bytes(UART_NUM_1, &param_value[i], sizeof(param_value[i]));
    }
    uart_write_bytes(UART_NUM_1, &frame_footer, sizeof(frame_footer));
}

static void write_abd_parameters(const uint16_t *param_name, const uint32_t *param_value, unsigned count)
{
    uint16_t command = 0x07;
    write_parameters(command, param_name, param_value, count);
}

static void write_system_parameter(const uint16_t param_name, const uint32_t param_value)
{
    uint16_t command = 0x12;
    write_parameters(command, &param_name, &param_value, 1);
}

/*
* Calibration routines
*/
static void begin_calibration(void)
{
    if (in_calibration)
        return;

    ESP_LOGI(RX_TASK_TAG, "Calibration is started: %ld", calibrationTime);

    in_calibration = 1;

    unsigned i;
    for (i = 0; i < NGATE; i++)
        gate_peak_energy[i] = 0;

    pending_command = NULL;
    write_system_parameter(system_mode_command, system_energy_output);
}

static void update_gate_thresholds(void)
{
    unsigned i;
    unsigned cnt = 0;
    for (i = 0; i < NGATE; i++) {
        uint32_t gate_peak = gate_peak_energy[i];
        gate_peak = ((gate_peak * 3 + 50) / 100) * 100;
        if (gate_peak > 65535) gate_peak = 65535;
        gate_thresholds[i] = (uint16_t)gate_peak;
        cnt += 1;
/* Write gate thresholds in 4 pieces to avoid sensor's buffer overflow (64 bytes) */
        if (cnt == NGATE/4) {
            write_abd_parameters(&gate_move_threshold_regname[cnt - NGATE/4], &gate_thresholds[cnt - NGATE/4], cnt);
            vTaskDelay(pdMS_TO_TICKS(20));
            cnt = 0;
        }
    }

    ESP_LOGI(RX_TASK_TAG, "Moving thresholds: %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
        gate_thresholds[0], gate_thresholds[1], gate_thresholds[2], gate_thresholds[3],
        gate_thresholds[4], gate_thresholds[5], gate_thresholds[6], gate_thresholds[7],
        gate_thresholds[8], gate_thresholds[9], gate_thresholds[10], gate_thresholds[11],
        gate_thresholds[12], gate_thresholds[13], gate_thresholds[14], gate_thresholds[15] );

    cnt = 0;
    for (i = 0; i < NGATE; i++) {
        uint32_t gate_peak = gate_peak_energy[i];
        gate_peak = ((gate_peak + gate_peak / 2 + 50) / 100) * 100;
        if (gate_peak > 65535) gate_peak = 65535;
        gate_thresholds[i] = (uint16_t)gate_peak;

        cnt += 1;
/* Write gate thresholds in 2 pieces to avoid sensor's buffer overflow (64 bytes) */
        if (cnt == NGATE/4) {
            write_abd_parameters(&gate_still_threshold_regname[cnt - NGATE/4], &gate_thresholds[cnt - NGATE/4], cnt);
            vTaskDelay(pdMS_TO_TICKS(20));
            cnt = 0;
        }
    }

    ESP_LOGI(RX_TASK_TAG, "Still thresholds: %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
        gate_thresholds[0], gate_thresholds[1], gate_thresholds[2], gate_thresholds[3],
        gate_thresholds[4], gate_thresholds[5], gate_thresholds[6], gate_thresholds[7],
        gate_thresholds[8], gate_thresholds[9], gate_thresholds[10], gate_thresholds[11],
        gate_thresholds[12], gate_thresholds[13], gate_thresholds[14], gate_thresholds[15] );

/* Update detector timeout */
    write_abd_parameters(&moving_detector_timeout_param, &moving_detector_timeout_value, 1);
}

static void end_calibration(void)
{
    if (!in_calibration)
        return;

    ESP_LOGI(RX_TASK_TAG, "Calibration has ended.");
    in_calibration = 0;

/* Delay gate update until after the previous command is finished */
    pending_command = update_gate_thresholds;

    write_system_parameter(system_mode_command, system_basic_output);
}

void ld2420_task(void *arg)
{
    init_uart();

/* This delay is needed for UART to become active */
    vTaskDelay(pdMS_TO_TICKS(2000));

    init_timer(&timer, timer_resolution);

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint64_t clock_now = 0, clock_prev = 0;
    uint64_t cal_timer = 0;

    while (1) {
        uart_process();

        clock_now = clock();
        uint64_t delta = clock_now - clock_prev;
        clock_prev = clock_now;

        if (in_calibration) {
            if (calibrationTime > 0) {
                cal_timer += delta;
                if (cal_timer >= timer_resolution) {
                    calibrationTime -= 1;
                    cal_timer -= timer_resolution;
                    ESP_LOGI(RX_TASK_TAG, "Calibration time left: %ld", calibrationTime);
                }
            }

            if (calibrationTime == 0) {
                end_calibration();
            }

        } else {
            if (calibrationTime > 0) {
                begin_calibration();
            }
        }


    }
}
