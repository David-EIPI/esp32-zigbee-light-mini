/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_zigbee_core.h"
#include "light_driver.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */


#define HA_FIRST_ENDPOINT     1
#define HA_DISTANCE_SENSOR_ENDPOINT   HA_FIRST_ENDPOINT
#define HA_CALIBRATION_ENDPOINT       HA_FIRST_ENDPOINT + 1
#define HA_ON_OFF_DIMMER_ENDPOINT     HA_FIRST_ENDPOINT
#define HA_MULTISTATE_ENDPOINT        HA_FIRST_ENDPOINT+1
#define HA_LIGHT_ON_BINARY_ENDPOINT   HA_FIRST_ENDPOINT
#define HA_MOTION_BINARY_ENDPOINT     HA_FIRST_ENDPOINT+1
#define HA_TEMPERATURE_ENDPOINT       HA_FIRST_ENDPOINT


#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */

/* Basic manufacturer information */
#define MANUFACTURER_NAME "\x02" "DS"      /* Customized manufacturer name */
#define MODEL_IDENTIFIER "\x06" "Model2" /* Customized model identifier */

#define DISTANCE_SENSOR_DESC  "\x08" "Distance"
#define CALIBRATION_DESC  "\x0b" "Calibration"

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,      \
    }

#define lengthof(a) (sizeof(a)/sizeof(a[0]))

#define STRINGIFY(s) STRINGIFY_HELPER(s)
#define STRINGIFY_HELPER(s) #s
