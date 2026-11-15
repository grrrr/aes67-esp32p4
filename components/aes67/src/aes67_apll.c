/*
 * SPDX-FileCopyrightText: 2026 Thomas Grill
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* APLL fine trim, see aes67_apll.h. ESP32-P4. */
#include "aes67_apll.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_private/regi2c_ctrl.h"
#include "hal/clk_tree_ll.h"
#include "soc/regi2c_apll.h"

static const char *TAG = "aes67_apll";

#define MAX_STEPS 600           /* ~900 ppm: far beyond any crystal error */

static struct {
    aes67_apll_status_t st;
    uint32_t base;              /* sdm2:sdm1:sdm0 at init, 22 bits */
    portMUX_TYPE lock;
} a = { .lock = portMUX_INITIALIZER_UNLOCKED };

esp_err_t aes67_apll_init(void)
{
    if (a.st.ready) {
        return ESP_OK;                      /* keep the first base */
    }
    uint32_t o_div, s0, s1, s2;
    clk_ll_apll_get_config(&o_div, &s0, &s1, &s2);
    if (s0 == 0 && s1 == 0 && s2 == 0) {
        ESP_LOGW(TAG, "APLL not configured (is I2S on the APLL?)");
        return ESP_ERR_INVALID_STATE;
    }
    a.st.o_div = o_div;
    a.st.sdm0 = s0;
    a.st.sdm1 = s1;
    a.st.sdm2 = s2;
    a.base = (s2 << 16) | (s1 << 8) | s0;
    a.st.trim = 0;
    a.st.ppm_per_step = 1e6f / (float)(4 * 65536 + a.base);
    a.st.ready = true;
    const double mult = 40e6 * (4.0 + a.base / 65536.0);
    ESP_LOGI(TAG, "APLL o_div %lu sdm %lu/%lu/%lu: %.3f MHz -> %.6f MHz, %.3f ppm/step",
             (unsigned long)o_div, (unsigned long)s2, (unsigned long)s1, (unsigned long)s0,
             mult / 1e6, mult / ((o_div + 2) * 2) / 1e6, (double)a.st.ppm_per_step);
    return ESP_OK;
}

esp_err_t aes67_apll_set_steps(int32_t steps)
{
    if (!a.st.ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (abs(steps) > MAX_STEPS) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t v = a.base + steps;
    const uint32_t s0 = v & 0xFF, s1 = (v >> 8) & 0xFF, s2 = (v >> 16) & 0x3F;
    /* Only what changed. On a carry, sdm1 is written before sdm0: a brief
     * intermediate value of at most 256 steps' worth for a few microseconds,
     * averaged away by the PLL loop. */
    if (s2 != a.st.sdm2) REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM2, s2);
    if (s1 != a.st.sdm1) REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM1, s1);
    if (s0 != a.st.sdm0) REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM0, s0);
    portENTER_CRITICAL(&a.lock);
    a.st.sdm0 = s0;
    a.st.sdm1 = s1;
    a.st.sdm2 = s2;
    a.st.trim = steps;
    a.st.writes++;
    portEXIT_CRITICAL(&a.lock);
    return ESP_OK;
}

void aes67_apll_get_status(aes67_apll_status_t *st)
{
    portENTER_CRITICAL(&a.lock);
    *st = a.st;
    portEXIT_CRITICAL(&a.lock);
}
