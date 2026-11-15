/*
 * SPDX-FileCopyrightText: 2026 Thomas Grill
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * APLL fine trim (ESP32-P4): steers the audio clock in steps of the APLL's
 * sigma-delta modulator, for the media-clock servo (aes67_media_clock.c).
 *
 * f_mult = 40 MHz * (4 + sdm2 + sdm1/256 + sdm0/65536); the I2S MCLK divides
 * it. One sdm0 step changes the multiplier by 1/(4*65536 + sdm) of itself,
 * ~1.5 ppm at 393.216 MHz. Only the changed DSDM registers are written; no
 * recalibration, no divider change, so the clock keeps running (no clicks
 * heard on headphones at 10 writes/s).
 *
 * Private IDF API (hal/clk_tree_ll.h, REGI2C), checked against ESP-IDF v5.5.
 * Everything clocked from the APLL moves together.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool ready;
    uint32_t o_div, sdm0, sdm1, sdm2;    /* current register values */
    int32_t trim;                        /* steps from the value I2S set up */
    float ppm_per_step;
    uint32_t writes;
} aes67_apll_status_t;

/* Read the APLL coefficients the I2S driver set up; the trim is relative to
 * them. Call once I2S runs on the APLL. Repeated calls keep the first base. */
esp_err_t aes67_apll_init(void);

/* Set the trim to `steps` sdm0 LSBs from the base (|steps| <= 600, ~900 ppm).
 * Positive = faster. */
esp_err_t aes67_apll_set_steps(int32_t steps);

void aes67_apll_get_status(aes67_apll_status_t *st);

#ifdef __cplusplus
}
#endif
