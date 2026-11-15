/*
 * SPDX-FileCopyrightText: 2026 Thomas Grill
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Media clock: locks the I2S sample clock (APLL) to PTP.
 *
 * Without it, I2S runs on its crystal while RTP timestamps and the packet
 * timer follow PTP, so the capture and playback buffers drift towards one
 * end (issue #3). The servo measures, every 100 ms, the frames I2S has
 * clocked (DMA completion stamps) against PTP time, and holds their
 * difference at the value it had when it started, by trimming the APLL
 * (aes67_apll.c): frequency and phase locked, buffer fill levels constant.
 *
 * Measured on an ESP32-P4-NANO following a Dante grandmaster: the crystal
 * was -24.5 ppm off, the servo settles in ~30 s at +24.4 ppm and holds the
 * phase within +-0.1 frame.
 *
 * Loop: e = PTP-expected frames - clocked frames since the reference.
 *   PI, once a second: ppm = kp*e + integral(ki*e), clamped. Plant:
 *   1 ppm = 0.048 frames/s at 48 kHz; kp 5 ppm/frame gives a crossover of
 *   ~0.24 rad/s, ki 0.5 ppm/frame/s a zero at 0.1 rad/s (phase margin ~65 deg).
 *   ppm -> APLL steps with error diffusion at 10 Hz (steps are ~1.5 ppm).
 *   A jump of more than 1 ms in e (PTP step, new grandmaster) takes a new
 *   reference; the frequency (integral) is kept. While PTP is not locked
 *   the trim is held.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "aes67_ptp.h"
#include "aes67_audio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool running;
    bool locked;                /* PTP locked and a reference taken: servoing */
    float phase_frames;         /* e: PTP-expected minus clocked frames (0 = locked in phase) */
    float trim_ppm;             /* servo output: the APLL correction, + = faster */
    int32_t trim_steps;         /* APLL steps currently written */
    float ppm_per_step;
    uint32_t references;        /* references taken (start, PTP steps) */
} aes67_media_clock_status_t;

/* Start the servo task. audio must be running with I2S on the APLL. */
esp_err_t aes67_media_clock_start(aes67_ptp_handle_t ptp, aes67_audio_handle_t audio,
                                  uint32_t sample_rate);
void aes67_media_clock_stop(void);
void aes67_media_clock_get_status(aes67_media_clock_status_t *st);

#ifdef __cplusplus
}
#endif
