/*
 * SPDX-FileCopyrightText: 2026 Thomas Grill
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Media clock servo: I2S (APLL) locked to PTP. See aes67_media_clock.h. */
#include "aes67_media_clock.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "aes67_apll.h"

static const char *TAG = "aes67_mclk";

#define PERIOD_MS   100         /* measurement and trim update */
#define SERVO_EVERY 10          /* PI update every 10 periods (1 s) */
#define PTP_TRIES   4           /* PTP reads per sample; keep the tightest bracket */
#define KP          5.0f        /* ppm per frame */
#define KI          0.5f        /* ppm per frame, per second */

#ifndef CONFIG_AES67_MEDIA_CLOCK_MAX_PPM
#define CONFIG_AES67_MEDIA_CLOCK_MAX_PPM 200
#endif

static struct {
    aes67_ptp_handle_t ptp;
    aes67_audio_handle_t audio;
    double fs;
    TaskHandle_t task;
    volatile bool stop;
    /* reference: PTP time and clocked frames when the lock was taken */
    bool have_ref;
    uint64_t ref_ptp_ns;
    double ref_frames;
    /* servo */
    float integ, ppm;
    double dither;
    int32_t steps;
    aes67_media_clock_status_t st;
    portMUX_TYPE lock;
} m = { .lock = portMUX_INITIALIZER_UNLOCKED };

/* PI with anti-windup: positive error = we are slow -> speed up */
static float servo_update(float e)
{
    const float max = (float)CONFIG_AES67_MEDIA_CLOCK_MAX_PPM;
    const float p = KP * e;
    float integ = m.integ + KI * e;
    if (integ > max) integ = max;
    if (integ < -max) integ = -max;
    float out = p + integ;
    if (out > max) out = max;
    else if (out < -max) out = -max;
    else m.integ = integ;                   /* integrate only when not saturated */
    return out;
}

static void apply_trim(void)
{
    aes67_apll_status_t a;
    aes67_apll_get_status(&a);
    if (!a.ready || a.ppm_per_step <= 0) {
        return;
    }
    /* ppm -> steps, error diffusion between neighbouring steps */
    const double want = m.ppm / a.ppm_per_step + m.dither;
    const int32_t steps = (int32_t)floor(want + 0.5);
    m.dither = want - steps;
    if (steps != m.steps && aes67_apll_set_steps(steps) == ESP_OK) {
        m.steps = steps;
    }
}

static void mclk_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    for (unsigned n = 1; !m.stop; n++) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));

        aes67_ptp_status_t ps;
        if (aes67_ptp_get_status(m.ptp, &ps) != ESP_OK || ps.lock_state != AES67_PTP_LOCKED) {
            m.have_ref = false;                 /* hold the trim, new reference on relock */
            portENTER_CRITICAL(&m.lock);
            m.st.locked = false;
            portEXIT_CRITICAL(&m.lock);
            continue;
        }
        /* esp_timer -> PTP: bracket the (EMAC register) read, keep the tightest */
        int64_t best = INT64_MAX, t_mid = 0;
        uint64_t ptp_ns = 0;
        for (int i = 0; i < PTP_TRIES; i++) {
            uint64_t ns;
            const int64_t t0 = esp_timer_get_time();
            if (aes67_ptp_get_time_ns(m.ptp, &ns) != ESP_OK) {
                continue;
            }
            const int64_t t1 = esp_timer_get_time();
            if (t1 - t0 < best) {
                best = t1 - t0;
                t_mid = t0 + (t1 - t0) / 2;
                ptp_ns = ns;
            }
        }
        uint64_t st_frames;
        int64_t st_us;
        if (best == INT64_MAX || !aes67_audio_get_clock_stamp(m.audio, &st_frames, &st_us)) {
            continue;
        }
        /* frames at t_mid: the last DMA block, extrapolated by < 1 block at
         * the nominal rate (the rate error over ~1 ms is negligible) */
        const double frames = (double)st_frames + (double)(t_mid - st_us) * m.fs * 1e-6;

        if (!m.have_ref) {
            m.ref_ptp_ns = ptp_ns;
            m.ref_frames = frames;
            m.have_ref = true;
            portENTER_CRITICAL(&m.lock);
            m.st.references++;
            portEXIT_CRITICAL(&m.lock);
            ESP_LOGI(TAG, "reference taken: servoing I2S to PTP");
            continue;
        }
        /* e = PTP-expected frames - clocked frames since the reference */
        const double e = (double)(int64_t)(ptp_ns - m.ref_ptp_ns) * m.fs * 1e-9 - (frames - m.ref_frames);
        if (fabs(e) > m.fs * 0.001) {
            /* PTP step or new grandmaster: not drift. New reference, keep the frequency. */
            ESP_LOGW(TAG, "phase jumped by %.1f frames: new reference", e);
            m.have_ref = false;
            continue;
        }
        if (n % SERVO_EVERY == 0) {
            m.ppm = servo_update((float)e);
        }
        apply_trim();
        portENTER_CRITICAL(&m.lock);
        m.st.locked = true;
        m.st.phase_frames = (float)e;
        m.st.trim_ppm = m.ppm;
        m.st.trim_steps = m.steps;
        portEXIT_CRITICAL(&m.lock);
    }
    m.task = NULL;
    vTaskDelete(NULL);
}

esp_err_t aes67_media_clock_start(aes67_ptp_handle_t ptp, aes67_audio_handle_t audio,
                                  uint32_t sample_rate)
{
    if (!ptp || !audio || !sample_rate) {
        return ESP_ERR_INVALID_ARG;
    }
    if (m.task) {
        return ESP_OK;
    }
    esp_err_t err = aes67_apll_init();
    if (err != ESP_OK) {
        return err;
    }
    aes67_apll_status_t a;
    aes67_apll_get_status(&a);
    m.ptp = ptp;
    m.audio = audio;
    m.fs = (double)sample_rate;
    m.stop = false;
    m.have_ref = false;
    m.st.running = true;
    m.st.ppm_per_step = a.ppm_per_step;
    if (xTaskCreatePinnedToCore(mclk_task, "aes67_mclk", 3072, NULL, 8, &m.task, 0) != pdPASS) {
        m.st.running = false;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "media clock servo started (APLL, %.3f ppm/step)", (double)a.ppm_per_step);
    return ESP_OK;
}

void aes67_media_clock_stop(void)
{
    if (!m.task) {
        return;
    }
    m.stop = true;
    for (int i = 0; i < 20 && m.task; i++) {
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
    m.st.running = false;
    m.st.locked = false;
}

void aes67_media_clock_get_status(aes67_media_clock_status_t *st)
{
    portENTER_CRITICAL(&m.lock);
    *st = m.st;
    portEXIT_CRITICAL(&m.lock);
}
