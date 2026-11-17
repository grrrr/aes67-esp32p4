/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include "esp_eth_time.h"

/* Renamed from esp_eth_clock_* (the ESP-IDF ethernet/ptp example) to
 * aes67_eth_clock_*: ESP-IDF 6.1 ships its own esp_eth_clock with the same
 * function names. IDF 6 removed the PTP ioctls in favour of functions on the
 * MAC instance; IDF 5 only has the ioctls. Both paths do the same thing. */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define AES67_ETH_MAC_API 1
#include "esp_eth_mac_esp.h"
static esp_eth_mac_t *s_mac;
#endif

static esp_eth_handle_t s_eth_hndl;

static int aes67_eth_clock_esp_err_to_errno(esp_err_t esp_err)
{
    switch (esp_err)
    {
    case ESP_ERR_INVALID_ARG:
        return EINVAL;
    case ESP_ERR_INVALID_STATE:
        return EBUSY;
    case ESP_ERR_TIMEOUT:
        return ETIME;
    }
    // default "no err" when error cannot be isolated
    return 0;
}

int aes67_eth_clock_adjtime(clockid_t clk_id, aes67_eth_clock_adj_param_t *adj)
{
    switch (clk_id) {
    case CLOCK_PTP_SYSTEM:
        if (adj->mode == AES67_ETH_CLK_ADJ_FREQ_SCALE) {
#if AES67_ETH_MAC_API
            esp_err_t ret = esp_eth_mac_adj_ptp_freq(s_mac, adj->freq_scale);
#else
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_ADJ_PTP_FREQ, &adj->freq_scale);
#endif
            if (ret != ESP_OK) {
                errno = aes67_eth_clock_esp_err_to_errno(ret);
                return -1;
            }
        } else {
            errno = EINVAL;
            return -1;
        }
        break;
    default:
        errno = EINVAL;
        return -1;
    }
    return 0;
}

int aes67_eth_clock_settime(clockid_t clock_id, const struct timespec *tp)
{
    switch (clock_id) {
    case CLOCK_PTP_SYSTEM: {
        if (s_eth_hndl) {
            eth_mac_time_t ptp_time = {
                .seconds = tp->tv_sec,
                .nanoseconds = tp->tv_nsec
            };
#if AES67_ETH_MAC_API
            esp_err_t ret = esp_eth_mac_set_ptp_time(s_mac, &ptp_time);
#else
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_S_PTP_TIME, &ptp_time);
#endif
            if (ret != ESP_OK) {
                errno = aes67_eth_clock_esp_err_to_errno(ret);
                return -1;
            }
        } else {
            errno = ENODEV;
            return -1;
        }
        break;
    }
    default:
        errno = EINVAL;
        return -1;
    }
    return 0;
}

int aes67_eth_clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    switch (clock_id) {
    case CLOCK_PTP_SYSTEM: {
        if (s_eth_hndl) {
            eth_mac_time_t ptp_time;
#if AES67_ETH_MAC_API
            esp_err_t ret = esp_eth_mac_get_ptp_time(s_mac, &ptp_time);
#else
            esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_G_PTP_TIME, &ptp_time);
#endif
            if (ret != ESP_OK) {
                errno = aes67_eth_clock_esp_err_to_errno(ret);
                return -1;
            }
            tp->tv_sec = ptp_time.seconds;
            tp->tv_nsec = ptp_time.nanoseconds;
        } else {
            errno = ENODEV;
            return -1;
        }
        break;
    }
    default:
        errno = EINVAL;
        return -1;
    }
    return 0;
}

int aes67_eth_clock_set_target_time(clockid_t clock_id, struct timespec *tp)
{
    eth_mac_time_t mac_target_time = {
        .seconds = tp->tv_sec,
        .nanoseconds = tp->tv_nsec
    };
#if AES67_ETH_MAC_API
    esp_err_t ret = esp_eth_mac_set_target_time(s_mac, &mac_target_time);
#else
    esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_S_TARGET_TIME, &mac_target_time);
#endif
    if (ret != ESP_OK) {
        errno = aes67_eth_clock_esp_err_to_errno(ret);
        return -1;
    }
    return 0;
}

int aes67_eth_clock_register_target_cb(clockid_t clock_id,
                                     ts_target_exceed_cb_from_isr_t ts_callback)
{
#if AES67_ETH_MAC_API
    esp_err_t ret = esp_eth_mac_set_target_time_cb(s_mac, ts_callback);
#else
    esp_err_t ret = esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_S_TARGET_CB, ts_callback);
#endif
    if (ret != ESP_OK) {
        errno = aes67_eth_clock_esp_err_to_errno(ret);
        return -1;
    }
    return 0;
}

esp_err_t aes67_eth_clock_adj_ppb(int32_t adj_ppb)
{
#if AES67_ETH_MAC_API
    return esp_eth_mac_adj_ptp_time(s_mac, adj_ppb);
#else
    return esp_eth_ioctl(s_eth_hndl, ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_ppb);
#endif
}

esp_err_t aes67_eth_clock_init(clockid_t clock_id, aes67_eth_clock_cfg_t *cfg)
{
    switch (clock_id) {
    case CLOCK_PTP_SYSTEM:
        // PTP Clock is part of Ethernet system
#if AES67_ETH_MAC_API
    {
        esp_eth_mac_t *mac;
        if (esp_eth_get_mac_instance(cfg->eth_hndl, &mac) != ESP_OK) {
            return ESP_FAIL;
        }
        eth_mac_ptp_config_t ptp_cfg = ETH_MAC_ESP_PTP_DEFAULT_CONFIG();
        /* the IDF 5 ioctl used digital rollover; IDF 6 defaults to binary */
        ptp_cfg.roll_type = ETH_PTP_DIGITAL_ROLLOVER;
        if (esp_eth_mac_ptp_enable(mac, &ptp_cfg) != ESP_OK) {
            return ESP_FAIL;
        }
        s_mac = mac;
    }
#else
        bool ptp_enable = true;
        if (esp_eth_ioctl(cfg->eth_hndl, ETH_MAC_ESP_CMD_PTP_ENABLE, &ptp_enable) != ESP_OK) {
            return ESP_FAIL;
        }
#endif
        s_eth_hndl = cfg->eth_hndl;
        break;
    default:
        return ESP_FAIL;
    }
    return ESP_OK;
}
