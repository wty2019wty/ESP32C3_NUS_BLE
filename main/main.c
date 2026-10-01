/*
 * ESP32-C3 蓝牙串口调试器 —— BLE(NUS) <-> UART(TTL) 双向透传
 *
 *   网页 WebBLE  <--BLE-->  ESP32-C3  <--UART-->  被测设备(DUT)
 *
 * BLE 侧复用 ESP-IDF 官方 common/ble_uart 组件（事实标准 NUS 布局）：
 *   服务 6e400001-b5a3-f393-e0a9-e50e24dcca9e
 *   RX   6e400002-b5a3-f393-e0a9-e50e24dcca9e  (网页写入 -> 本机)
 *   TX   6e400003-b5a3-f393-e0a9-e50e24dcca9e  (本机通知 -> 网页)
 *
 * 数据通路：
 *   BLE 写 -> on_rx 回调 -> StreamBuffer -> bridge 任务 -> UART TX -> DUT
 *   DUT -> UART RX -> uart_rx 任务 -> ble_uart_tx -> BLE 通知
 *
 * 控制通道（网页可动态改波特率等）：
 *   网页 -> 本机：控制帧以 1B 42 4C ("ESC B L") 开头，其余字节为 ASCII 命令。
 *   本机 -> 网页：状态帧同样以 1B 42 4C 开头，其余为 ASCII，结尾 '\n'。
 *   控制帧不会转发给 DUT；不以该魔术字开头的写入一律原样透传。
 */

#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "ble_uart.h"

/* 射频功率运行时调整：esp_ble_tx_power_set()/esp_power_level_t 来自控制器 bt 组件，
 * 该 API 与主机栈无关，NimBLE 下同样可用。 */
#include "esp_bt.h"

/* 连接调优需要直接调用 NimBLE GAP API（2M PHY / 连接参数）。
 * ble_uart 组件 REQUIRES bt，公共依赖会传递过来；main/CMakeLists.txt 亦显式列出 bt。 */
#include "host/ble_gap.h"
#ifndef BLE_HCI_LE_PHY_2M_PREF_MASK
#define BLE_HCI_LE_PHY_2M_PREF_MASK 0x02
#endif

/* ------------------------------------------------------------------ */
/* 配置                                                                */
/* ------------------------------------------------------------------ */

#define DUT_UART            ((uart_port_t)CONFIG_BRIDGE_UART_NUM)
#define DUT_TX_GPIO         CONFIG_BRIDGE_UART_TX
#define DUT_RX_GPIO         CONFIG_BRIDGE_UART_RX
#define DEFAULT_BAUD        CONFIG_BRIDGE_BAUD
#define BUF_SIZE            CONFIG_BRIDGE_BUF_SIZE

/* ---- 连接调优（提速）-------------------------------------------------
 * K5 协议是“发一帧等一帧 ACK”的停等协议，端到端延迟主要由 BLE 连接间隔决定。
 * Android 默认常给 30~50ms，这里在连接后主动请求更短间隔 + 2M PHY。 */
#define TUNE_CONN_ITVL_MIN   6     /* 7.5ms = 6  * 1.25ms */
#define TUNE_CONN_ITVL_MAX   12    /* 15ms  = 12 * 1.25ms */
#define TUNE_SUPERVISION_TO  400   /* 4s    = 400 * 10ms  */

/* ---- BLE 射频发射功率 -------------------------------------------------
 * 本板 3.0V 供电：高功率档实际输出会衰减，发射峰值电流也更易把电源拉低，
 * 因此默认取 +3dBm（兼顾距离与电源裕量）。芯片档位步进 3dB，运行时可用
 * 控制命令 POWER=<dBm> 动态调整（自动就近吸附到支持的档位）。 */
#define TX_POWER_DEFAULT_DBM  3

/* 运行期允许的最大功率：3.0V 下超过约 +9dBm 会因电源跌落导致链路监督超时
 * （disconnect reason 0x208），可在 menuconfig 里调整上限。 */
#define TX_POWER_MAX_DBM      CONFIG_BRIDGE_TX_POWER_MAX_DBM

/* 运行期设置的功率写入 NVS，掉电后仍生效（无记录时用上面的编译期默认值） */
#define NVS_NS_BRIDGE         "bridge"
#define NVS_KEY_TXPWR         "txpow"

/* 控制帧魔术前缀："ESC B L" */
#define CTL_MAGIC0          0x1B
#define CTL_MAGIC1          'B'
#define CTL_MAGIC2          'L'

static const char *TAG = "bridge";

/* ------------------------------------------------------------------ */
/* 全局状态                                                            */
/* ------------------------------------------------------------------ */

static StreamBufferHandle_t s_ble2uart;   /* BLE 写入 -> UART TX */
static StreamBufferHandle_t s_uart2ble;   /* UART RX   -> BLE 通知 */
static uint32_t             s_baud = DEFAULT_BAUD;
static int                  s_tx_power_dbm = TX_POWER_DEFAULT_DBM;   /* 当前 BLE 发射功率(dBm) */

/* 统计与诊断：用于判断「能发不能收」到底断在哪一段 */
static volatile uint32_t    s_dropped_ble2uart;   /* BLE 写入过快被丢弃的字节数 */
static volatile uint32_t    s_ble_rx_total;       /* 从 BLE 收到并转发的总字节数 (B2U) */
static volatile uint32_t    s_uart_rx_total;      /* 从 UART 收到的总字节数 (U2B) */

/* 自检：同一时刻只允许跑一次 */
static volatile bool        s_selftest_running;

/* BLE 链路是否已连接（供连接调优任务判断） */
static volatile bool        s_link_up;

static void selftest_task(void *arg);

/* ------------------------------------------------------------------ */
/* 射频功率：dBm <-> 控制器档位 映射与设置                              */
/* ------------------------------------------------------------------ */

typedef struct {
    int               dbm;
    esp_power_level_t lvl;
} tx_pwr_step_t;

/* 控制器支持的档位（步进 3dB；最高 +20dBm 对应 ESP_PWR_LVL_P20） */
static const tx_pwr_step_t s_tx_pwr_tab[] = {
    { -24, ESP_PWR_LVL_N24 }, { -21, ESP_PWR_LVL_N21 }, { -18, ESP_PWR_LVL_N18 },
    { -15, ESP_PWR_LVL_N15 }, { -12, ESP_PWR_LVL_N12 }, {  -9, ESP_PWR_LVL_N9  },
    {  -6, ESP_PWR_LVL_N6  }, {  -3, ESP_PWR_LVL_N3  }, {   0, ESP_PWR_LVL_N0  },
    {   3, ESP_PWR_LVL_P3  }, {   6, ESP_PWR_LVL_P6  }, {   9, ESP_PWR_LVL_P9  },
    {  12, ESP_PWR_LVL_P12 }, {  15, ESP_PWR_LVL_P15 }, {  18, ESP_PWR_LVL_P18 },
    {  20, ESP_PWR_LVL_P20 },
};
#define TX_PWR_TAB_N (sizeof(s_tx_pwr_tab) / sizeof(s_tx_pwr_tab[0]))

/* 把任意 dBm 就近吸附到支持档位；out_dbm 回传实际档位 */
static esp_power_level_t tx_power_nearest(int dbm, int *out_dbm)
{
    size_t best = 0;
    int best_diff = abs(dbm - s_tx_pwr_tab[0].dbm);
    for (size_t i = 1; i < TX_PWR_TAB_N; i++) {
        int d = abs(dbm - s_tx_pwr_tab[i].dbm);
        if (d < best_diff) {
            best_diff = d;
            best = i;
        }
    }
    if (out_dbm) {
        *out_dbm = s_tx_pwr_tab[best].dbm;
    }
    return s_tx_pwr_tab[best].lvl;
}

/* 设置 BLE 发射功率：默认(涵盖此后建立的连接) + 广播 + 扫描都设一遍，
 * 并对当前已建立的连接立即生效（否则旧连接的功率仍是建立时的旧值）。
 * 该函数在 BLE 主机任务上下文调用（handle_control / app_main）。 */
static esp_err_t tx_power_apply(int dbm, int *applied_dbm)
{
    int got = 0;
    esp_power_level_t lvl = tx_power_nearest(dbm, &got);

    esp_err_t e_def = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, lvl);
    esp_err_t e_adv = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, lvl);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, lvl);   /* 本机不扫描，失败可忽略 */

    /* 已建立的连接逐个改（CONN_HDLx 与连接句柄一一对应） */
    for (uint16_t h = 0; h < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; h++) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(h, &desc) != 0) {
            continue;
        }
        esp_ble_tx_power_set((esp_ble_power_type_t)(ESP_BLE_PWR_TYPE_CONN_HDL0 + h), lvl);
    }

    if (e_def != ESP_OK && e_adv != ESP_OK) {
        return (e_def != ESP_OK) ? e_def : e_adv;
    }
    s_tx_power_dbm = got;
    if (applied_dbm) {
        *applied_dbm = got;
    }
    return ESP_OK;
}

/* 从 NVS 读取上次保存的发射功率；无记录或越界返回 false */
static bool tx_power_nvs_load(int *dbm)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_BRIDGE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    int8_t v = 0;
    esp_err_t err = nvs_get_i8(h, NVS_KEY_TXPWR, &v);
    nvs_close(h);
    if (err != ESP_OK || v < -24 || v > 20) {
        return false;
    }
    *dbm = (int)v;
    return true;
}

/* 把功率写入 NVS（失败仅告警，不影响本次运行） */
static void tx_power_nvs_save(int dbm)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_BRIDGE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS 打开失败，功率未持久化");
        return;
    }
    esp_err_t err = nvs_set_i8(h, NVS_KEY_TXPWR, (int8_t)dbm);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "功率写入 NVS 失败：%s", esp_err_to_name(err));
    }
}

/* ------------------------------------------------------------------ */
/* 设备 -> 网页：发送一条控制/状态帧                                     */
/* ------------------------------------------------------------------ */

static void ctl_send(const char *fmt, ...)
{
    uint8_t frame[128];
    frame[0] = CTL_MAGIC0;
    frame[1] = CTL_MAGIC1;
    frame[2] = CTL_MAGIC2;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf((char *)frame + 3, sizeof(frame) - 4, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)(sizeof(frame) - 4)) {
        n = sizeof(frame) - 4;
    }
    frame[3 + n] = '\n';
    ble_uart_tx(frame, (size_t)(3 + n + 1));
}

/* ------------------------------------------------------------------ */
/* 网页 -> 设备：解析控制命令并执行                                      */
/* 运行在 BLE 主机任务上下文，保持短小、非阻塞。                          */
/* ------------------------------------------------------------------ */

static void handle_control(const uint8_t *data, size_t len)
{
    char cmd[64];
    size_t n = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
    memcpy(cmd, data, n);
    cmd[n] = '\0';

    /* 去掉可能的前导空格 */
    char *p = cmd;
    while (*p == ' ' || *p == '\t') {
        p++;
    }

    if (strncmp(p, "BAUD", 4) == 0) {
        char *arg = p + 4;
        while (*arg == ' ' || *arg == '=' || *arg == ':') {
            arg++;
        }
        long baud = strtol(arg, NULL, 10);
        if (baud < 300 || baud > 5000000) {
            ctl_send("ERR BAUD invalid");
            return;
        }
        esp_err_t err = uart_set_baudrate(DUT_UART, (uint32_t)baud);
        if (err == ESP_OK) {
            s_baud = (uint32_t)baud;
            ESP_LOGI(TAG, "波特率已切换为 %" PRIu32, s_baud);
            ctl_send("OK BAUD %" PRIu32, s_baud);
        } else {
            ctl_send("ERR BAUD %s", esp_err_to_name(err));
        }
    } else if (strncmp(p, "STATUS", 6) == 0) {
        ctl_send("STATUS BAUD %" PRIu32 " PWR %d TX %d RX %d B2U %" PRIu32
                 " U2B %" PRIu32 " DROP %" PRIu32,
                 s_baud, s_tx_power_dbm, DUT_TX_GPIO, DUT_RX_GPIO,
                 s_ble_rx_total, s_uart_rx_total, s_dropped_ble2uart);
    } else if (strncmp(p, "FLUSH", 5) == 0) {
        uart_flush_input(DUT_UART);
        xStreamBufferReset(s_ble2uart);
        xStreamBufferReset(s_uart2ble);
        s_dropped_ble2uart = 0;
        ctl_send("OK FLUSH");
    } else if (strncmp(p, "POWER", 5) == 0) {
        /* POWER         -> 回报当前发射功率
         * POWER=<dBm>   -> 设置（就近吸附到芯片支持的 3dB 档，范围 -24..+20） */
        char *arg = p + 5;
        while (*arg == ' ' || *arg == '\t' || *arg == '=' || *arg == ':') {
            arg++;
        }
        if (*arg == '\0') {
            ctl_send("OK POWER %d", s_tx_power_dbm);
        } else {
            char *end = NULL;
            long dbm = strtol(arg, &end, 10);
            while (end && (*end == ' ' || *end == '\t')) {
                end++;
            }
            if (end == arg || (end && *end != '\0')) {
                ctl_send("ERR POWER invalid");
                return;
            }
            if (dbm < -24 || dbm > TX_POWER_MAX_DBM) {
                ctl_send("ERR POWER range -24..%d", TX_POWER_MAX_DBM);
                return;
            }
            int applied = 0;
            esp_err_t err = tx_power_apply((int)dbm, &applied);
            if (err == ESP_OK) {
                tx_power_nvs_save(applied);   /* 持久化：掉电后仍按此档位 */
                ESP_LOGI(TAG, "射频功率已切换为 %d dBm", applied);
                ctl_send("OK POWER %d", applied);
            } else {
                ctl_send("ERR POWER %s", esp_err_to_name(err));
            }
        }
    } else if (strncmp(p, "PING", 4) == 0) {
        ctl_send("PONG");
    } else if (strncmp(p, "SELFTEST", 8) == 0) {
        /* 自检任务里可以安全地 sleep/轮询，所以丢到独立任务里跑，
         * 不阻塞 BLE 主机任务。 */
        if (s_selftest_running) {
            ctl_send("ERR SELFTEST busy");
        } else {
            s_selftest_running = true;
            if (xTaskCreate(selftest_task, "selftest", 3072, NULL, 5, NULL) != pdPASS) {
                s_selftest_running = false;
                ctl_send("ERR SELFTEST no mem");
            } else {
                ctl_send("OK SELFTEST started");
            }
        }
    } else {
        ctl_send("ERR UNKNOWN");
    }
}

/* ------------------------------------------------------------------ */
/* BLE -> UART：收到网页写入                                            */
/* 运行在 BLE 主机任务，绝不阻塞。                                       */
/* ------------------------------------------------------------------ */

static void ble_on_rx(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return;
    }

    /* 控制帧？(仅当一次写入以魔术字开头时成立) */
    if (len >= 3 && data[0] == CTL_MAGIC0 && data[1] == CTL_MAGIC1 && data[2] == CTL_MAGIC2) {
        handle_control(data + 3, len - 3);
        return;
    }

    /* 普通数据：投递到缓冲，满了就丢弃并计数（调试场景可接受） */
    s_ble_rx_total += (uint32_t)len;
    size_t sent = xStreamBufferSend(s_ble2uart, data, len, 0);
    if (sent < len) {
        s_dropped_ble2uart += (uint32_t)(len - sent);
    }
}

/* ------------------------------------------------------------------ */
/* BLE 事件：连接/订阅状态                                             */
/* ------------------------------------------------------------------ */

static void ble_on_event(const ble_uart_evt_t *e)
{
    switch (e->id) {
    case BLE_UART_EVT_CONNECTED: {
        const uint8_t *b = e->connected.peer.bytes;
        ESP_LOGI(TAG, "已连接 %02x:%02x:%02x:%02x:%02x:%02x",
                 b[0], b[1], b[2], b[3], b[4], b[5]);
        s_link_up = true;   /* 唤醒调优任务：协商 2M PHY 与短连接间隔 */
        break;
    }
    case BLE_UART_EVT_DISCONNECTED:
        ESP_LOGW(TAG, "已断开 reason=0x%x", e->disconnected.reason);
        s_link_up = false;
        break;
    case BLE_UART_EVT_SUBSCRIBED:
        ESP_LOGI(TAG, "TX 通知%s", e->subscribed.subscribed ? "已订阅" : "已取消");
        if (e->subscribed.subscribed) {
            /* 网页订阅成功后主动回报当前状态 */
            ctl_send("READY BAUD %" PRIu32 " PWR %d", s_baud, s_tx_power_dbm);
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* 任务：BLE 缓冲 -> UART TX                                            */
/* ------------------------------------------------------------------ */

static void bridge_ble2uart_task(void *arg)
{
    uint8_t buf[256];
    while (1) {
        size_t n = xStreamBufferReceive(s_ble2uart, buf, sizeof(buf), portMAX_DELAY);
        if (n > 0) {
            uart_write_bytes(DUT_UART, (const char *)buf, n);
        }
    }
}

/* ------------------------------------------------------------------ */
/* 任务：UART RX -> BLE 通知                                            */
/* ------------------------------------------------------------------ */

static void bridge_uart2ble_task(void *arg)
{
    uint8_t buf[256];
    while (1) {
        /* 用有限超时而不是 portMAX_DELAY：uart_read_bytes 在阻塞期间会一直
         * 持有驱动的 rx_mux，会导致 FLUSH(要调 uart_flush_input) 等操作
         * 永久拿不到锁。有限超时下 rx_mux 会周期性释放，代价只是空闲时
         * 每 100ms 醒一次。 */
        int n = uart_read_bytes(DUT_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n <= 0) {
            continue;
        }
        s_uart_rx_total += (uint32_t)n;
#if CONFIG_BRIDGE_RX_DEBUG_LOG
        ESP_LOGI(TAG, "UART RX %d bytes (累计 %" PRIu32 ")", n, s_uart_rx_total);
#endif
        /* 未连接时返回 ENOTCONN；通知队列满(ENOMEM)时短暂退避重试 */
        int rc = ble_uart_tx(buf, (size_t)n);
        for (int retry = 0; rc == BLE_UART_ENOMEM && retry < 20; retry++) {
            vTaskDelay(pdMS_TO_TICKS(5));
            rc = ble_uart_tx(buf, (size_t)n);
        }
    }
}

/* ------------------------------------------------------------------ */
/* 任务：BLE 连接调优（提速）                                           */
/*                                                                     */
/* 连接建立后主动做两件事：                                              */
/*   1) 请求 2M PHY（BLE 5.0，ESP32-C3 支持）：符号率翻倍，大包更明显；  */
/*   2) 请求更短的连接间隔（7.5~15ms, latency=0）。                      */
/* 两者都能直接降低“停等协议”每帧的往返延迟，是最有效的固件侧提速手段。   */
/* 最终由主机拍板，若对端不支持则维持现状（仅打日志，不影响功能）。       */
/* ------------------------------------------------------------------ */

static void ble_conn_tune_task(void *arg)
{
    while (1) {
        if (!s_link_up) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        /* 等 ATT MTU 协商（及可选加密）完成再调参，避免打断握手 */
        vTaskDelay(pdMS_TO_TICKS(300));
        if (!s_link_up) {
            continue;
        }

        struct ble_gap_conn_desc desc;
        for (uint16_t h = 0; h < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; h++) {
            if (ble_gap_conn_find(h, &desc) != 0) {
                continue;
            }
            int rc = ble_gap_set_prefered_le_phy(h,
                                                 BLE_HCI_LE_PHY_2M_PREF_MASK,
                                                 BLE_HCI_LE_PHY_2M_PREF_MASK,
                                                 0 /* phy_opts：非 coded PHY 填 0 */);
            ESP_LOGI(TAG, "PHY 调优(2M) conn=%u rc=%d", h, rc);

            const struct ble_gap_upd_params up = {
                .itvl_min            = TUNE_CONN_ITVL_MIN,
                .itvl_max            = TUNE_CONN_ITVL_MAX,
                .latency             = 0,
                .supervision_timeout = TUNE_SUPERVISION_TO,
                .min_ce_len          = 0,
                .max_ce_len          = 0,
            };
            rc = ble_gap_update_params(h, &up);
            ESP_LOGI(TAG, "连接参数调优 conn=%u itvl=%u-%u rc=%d", h,
                     TUNE_CONN_ITVL_MIN, TUNE_CONN_ITVL_MAX, rc);
            break;
        }

        /* 本次连接已调完，等断开后再处理下一次连接 */
        while (s_link_up) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

/* ------------------------------------------------------------------ */
/* 任务：UART 自检 (SELFTEST)                                           */
/*                                                                     */
/* 做法：往 UART TX 写一串已知图案，DUT 侧把 TX-RX 短接时，图案会经环回   */
/* 被既有的 u2b 任务收走并计入 s_uart_rx_total，这里只统计其增量。         */
/* 刻意不直接调 uart_read_bytes / uart_flush_input —— 那两个都会抢 rx_mux，*/
/* 而阻塞在 portMAX_DELAY 里的 u2b 任务会一直持有它，直接调用会死锁。      */
/* ------------------------------------------------------------------ */

#define SELFTEST_PATTERN    "SELFTEST:TX-RX:0123456789ABCDEF\r\n"
#define SELFTEST_SETTLE_MS  300   /* 等接收静默，避免把残留数据算进来 */
#define SELFTEST_WAIT_MS    500   /* 等待环回数据回来的上限 */

static void selftest_task(void *arg)
{
    const char pattern[] = SELFTEST_PATTERN;
    const size_t len = sizeof(pattern) - 1;

    /* 先等 RX 计数稳定下来（残留数据会被 u2b 任务很快读走并计数） */
    uint32_t last = s_uart_rx_total;
    for (int i = 0; i < SELFTEST_SETTLE_MS / 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (s_uart_rx_total == last) {
            break;
        }
        last = s_uart_rx_total;
    }
    const uint32_t before = s_uart_rx_total;

    /* 写图案：uart_write_bytes 用的是 tx_mux，与接收侧的 rx_mux 互不冲突 */
    int written = uart_write_bytes(DUT_UART, pattern, len);

    /* 轮询等待环回，最多 SELFTEST_WAIT_MS */
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SELFTEST_WAIT_MS);
    uint32_t got;
    do {
        got = s_uart_rx_total - before;
        if (got >= (uint32_t)len) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (xTaskGetTickCount() < deadline);
    got = s_uart_rx_total - before;

    ESP_LOGI(TAG, "SELFTEST sent=%d/%u got=%" PRIu32, written, (unsigned)len, got);
    ctl_send("SELFTEST sent=%d/%u got=%" PRIu32 " %s",
             written, (unsigned)len, got,
             (written == (int)len && got >= (uint32_t)len) ? "PASS" : "FAIL");

    s_selftest_running = false;
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* UART 初始化                                                         */
/* ------------------------------------------------------------------ */

static void dut_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = DEFAULT_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(DUT_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(DUT_UART, DUT_TX_GPIO, DUT_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* 先配置参数/引脚，再安装驱动：顺序反了会导致接收方向不工作 */
    ESP_ERROR_CHECK(uart_driver_install(DUT_UART, 2048, 2048, 0, NULL, 0));
    ESP_LOGI(TAG, "UART%d 就绪 TX=GPIO%d RX=GPIO%d baud=%d",
             CONFIG_BRIDGE_UART_NUM, DUT_TX_GPIO, DUT_RX_GPIO, DEFAULT_BAUD);
}

/* ------------------------------------------------------------------ */
/* 广播数据：把 128 位服务 UUID 放进主广播包，保证 Web Bluetooth 可发现   */
/* 主广播包仅 31 字节，所以这里用短名 "C3-UART"。                        */
/* ------------------------------------------------------------------ */

static const uint8_t adv_payload[] = {
    /* AD type 0x09: Complete Local Name = "C3-UART" */
    0x08, 0x09, 'C', '3', '-', 'U', 'A', 'R', 'T',
    /* AD type 0x07: Complete List of 128-bit Service UUIDs (小端线序) */
    0x11, 0x07,
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
};

/* ------------------------------------------------------------------ */
/* 入口                                                                */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* NVS：蓝牙控制器初始化(射频校准)需要 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* UART + 缓冲 + 任务 */
    dut_uart_init();
    s_ble2uart = xStreamBufferCreate(BUF_SIZE, 1);
    s_uart2ble = xStreamBufferCreate(BUF_SIZE, 1);
    configASSERT(s_ble2uart && s_uart2ble);
    xTaskCreate(bridge_ble2uart_task, "b2u", 3072, NULL, 5, NULL);
    xTaskCreate(bridge_uart2ble_task, "u2b", 3072, NULL, 5, NULL);
    xTaskCreate(ble_conn_tune_task, "tune", 3072, NULL, 4, NULL);

    /* 设备名 = "C3-UART-XXXX"（GAP 服务名，连接后可见） */
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_BT) != ESP_OK) {
        ESP_LOGW(TAG, "读取蓝牙 MAC 失败，设备名后缀降级为 0000");
    }
    char name[BLE_UART_DEVICE_NAME_MAX + 1];
    snprintf(name, sizeof(name), "C3-UART-%02X%02X", mac[4], mac[5]);

    /* 明文、免配对：encrypted=false，security 全 AUTO 即全部关闭 */
    ESP_ERROR_CHECK(ble_uart_install(&(ble_uart_config_t){
        .encrypted      = false,
        .device_name    = name,
        .adv_data       = adv_payload,
        .adv_data_len   = sizeof(adv_payload),
        .ble_uart_on_rx = ble_on_rx,
        .on_event       = ble_on_event,
    }));
    ESP_ERROR_CHECK(ble_uart_open());

    /* 射频功率：优先用 NVS 里上次保存的档位（掉电持久化），无记录时退回
     * 编译期默认值（+3dBm @3.0V）。之后网页端可用 POWER=<dBm> 调整并保存。 */
    int boot_dbm = TX_POWER_DEFAULT_DBM;
    if (tx_power_nvs_load(&boot_dbm)) {
        ESP_LOGI(TAG, "读取到已保存的射频功率 %d dBm", boot_dbm);
    }
    if (boot_dbm > TX_POWER_MAX_DBM) {
        /* 兼容旧版本存下来的超上限值（如误设的 +20dBm），上电直接钳位 */
        ESP_LOGW(TAG, "已保存功率 %d dBm 超过上限，钳位到 %d dBm", boot_dbm, TX_POWER_MAX_DBM);
        boot_dbm = TX_POWER_MAX_DBM;
    }
    int pwr_applied = 0;
    esp_err_t pwr_err = tx_power_apply(boot_dbm, &pwr_applied);
    if (pwr_err == ESP_OK) {
        ESP_LOGI(TAG, "射频发射功率 = %d dBm", pwr_applied);
    } else {
        ESP_LOGW(TAG, "射频功率设置失败：%s（沿用控制器默认档）", esp_err_to_name(pwr_err));
    }

    ESP_LOGI(TAG, "BLE 已启动，广播名 '%s'（广播短名 'C3-UART'）", name);
}
