#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "boards.h"
#include "app_timer.h"
#include "custom_gpio_drivers.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

#include "bsp.h"
#include "bsp_btn_ble.h"

#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

#include "generic_level_server.h"
#include "scene_setup_server.h"
#include "model_config_file.h"
#include "generic_level_client.h"

#include "log.h"
#include "rtt_input.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_mesh_config_app.h"
#include "nrf.h"
#include "example_common.h"
#include "pwm_utils.h"
#include "nrf_mesh_config_examples.h"
#include "app_level.h"
#include "app_dtt.h"
#include "ble_softdevice_support.h"
#include "nrf_delay.h"

#include "nrf_drv_wdt.h"

#include "nrf_drv_saadc.h"
#define MQ2_ANALOG_PIN NRF_SAADC_INPUT_AIN0

#include "st7789_drivers.h"
#include "nrf52_timer.h"
#include "nrf52_at24c32_64.h"
#include "nrf52_aht10.h"
#include "DHT.h"

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

/* ========================================================================== */
/* CAU HINH TN2 (HIEU CHUAN 2 DIEM KHONG DAY + WHITELIST CHUOI 1..4 CHANG)    */
/* ========================================================================== */
#ifndef EXP_ID
#define EXP_ID                    2
#endif
#ifndef FW_GIT_HASH
#define FW_GIT_HASH               "tn2_wcal_v2"
#endif
#ifndef EXP_NUM_MSGS
#define EXP_NUM_MSGS              1500
#endif
#ifndef EXP_PERIOD_MS
#define EXP_PERIOD_MS             1000
#endif
#ifndef EXP_TTL
#define EXP_TTL                   4
#endif
#ifndef EXP_NET_TX_COUNT
#define EXP_NET_TX_COUNT          2
#endif
#ifndef EXP_RELAY_TX_COUNT
#define EXP_RELAY_TX_COUNT        3     /* stock: relay_retx = 2 -> tx_count = 3 */
#endif
#ifndef EXP_TX_POWER_DBM
#define EXP_TX_POWER_DBM          0
#endif
#ifndef EXP_SEND_UNACK
#define EXP_SEND_UNACK            1
#endif
#ifndef EXP_READ_CFG
#define EXP_READ_CFG              1
#endif
#ifndef EXP_ENABLE_TRIG
#define EXP_ENABLE_TRIG           1
#endif
#ifndef EXP_LIVE_LOG
#define EXP_LIVE_LOG              0
#endif
#ifndef EXP_ENABLE_ALARM_BUTTON
#define EXP_ENABLE_ALARM_BUTTON   0
#endif
#ifndef EXP_HOPS
#define EXP_HOPS                  4     /* 1..4 chang cho TN2 */
#endif
#ifndef EXP_DEBUG_RX_MAC
#define EXP_DEBUG_RX_MAC          0
#endif
#ifndef EXP_MAX_TX_FAIL
#define EXP_MAX_TX_FAIL           30
#endif
#ifndef EXP_HALT_ON_FAULT
#define EXP_HALT_ON_FAULT         1
#endif
#ifndef EXP_STRICT_CFG
#define EXP_STRICT_CFG            1     /* Kiem tra chat TTL, net_tx va relay_tx */
#endif
#ifndef EXP_RTT_KEYS
#define EXP_RTT_KEYS              0     /* 0 = chi dung nut bam vat ly */
#endif
#ifndef EXP_TRACE_STEPS
#define EXP_TRACE_STEPS           0
#endif
#ifndef EXP_HOP_WHITELIST
#define EXP_HOP_WHITELIST         1
#endif
#ifndef EXP_HOPS_RUNTIME
#define EXP_HOPS_RUNTIME          1     /* 1 = doi so chang bang Nut 1 */
#endif

/* Cau hinh dot hieu chuan 2 diem khong day (CAL_PRE & CAL_POST: 20 goi/dot) */
#ifndef EXP_CAL_SEQ_BASE
#define EXP_CAL_SEQ_BASE          30000 /* 30001..30020 = CAL_PRE; 30021..30040 = CAL_POST */
#endif
#ifndef EXP_CAL_BURST_MSGS
#define EXP_CAL_BURST_MSGS        20
#endif

#if (EXP_PERIOD_MS != 1000)
#warning "Chu ky phat thuc te co dinh 1000 ms (theo m_second_changed)."
#endif
#if (EXP_HOPS < 1) || (EXP_HOPS > 4)
#error "EXP_HOPS phai nam trong 1..4"
#endif

#if EXP_SEND_UNACK
#define EXP_PAYLOAD_LEN           5
#else
#define EXP_PAYLOAD_LEN           7
#endif

#if EXP_TRACE_STEPS
#define EXP_STEP(s)               SEGGER_RTT_WriteString(0, "# [STEP] " s "\r\n")
#else
#define EXP_STEP(s)               ((void)0)
#endif

#if EXP_READ_CFG
#include "mesh_opt_core.h"
#endif

/* Bien dong bo ma goi tang Network tu network.c */
extern volatile uint32_t g_last_tx_net_seq;
extern volatile uint32_t g_last_rx_net_seq;

#ifndef TRIG_PIN
#define TRIG_PIN                  11
#endif
#ifndef TRIG_WIDTH_US
#define TRIG_WIDTH_US             200
#endif
#ifndef TRIG_GAP_US
#define TRIG_GAP_US               200
#endif

#define EXP_TRIG_TX               1
#define EXP_TRIG_FWD              2
#define EXP_TRIG_RX               3

static volatile uint32_t m_trig_cnt[4];

void exp_trig_begin(void)
{
#if EXP_ENABLE_TRIG
    nrf_gpio_pin_set(TRIG_PIN);
#endif
}

void exp_trig_end(uint8_t pulses)
{
#if EXP_ENABLE_TRIG
    nrf_delay_us(TRIG_WIDTH_US);
    nrf_gpio_pin_clear(TRIG_PIN);
    for (uint8_t i = 1; i < pulses; i++)
    {
        nrf_delay_us(TRIG_GAP_US);
        nrf_gpio_pin_set(TRIG_PIN);
        nrf_delay_us(TRIG_WIDTH_US);
        nrf_gpio_pin_clear(TRIG_PIN);
    }
    if (pulses < 4)
    {
        m_trig_cnt[pulses]++;
    }
#else
    (void)pulses;
#endif
}

/* ========================================================================== */
/* WHITELIST MAC THEO CHUOI TUYEN TINH (N4 -> N3 -> N2 -> N1 -> GW)           */
/* ========================================================================== */
typedef struct {
    uint8_t addr[BLE_GAP_ADDR_LEN];
} exp_node_mac_t;

static const exp_node_mac_t EXP_TOPOLOGY_CHAIN[5] = {
    { {0x4D, 0xEA, 0x4F, 0x49, 0x8F, 0xF4} }, /* N4 (index 0) */
    { {0xA6, 0x63, 0x52, 0xE2, 0x7B, 0xCB} }, /* N3 (index 1) */
    { {0x16, 0x19, 0x71, 0x80, 0x5B, 0xC9} }, /* N2 (index 2) */
    { {0xD0, 0x26, 0xE8, 0x7D, 0x07, 0xD2} }, /* N1 (index 3) */
    { {0x4A, 0x5A, 0x32, 0xA2, 0x18, 0xF8} }, /* GW (index 4) */
};

static int8_t m_my_chain_index = -1;
static ble_gap_addr_t m_my_ble_addr;

void exp_chain_mac_init(void)
{
    (void)sd_ble_gap_addr_get(&m_my_ble_addr);

    SEGGER_RTT_printf(0, "# [NODE_ADDR] MAC (UI):  %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                      m_my_ble_addr.addr[5], m_my_ble_addr.addr[4], m_my_ble_addr.addr[3],
                      m_my_ble_addr.addr[2], m_my_ble_addr.addr[1], m_my_ble_addr.addr[0]);

    SEGGER_RTT_printf(0, "# [NODE_ADDR] ARRAY C:   { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X }\r\n",
                      m_my_ble_addr.addr[0], m_my_ble_addr.addr[1], m_my_ble_addr.addr[2],
                      m_my_ble_addr.addr[3], m_my_ble_addr.addr[4], m_my_ble_addr.addr[5]);

    m_my_chain_index = -1;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (memcmp(m_my_ble_addr.addr, EXP_TOPOLOGY_CHAIN[i].addr, BLE_GAP_ADDR_LEN) == 0)
        {
            m_my_chain_index = (int8_t)i;
            break;
        }
    }

#if EXP_HOP_WHITELIST
    if (m_my_chain_index == -1)
    {
        SEGGER_RTT_WriteString(0, "# [LOI] MAC KHONG CO TRONG BANG CHUOI! TU CHOI chay run.\r\n");
    }
    else
    {
        SEGGER_RTT_printf(0, "# [INFO] Node index trong chuoi: %d\r\n", (int)m_my_chain_index);
    }
#endif
}

#define EXP_GW_INDEX              4

volatile uint8_t g_dbg_rx_mac_left = 0;
volatile uint8_t g_exp_hops = EXP_HOPS;

static inline bool exp_is_dest(void)
{
#if EXP_HOP_WHITELIST
    return (m_my_chain_index == EXP_GW_INDEX);
#else
    return true;
#endif
}

bool exp_packet_allow(const uint8_t * p_adv_addr, uint8_t adv_type)
{
#if EXP_HOP_WHITELIST
    if (adv_type != 0x2A)
    {
        return true;
    }
    if (m_my_chain_index <= 0)
    {
        return false;
    }

    const int hops = (int)g_exp_hops;
    int pred;
    if (m_my_chain_index == EXP_GW_INDEX)
    {
        pred = hops - 1;
    }
    else if (m_my_chain_index < hops)
    {
        pred = m_my_chain_index - 1;
    }
    else
    {
        return false;
    }

    return (memcmp(p_adv_addr, EXP_TOPOLOGY_CHAIN[pred].addr, BLE_GAP_ADDR_LEN) == 0);
#else
    (void)p_adv_addr;
    (void)adv_type;
    return true;
#endif
}

extern uint32_t m_secondCounter;

/* Dong ho cuc bo RTC1 32768 Hz CHAY LIEN TUC (KHONG reset giua CAL_PRE -> MAIN -> CAL_POST). */
static uint32_t m_rtc_last = 0;
static uint64_t m_rtc_acc  = 0;

static inline uint32_t get_local_time_us(void)
{
    uint32_t now, diff;
    uint64_t acc;

    CRITICAL_REGION_ENTER();
    now  = app_timer_cnt_get() & 0x00FFFFFFu;
    diff = (now - m_rtc_last) & 0x00FFFFFFu;
    m_rtc_last = now;
    m_rtc_acc += diff;
    acc = m_rtc_acc;
    CRITICAL_REGION_EXIT();

    return (uint32_t)((acc * 1000000ULL) / 32768ULL);
}

enum signalStatus
{
    _normalStatus    = 1,
    _fireAlarmStatus = 3,
};

static uint32_t m_present_level;
uint32_t timerBCounter;
uint32_t refreshDisplayCounter = 1;
bool _getResetProvisionKey = false;

volatile uint16_t g_last_mesh_src_addr = 0;
volatile int8_t   g_last_mesh_rssi     = 0;
volatile uint8_t  g_last_mesh_ttl      = 0;

bool clientProvisioned = false;
bool clientProvisionChanged = false;

static void trigger_egu30_event(void);

bool aht10Available;
float _clientTemperature;
float _clientHumidity;

bool mq2Available = true;
uint16_t _clientMQ2Value;

bool clearAllDisplayNodes;

#define UART_TX_BUF_SIZE        256
#define UART_RX_BUF_SIZE        256

void uart_error_handle(app_uart_evt_t * p_event)
{
    (void)p_event;
}

dsm_local_unicast_address_t node_address;

void uart_puts(const char *_ch)
{
    while (*_ch)
    {
        if (app_uart_put(*_ch) != NRF_SUCCESS)
        {
            break;
        }
        _ch++;
    }
}

void uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_PIN_DISCONNECTED,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = UART_PIN_DISCONNECTED,
        .cts_pin_no   = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
    #if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
    #else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
    #endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

#define LOG_QUEUE_SIZE        16
#define LOG_MSG_MAX_LEN       128

static char m_log_queue[LOG_QUEUE_SIZE][LOG_MSG_MAX_LEN];
static volatile uint8_t m_log_head = 0;
static volatile uint8_t m_log_tail = 0;
static volatile bool m_logging_enabled = true;
bool periodic_message_allowed = false;

static void log_uart_mesh_event(const char* event, uint16_t src, uint16_t dst,
                                uint32_t seq, uint8_t ttl_tx, uint8_t ttl_rx,
                                int8_t rssi, const char* traffic_class, uint8_t len)
{
#if !EXP_LIVE_LOG
    (void)event; (void)src; (void)dst; (void)seq; (void)ttl_tx; (void)ttl_rx;
    (void)rssi; (void)traffic_class; (void)len;
    return;
#endif
    if (!m_logging_enabled)
    {
        return;
    }
    uint32_t t_us = get_local_time_us();
    uint16_t my_id = node_address.address_start & 0xFFFF;

    uint8_t next_head = (uint8_t)((m_log_head + 1) % LOG_QUEUE_SIZE);
    if (next_head != m_log_tail)
    {
        snprintf(m_log_queue[m_log_head], LOG_MSG_MAX_LEN,
                 "%u,0x%04X,%s,0x%04X,0x%04X,%u,%u,%u,%d,%s,%u\r\n",
                 (unsigned)t_us, (unsigned)my_id, event,
                 (unsigned)src, (unsigned)dst, (unsigned)seq,
                 (unsigned)ttl_tx, (unsigned)ttl_rx, (int)rssi, traffic_class, (unsigned)len);

        SEGGER_RTT_WriteString(0, m_log_queue[m_log_head]);
        m_log_head = next_head;
    }
}

static void log_queue_flush(void)
{
    while (m_log_tail != m_log_head)
    {
        uart_puts(m_log_queue[m_log_tail]);
        m_log_tail = (uint8_t)((m_log_tail + 1) % LOG_QUEUE_SIZE);
    }
}

extern nrfx_wdt_channel_id m_channel_id;

/* 1580 x 12 byte = 18.960 byte (+960 B so voi ban 1800 x 10 B cu).
 * Du cho 20 CAL_PRE + 1500 MAIN + 20 CAL_POST + 40 du phong. */
#define MAX_OFFLINE_RECORDS  1580

enum {
    EXP_EVT_TX    = 0,
    EXP_EVT_RX    = 1,
    EXP_EVT_RELAY = 2
};

#define PACK_TTL_EVT(ttl, evt)   ((uint8_t)((((evt) & 0x03u) << 6) | ((ttl) & 0x3Fu)))
#define UNPACK_TTL(ttl_evt)      ((uint8_t)((ttl_evt) & 0x3Fu))
#define UNPACK_EVT(ttl_evt)      ((uint8_t)(((ttl_evt) >> 6) & 0x03u))

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint16_t seq_alarm;   /* bit15=alarm; bit14..0=app_seq (1..1500=MAIN, >=30000=CAL, 0=RELAY) */
    uint16_t net_seq16;   /* 16 bit thap Network SEQ (khop TX -> Relay -> GW) */
    uint16_t src;
    uint8_t  ttl_evt;
    int8_t   rssi;
} offline_log_t;

typedef enum {
    CAL_PHASE_IDLE      = 0,
    CAL_PHASE_PRE_DONE  = 1,
    CAL_PHASE_MAIN_DONE = 2,
    CAL_PHASE_POST_DONE = 3,
    CAL_PHASE_ABORTED   = 4   /* Dung giua chung -> bat buoc xa (Nut 3) hoac xoa RAM (Nut 4) */
} exp_cal_phase_t;

static offline_log_t     m_offline_logs[MAX_OFFLINE_RECORDS];
static volatile uint16_t m_offline_count    = 0;
static volatile uint16_t m_offline_dropped  = 0;
static volatile bool     m_dump_requested   = false;
volatile uint8_t         g_display_src      = 0;

static volatile bool     m_req_toggle       = false;
#if EXP_HOPS_RUNTIME
static volatile bool     m_req_hops         = false;
#endif
static volatile bool     m_req_cal          = false;
static volatile bool     m_req_clear_ram    = false;
static volatile uint16_t m_cal_remaining    = 0;
static volatile bool     m_cal_done         = false;
static volatile exp_cal_phase_t m_cal_phase = CAL_PHASE_IDLE;
static volatile bool     m_boot_cfg_checked = false;
static uint16_t          m_cal_seq          = EXP_CAL_SEQ_BASE;
static uint16_t          m_cal_tx_cnt       = 0;

static volatile bool     m_run_done         = false;
static volatile bool     m_run_aborted      = false;
static volatile bool     m_role_tx          = false;
static volatile uint32_t m_tx_fail_total    = 0;
static volatile uint32_t m_tx_fail_consec   = 0;
static volatile uint32_t m_last_err         = 0;
static uint16_t          m_tx_dst           = 0xFFFF;
static uint32_t          g_reset_reason     = 0;

static uint8_t m_cfg_ttl_pub  = 0xFF;
static uint8_t m_cfg_rtx      = 0xFF;
static uint8_t m_cfg_txcnt    = 0xFF;
static int8_t  m_cfg_dbm      = 127;
static uint8_t m_cfg_ttl_eff  = 0xFF;
static uint8_t m_cfg_relay_tx = 0xFF;
static uint8_t m_cfg_relay_en = 0xFF;
static int     m_cfg_srv_pub  = -1;
static int     m_cfg_cli_pub  = -1;
static char    m_cfg_buf[544];
static char    m_line_buf[144];

/* Dat sau m_cfg_cli_pub de build sach ca khi EXP_HOP_WHITELIST == 0 */
static inline bool exp_is_originator(void)
{
#if EXP_HOP_WHITELIST
    return (m_my_chain_index == 0);
#else
    return (m_cfg_cli_pub == 1);
#endif
}

static bool rtt_write_line_reliable(const char *p, unsigned len)
{
    uint32_t waited_ms = 0;
    while (SEGGER_RTT_Write(0, p, len) != len)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);
        nrf_delay_ms(1);
        if (++waited_ms >= 3000)
        {
            return false;
        }
    }
    return true;
}

static void offline_log_store_tx(uint16_t seq, uint16_t net_seq16, uint8_t ttl, bool is_alarm, uint32_t t_us)
{
    if (!m_logging_enabled)
    {
        return;
    }
    if (m_offline_count >= MAX_OFFLINE_RECORDS)
    {
        m_offline_dropped++;
        return;
    }

    uint16_t idx = m_offline_count++;
    m_offline_logs[idx].t_us      = t_us;
    m_offline_logs[idx].seq_alarm = (uint16_t)(((is_alarm ? 1u : 0u) << 15) | (seq & 0x7FFFu));
    m_offline_logs[idx].net_seq16 = net_seq16;
    m_offline_logs[idx].src       = (uint16_t)(node_address.address_start & 0xFFFFu);
    m_offline_logs[idx].ttl_evt   = PACK_TTL_EVT(ttl, EXP_EVT_TX);
    m_offline_logs[idx].rssi      = 0;
}

void offline_log_push_tx(uint16_t seq, uint8_t ttl, bool is_alarm)
{
    offline_log_store_tx(seq, (uint16_t)(g_last_tx_net_seq & 0xFFFFu), ttl, is_alarm, get_local_time_us());
}

/* Ghi moc t_local_us va net_seq16 tai cac node Relay (goi tu packet_relay trong network.c) */
void offline_log_push_relay(uint16_t src, uint16_t dst, uint32_t net_seq,
                            uint8_t ttl_tx, uint8_t ttl_rx, int8_t rssi)
{
    (void)dst;
    (void)ttl_tx;
    if (!m_logging_enabled || exp_is_dest())
    {
        return;
    }

    const uint32_t t_us = get_local_time_us();
    uint16_t clean_src = src;
    if (clean_src % 2 == 0 && clean_src > 1)
    {
        clean_src -= 1;
    }

    if (m_offline_count < MAX_OFFLINE_RECORDS)
    {
        uint16_t idx = m_offline_count++;
        m_offline_logs[idx].t_us      = t_us;
        m_offline_logs[idx].seq_alarm = 0;
        m_offline_logs[idx].net_seq16 = (uint16_t)(net_seq & 0xFFFFu);
        m_offline_logs[idx].src       = clean_src;
        m_offline_logs[idx].ttl_evt   = PACK_TTL_EVT(ttl_rx, EXP_EVT_RELAY);
        m_offline_logs[idx].rssi      = rssi;
    }
    else
    {
        m_offline_dropped++;
    }
}

/* Ghi moc nhan RX chi tai node dich GW (Relay da ghi trong offline_log_push_relay) */
void offline_log_push(uint16_t src, uint8_t ttl, int8_t rssi, uint16_t payload)
{
    if (!m_logging_enabled)
    {
        return;
    }

    uint16_t own = (uint16_t)(node_address.address_start & 0xFFFFu);
    if (src == own || src == (uint16_t)(own + 1))
    {
        return;
    }

    if (!exp_is_dest())
    {
        return;
    }

    exp_trig_begin();
    const uint32_t t_us = get_local_time_us();

    uint16_t raw_seq     = payload & 0x7FFFu;
    uint8_t  traffic_bit = (payload >> 15) & 0x01u;

    uint16_t clean_src = src;
    if (clean_src % 2 == 0 && clean_src > 1)
    {
        clean_src -= 1;
    }
    g_display_src = (uint8_t)(clean_src & 0xFFu);

    if (m_offline_count < MAX_OFFLINE_RECORDS)
    {
        uint16_t idx = m_offline_count++;
        m_offline_logs[idx].t_us      = t_us;
        m_offline_logs[idx].seq_alarm = (uint16_t)((traffic_bit << 15) | raw_seq);
        m_offline_logs[idx].net_seq16 = (uint16_t)(g_last_rx_net_seq & 0xFFFFu);
        m_offline_logs[idx].src       = clean_src;
        m_offline_logs[idx].ttl_evt   = PACK_TTL_EVT(ttl, EXP_EVT_RX);
        m_offline_logs[idx].rssi      = rssi;
    }
    else
    {
        m_offline_dropped++;
    }

    log_uart_mesh_event("RX", clean_src, node_address.address_start,
                        raw_seq, 0, ttl, rssi,
                        (raw_seq >= EXP_CAL_SEQ_BASE) ? "CAL" : (traffic_bit ? "ALARM" : "TELEMETRY"),
                        EXP_PAYLOAD_LEN);

    exp_trig_end(EXP_TRIG_RX);
}

static void log_config_init(void)
{
    SEGGER_RTT_Init();
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_DEFAULT_BACKENDS_INIT();
    }
    SEGGER_RTT_WriteString(0, "\r\n\r\n====== RTT DA KET NOI THANH CONG (" FW_GIT_HASH ") ======\r\n");
    SEGGER_RTT_WriteString(0, "# Nut 1: Doi hops (1..4) | Nut 2: Start/Stop 1500 goi | Nut 3: Xa CSV | Nut 4: CAL 20 goi (N4) / Clear RAM\r\n\r\n");
}

void mq2_saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(MQ2_ANALOG_PIN);
    err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

uint16_t read_mq2_sensor(void)
{
    nrf_saadc_value_t val;
    nrf_drv_saadc_sample_convert(0, &val);
    if (val < 0) val = 0;
    return (uint16_t)val;
}

#define APP_LEVEL_ELEMENT_INDEX       (0)
#define APP_FORCE_SEGMENTATION        (false)
#define APP_MIC_SIZE                  (NRF_MESH_TRANSMIC_SIZE_SMALL)
#define CLIENT_MODEL_INSTANCE_COUNT   (1)
#define APP_LEVEL_DELAY_MS            (100)
#define APP_LEVEL_TRANSITION_TIME_MS  (10)

static void app_generic_level_client_status_cb(const generic_level_client_t * p_self, const access_message_rx_meta_t * p_meta, const generic_level_status_params_t * p_in);
static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status);
static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self);

static generic_level_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];

const generic_level_client_callbacks_t client_cbs =
{
    .level_status_cb = app_generic_level_client_status_cb,
    .ack_transaction_status_cb = app_gen_level_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_level_client_publish_interval_cb
};

static void app_generic_level_client_status_cb(const generic_level_client_t * p_self, const access_message_rx_meta_t * p_meta, const generic_level_status_params_t * p_in)
{
    (void)p_self; (void)p_meta; (void)p_in;
}

static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    (void)model_handle; (void)p_args; (void)status;
}

static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
    (void)handle; (void)p_self;
}

static volatile uint16_t m_tx_packet_count = 0;
static uint32_t m_global_seq = 0;

static void publish_present_alarm_status(bool _statusSending, bool _isCal)
{
    if (!_statusSending && !_isCal && m_tx_packet_count >= EXP_NUM_MSGS)
    {
        periodic_message_allowed = false;
        m_run_done = true;
        return;
    }

    uint8_t client = 0;
    static generic_level_set_params_t set_params = {0};

    const char *p_class = _isCal ? "CAL" : (_statusSending ? "ALARM" : "TELEMETRY");
    uint16_t traffic_bit = _statusSending ? 1u : 0u;

    static uint8_t m_tid_counter = 0;
    uint16_t seq_15bit = _isCal ? (uint16_t)((m_cal_seq + 1u) & 0x7FFFu)
                                : (uint16_t)((m_global_seq + 1u) & 0x7FFFu);

    uint16_t payload = (uint16_t)((traffic_bit << 15) | seq_15bit);

    set_params.level = (int16_t)payload;
    set_params.tid   = m_tid_counter++;

    exp_trig_begin();
    const uint32_t t_tx_us = get_local_time_us();

#if EXP_SEND_UNACK
    uint32_t status = generic_level_client_set_unack(&m_clients[client], &set_params, NULL, 0);
#else
    model_transition_t transition_params;
    transition_params.delay_ms = APP_LEVEL_DELAY_MS;
    transition_params.transition_time_ms = APP_LEVEL_TRANSITION_TIME_MS;
    (void)access_model_reliable_cancel(m_clients[client].model_handle);
    uint32_t status = generic_level_client_set(&m_clients[client], &set_params, &transition_params);
#endif

    exp_trig_end(EXP_TRIG_TX);

    if (status == NRF_SUCCESS)
    {
        m_tx_fail_consec = 0;

        if (_isCal)
        {
            m_cal_seq++;
            m_cal_tx_cnt++;
            if (m_cal_remaining > 0 && --m_cal_remaining == 0)
            {
                m_cal_done = true;
            }
        }
        else
        {
            m_global_seq++;
            if (!_statusSending)
            {
                m_tx_packet_count++;
            }
        }

        uint8_t current_ttl = 0xFF;
        (void)access_model_publish_ttl_get(m_clients[client].model_handle, &current_ttl);
        if (current_ttl == 0xFF)
        {
            current_ttl = access_default_ttl_get();
        }

        offline_log_store_tx(seq_15bit, (uint16_t)(g_last_tx_net_seq & 0xFFFFu),
                             current_ttl, _statusSending, t_tx_us);

        log_uart_mesh_event("TX", node_address.address_start, m_tx_dst,
                            (uint32_t)seq_15bit, current_ttl, 0, 0, p_class, EXP_PAYLOAD_LEN);

        if (!_statusSending && !_isCal && m_tx_packet_count >= EXP_NUM_MSGS)
        {
            periodic_message_allowed = false;
            m_run_done = true;
        }
    }
    else
    {
        m_tx_fail_total++;
        m_tx_fail_consec++;
        m_last_err = status;
        if (m_tx_fail_consec >= EXP_MAX_TX_FAIL)
        {
            periodic_message_allowed = false;
            m_cal_remaining = 0;
            m_cal_phase = CAL_PHASE_ABORTED;
            m_run_aborted = true;
        }
    }
}

static void mesh_main_button_event_handler(uint32_t button_number)
{
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        case 1:
#if EXP_HOPS_RUNTIME
            m_req_hops = true;
#endif
            break;
        case 2:
            m_req_toggle = true;
            break;
        case 3:
            m_dump_requested = true;
            break;
        case 4:
            clearAllDisplayNodes = true;
            if (exp_is_originator())
            {
                m_req_cal = true;
            }
            else
            {
                m_req_clear_ram = true;
            }
            break;
        default:
            break;
    }
}

#if EXP_RTT_KEYS
static void exp_rtt_key_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        mesh_main_button_event_handler((uint32_t)(key - '1'));
    }
}
#endif

void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        case BSP_EVENT_KEY_1:
        case BSP_EVENT_KEY_2:
        case BSP_EVENT_KEY_3:
            mesh_main_button_event_handler(event - BSP_EVENT_KEY_0);
            break;
        default:
            break;
    }
}

static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;
    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void models_client_init_cb(void)
{
    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;
        ERROR_CHECK(generic_level_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt);
static void app_level_server_set_cb(const app_level_server_t * p_server, uint32_t present_level);
static void app_level_server_get_cb(const app_level_server_t * p_server, uint32_t * p_present_level);
static void app_level_server_transition_cb(const app_level_server_t * p_server, uint32_t transition_time_ms, uint32_t target_level, app_transition_type_t transition_type);

static nrf_mesh_evt_handler_t m_event_handler =
{
    .evt_cb = mesh_events_handle,
};

APP_LEVEL_SERVER_DEF(m_level_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     NULL,
                     app_level_server_set_cb,
                     app_level_server_get_cb,
                     app_level_server_transition_cb);

static void app_level_server_set_cb(const app_level_server_t * p_server, uint32_t present_level)
{
    (void)p_server;
    uint16_t payload     = (uint16_t)present_level;
    uint8_t  traffic_bit = (payload >> 15) & 0x01;

    uint16_t raw_src = g_last_mesh_src_addr;
    uint16_t src_addr = raw_src;
    if (raw_src % 2 == 0 && raw_src > 1)
    {
        src_addr = raw_src - 1;
    }

    uint32_t display_stat = (traffic_bit == 1) ? _fireAlarmStatus : _normalStatus;
    insert_node((uint8_t)(src_addr & 0xFF), display_stat, false);
    refreshDisplayCounter = 1;
}

static void app_level_server_get_cb(const app_level_server_t * p_server, uint32_t * p_present_level)
{
    (void)p_server;
    *p_present_level = (node_address.address_start << 16) + 6;
}

static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                           uint32_t transition_time_ms,
                                           uint32_t target_level,
                                           app_transition_type_t transition_type)
{
    (void)p_server; (void)transition_time_ms; (void)target_level; (void)transition_type;
}

static void app_model_init(void)
{
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_LEVEL_ELEMENT_INDEX));
}

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_ENABLED)
    {
        APP_ERROR_CHECK(app_level_value_restore(&m_level_server_0));
    }
}

static void node_reset(void)
{
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    model_config_file_clear();
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, false);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_addresses_get(&node_address);
    clientProvisionChanged = true;
    clientProvisioned = true;
}

static void provisioning_complete_cb(void)
{
#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif
    unicast_address_print();
    m_boot_cfg_checked = false;
}

static void models_init_cb(void)
{
    app_model_init();
    models_client_init_cb();
}

static void mesh_init(void)
{
    model_config_file_init();

    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);

    if (status == NRF_SUCCESS)
    {
        status = model_config_file_config_apply();
    }

    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            model_config_file_clear();
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP, LOG_LEVEL_WARN, LOG_CALLBACK_DEFAULT);

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if EXP_ENABLE_TRIG
    nrf_gpio_cfg_output(TRIG_PIN);
    nrf_gpio_pin_clear(TRIG_PIN);
#endif

    bool erase_bonds;
    buttons_leds_init(&erase_bonds);
    ble_stack_init();

    exp_chain_mac_init();

    {
        uint32_t rr = 0;
        if (sd_power_reset_reason_get(&rr) == NRF_SUCCESS)
        {
            g_reset_reason = rr;
            (void)sd_power_reset_reason_clr(0xFFFFFFFFu);
        }
    }

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    hal_led_blink_stop();
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
    mesh_init();

#if EXP_RTT_KEYS
    rtt_input_enable(exp_rtt_key_handler, 100);
#endif
}

static void start(void)
{
    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_DM_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));

        clientProvisionChanged = true;
        clientProvisioned = false;
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());
    nrf_mesh_evt_handler_add(&m_event_handler);
    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

static void trigger_egu30_event(void)
{
    NRF_EGU3->TASKS_TRIGGER[0] = 1;
}

static void all_parameters_initialization(void)
{
    m_present_level = _normalStatus;
    timerBCounter = 1;
    periodic_message_allowed = false;
    aht10Available = true;
    _clientTemperature = 0;
    _clientHumidity = 0;
    clearAllDisplayNodes = false;
}

void SWI3_EGU3_IRQHandler(void)
{
    NRF_EGU3->EVENTS_TRIGGERED[0] = 0;

    if (_getResetProvisionKey)
    {
        if (mesh_stack_is_device_provisioned())
        {
            mesh_config_clear();
            node_reset();
        }
    }

    if (m_present_level == _fireAlarmStatus)
    {
        publish_present_alarm_status(true, false);
    }
    else if (m_cal_remaining > 0)
    {
        publish_present_alarm_status(false, true);
    }
    else if (periodic_message_allowed)
    {
        publish_present_alarm_status(false, false);
    }
}

static void egu_init(void)
{
    NRF_EGU3->INTENSET = EGU_INTENSET_TRIGGERED0_Enabled << EGU_INTENSET_TRIGGERED0_Pos;
    NVIC_SetPriority(SWI3_EGU3_IRQn, 6);
    NVIC_EnableIRQ(SWI3_EGU3_IRQn);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    static char err_buf[160];
    const uint32_t ram_end = 0x20000000u + (NRF_FICR->INFO.RAM * 1024u);
    bool info_ok = (info >= 0x20000000u) && ((info & 3u) == 0u) && ((info + 12u) <= ram_end);

    if (id == NRF_FAULT_ID_SDK_ERROR && info_ok)
    {
        const error_info_t * e = (const error_info_t *)info;
        snprintf(err_buf, sizeof(err_buf),
                 "\r\n[FAULT] SDK_ERROR id=0x%08X pc=0x%08X err=0x%08X line=%u\r\n",
                 (unsigned int)id, (unsigned int)pc,
                 (unsigned int)e->err_code, (unsigned int)e->line_num);
    }
    else if (id == NRF_FAULT_ID_SDK_ASSERT && info_ok)
    {
        const assert_info_t * a = (const assert_info_t *)info;
        snprintf(err_buf, sizeof(err_buf),
                 "\r\n[FAULT] ASSERT id=0x%08X pc=0x%08X line=%u\r\n",
                 (unsigned int)id, (unsigned int)pc, (unsigned int)a->line_num);
    }
    else
    {
        snprintf(err_buf, sizeof(err_buf),
                 "\r\n[FAULT] id=0x%08X pc=0x%08X info=0x%08X\r\n",
                 (unsigned int)id, (unsigned int)pc, (unsigned int)info);
    }
    uart_puts(err_buf);
    SEGGER_RTT_WriteString(0, err_buf);

#if EXP_HALT_ON_FAULT
    while (1)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);
    }
#else
    NVIC_SystemReset();
    while (1) {}
#endif
}

nrfx_wdt_channel_id m_channel_id;

void wd_event_handle(void) {}

void wdt_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrfx_wdt_config_t w_config = NRFX_WDT_DEAFULT_CONFIG;
    err_code = nrfx_wdt_init(&w_config, wd_event_handle);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable();
}

#define resetProvisionLED     18
#define resetProvisionBTN     13

bool checktoResetProvisioned(void)
{
    nrf_gpio_cfg_output(resetProvisionLED);
    nrf_gpio_cfg_input(resetProvisionBTN, NRF_GPIO_PIN_PULLUP);
    bool _checkresetProvision = false;
    uint64_t resetProvisionCounter = 0;
    while (!_checkresetProvision)
    {
        nrf_delay_ms(1);
        if (nrf_gpio_pin_read(resetProvisionBTN) == 0)
        {
            nrf_gpio_pin_clear(resetProvisionLED);
            resetProvisionCounter++;
            if (resetProvisionCounter > 5000)
            {
                _checkresetProvision = true;
            }
        }
        else
        {
            break;
        }
    }
    return _checkresetProvision;
}

void display_ClientStatus(void)
{
    if (clientProvisionChanged)
    {
        if (clientProvisioned)
        {
            char _buff[100];
            sprintf(_buff, "0x%04x", node_address.address_start);
            glcd_display_stationAddress(_buff);
        }
        else
        {
            clearSavedExtEEPROMNodes();
            clearNodesScreen();
            display_nodes();
            glcd_display_stationAddress("------");
        }
        clientProvisionChanged = false;
    }
}

#if EXP_READ_CFG
static bool model_pub_dst_get(access_model_handle_t h, uint16_t * p_dst)
{
    dsm_handle_t dh = DSM_HANDLE_INVALID;
    nrf_mesh_address_t a;
    if (access_model_publish_address_get(h, &dh) == NRF_SUCCESS &&
        dh != DSM_HANDLE_INVALID &&
        dsm_address_get(dh, &a) == NRF_SUCCESS)
    {
        *p_dst = a.value;
        return true;
    }
    return false;
}
#endif

static void exp_cfg_capture(void)
{
    m_cfg_ttl_pub  = 0xFF;
    m_cfg_rtx      = 0xFF;
    m_cfg_txcnt    = 0xFF;
    m_cfg_dbm      = 127;
    m_cfg_ttl_eff  = 0xFF;
    m_cfg_relay_tx = 0xFF;
    m_cfg_relay_en = 0xFF;
    m_cfg_srv_pub  = -1;
    m_cfg_cli_pub  = -1;
    m_tx_dst       = 0xFFFF;

    if (!mesh_stack_is_device_provisioned())
    {
        return;
    }

#if EXP_READ_CFG
    (void)access_model_publish_ttl_get(m_clients[0].model_handle, &m_cfg_ttl_pub);
    m_cfg_ttl_eff = (m_cfg_ttl_pub == 0xFF) ? access_default_ttl_get() : m_cfg_ttl_pub;

    access_publish_retransmit_t rtx;
    if (access_model_publish_retransmit_get(m_clients[0].model_handle, &rtx) == NRF_SUCCESS)
    {
        m_cfg_rtx = rtx.count;
    }

    mesh_opt_core_adv_t adv_r;
    radio_tx_power_t pw;
    if (mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &adv_r) == NRF_SUCCESS)
    {
        m_cfg_txcnt = (uint8_t)adv_r.tx_count;
    }
    if (mesh_opt_core_tx_power_get(CORE_TX_ROLE_ORIGINATOR, &pw) == NRF_SUCCESS)
    {
        m_cfg_dbm = (int8_t)pw;
    }

    mesh_opt_core_adv_t adv_rl;
    if (mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &adv_rl) == NRF_SUCCESS)
    {
        m_cfg_relay_tx = (uint8_t)adv_rl.tx_count;
        m_cfg_relay_en = adv_rl.enabled ? 1 : 0;
    }

    uint16_t dst = 0xFFFF;
    m_cfg_srv_pub = model_pub_dst_get(m_level_server_0.server.model_handle, &dst) ? 1 : 0;
    if (model_pub_dst_get(m_clients[0].model_handle, &dst))
    {
        m_cfg_cli_pub = 1;
        m_tx_dst = dst;
    }
    else
    {
        m_cfg_cli_pub = 0;
    }
#endif
}

/* Giu nguyen cac truong cu cua # CFG, chi them cal_tx/cal_phase/relay_en o cuoi de tuong thich script cu */
static void exp_cfg_line(char * out, size_t n)
{
    snprintf(out, n,
             "# CFG exp=%u;fw=%s;role=%s;num_msgs=%u;period_ms=%u;payload_len=%u;hops=%u;"
             "ttl_cfg=%u;ttl_pub=%u;ttl_eff=%u;pub_rtx=%u;net_tx_cfg=%u;net_tx_now=%u;tx_dbm_cfg=%d;tx_dbm_now=%d;relay_tx_now=%u;"
             "chain_idx=%d;mac=%02X%02X%02X%02X%02X%02X;"
             "opts=%s;srv_pub=%d;cli_pub=%d;cli_dst=0x%04X;tx_fail=%u;last_err=0x%08X;reset=0x%08X;"
             "relay_en=%u;cal_tx=%u;cal_phase=%u",
             (unsigned)EXP_ID, FW_GIT_HASH, m_role_tx ? "TX" : "RX",
             (unsigned)EXP_NUM_MSGS, (unsigned)EXP_PERIOD_MS, (unsigned)EXP_PAYLOAD_LEN, (unsigned)g_exp_hops,
             (unsigned)EXP_TTL, (unsigned)m_cfg_ttl_pub, (unsigned)m_cfg_ttl_eff, (unsigned)m_cfg_rtx,
             (unsigned)EXP_NET_TX_COUNT, (unsigned)m_cfg_txcnt,
             (int)EXP_TX_POWER_DBM, (int)m_cfg_dbm, (unsigned)m_cfg_relay_tx,
             (int)m_my_chain_index,
             m_my_ble_addr.addr[5], m_my_ble_addr.addr[4], m_my_ble_addr.addr[3],
             m_my_ble_addr.addr[2], m_my_ble_addr.addr[1], m_my_ble_addr.addr[0],
             "read_only",
             m_cfg_srv_pub, m_cfg_cli_pub, (unsigned)m_tx_dst,
             (unsigned)m_tx_fail_total, (unsigned)m_last_err, (unsigned)g_reset_reason,
             (unsigned)m_cfg_relay_en, (unsigned)m_cal_tx_cnt, (unsigned)m_cal_phase);
}

static bool exp_verify_tx_cfg(void)
{
#if EXP_READ_CFG
    bool ttl_bad = (EXP_TTL != 0xFF) && (m_cfg_ttl_eff != EXP_TTL);
    bool rtx_bad = (m_cfg_rtx != 0);
    if (ttl_bad || rtx_bad)
    {
        SEGGER_RTT_printf(0, "# [LOI] Publish sai: ttl=%u (can %u), rtx=%u (can 0).\r\n",
                          (unsigned)m_cfg_ttl_eff, (unsigned)EXP_TTL, (unsigned)m_cfg_rtx);
        return false;
    }
    if ((EXP_NET_TX_COUNT != 0xFF) && (m_cfg_txcnt != 0xFF) && (m_cfg_txcnt != EXP_NET_TX_COUNT))
    {
        SEGGER_RTT_printf(0, "# [LOI] Network Transmit Count=%u (can %u).\r\n",
                          (unsigned)m_cfg_txcnt, (unsigned)EXP_NET_TX_COUNT);
#if EXP_STRICT_CFG
        return false;
#endif
    }
#endif
    return true;
}

static bool exp_verify_relay_cfg(void)
{
#if EXP_READ_CFG && EXP_STRICT_CFG
    if (m_my_chain_index > 0 && m_my_chain_index < EXP_GW_INDEX)
    {
        if (m_cfg_relay_en != 1 || m_cfg_relay_tx != EXP_RELAY_TX_COUNT)
        {
            SEGGER_RTT_printf(0, "# [LOI] Relay sai cau hinh stock: relay_en=%u (can 1), relay_tx=%u (can %u).\r\n",
                              (unsigned)m_cfg_relay_en, (unsigned)m_cfg_relay_tx, (unsigned)EXP_RELAY_TX_COUNT);
            return false;
        }
    }
#endif
    return true;
}

static void exp_clear_ram_state(void)
{
    m_offline_count   = 0;
    m_offline_dropped = 0;
    m_tx_packet_count = 0;
    m_global_seq      = 0;
    m_cal_tx_cnt      = 0;
    m_cal_seq         = EXP_CAL_SEQ_BASE;
    m_cal_phase       = CAL_PHASE_IDLE;
    m_trig_cnt[1]     = 0;
    m_trig_cnt[2]     = 0;
    m_trig_cnt[3]     = 0;
    m_logging_enabled = true;
}

#if EXP_HOPS_RUNTIME
static void exp_hops_cycle(void)
{
    if (periodic_message_allowed || (m_cal_remaining > 0) || (m_offline_count > 0) || (m_cal_phase != CAL_PHASE_IDLE))
    {
        SEGGER_RTT_WriteString(0, "# [LOI] Dang chay hoac con log chua xa -> khong doi so chang.\r\n");
        return;
    }

    g_exp_hops = (uint8_t)((g_exp_hops >= 4) ? 1 : (g_exp_hops + 1));
    exp_cfg_capture();
    exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
    SEGGER_RTT_WriteString(0, m_cfg_buf);
    SEGGER_RTT_printf(0, "\r\n# [INFO] Cau hinh %u chang | ttl_rx du kien tai GW = %d\r\n",
                      (unsigned)g_exp_hops, (int)EXP_TTL - ((int)g_exp_hops - 1));

    hal_led_blink_stop();
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    if (!exp_verify_relay_cfg())
    {
        hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
    }
    else
    {
        hal_led_blink_ms(BLUE_LED_pos, 300, g_exp_hops);
    }
}
#endif

static void exp_start_cal_burst(void)
{
    if (periodic_message_allowed || m_cal_remaining > 0)
    {
        SEGGER_RTT_WriteString(0, "# [LOI] Dang phat, khong the thao tac Nut 4.\r\n");
        return;
    }
    if (!mesh_stack_is_device_provisioned())
    {
        SEGGER_RTT_WriteString(0, "# [LOI] Chua provision.\r\n");
        return;
    }

    /* Neu vua dung giua chung (ABORTED) hoac da xong POST ma muon huy -> Nut 4 xoa sach RAM */
    if (m_cal_phase == CAL_PHASE_ABORTED || m_cal_phase == CAL_PHASE_POST_DONE)
    {
        exp_clear_ram_state();
        hal_led_blink_ms(BLUE_LED_pos, 150, 1);
        SEGGER_RTT_WriteString(0, "# [ACTION] Nut 4 (N4): Da xoa sach RAM va dua ve CAL_PHASE_IDLE.\r\n");
        return;
    }

    if (m_cal_phase == CAL_PHASE_IDLE)
    {
        if (m_offline_count > 0)
        {
            SEGGER_RTT_printf(0, "# [LOI] Con %u dong log cu -> bam Nut 3 xa log truoc khi CAL_PRE.\r\n",
                              (unsigned)m_offline_count);
            return;
        }
        exp_cfg_capture();
        if (!exp_verify_tx_cfg())
        {
            hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
            return;
        }
        m_role_tx         = true;
        m_logging_enabled = true;
        m_cal_seq         = EXP_CAL_SEQ_BASE;
        m_cal_tx_cnt      = 0;
        m_tx_fail_total   = 0;
        m_tx_fail_consec  = 0;
        m_trig_cnt[1]     = 0;
        m_trig_cnt[2]     = 0;
        m_trig_cnt[3]     = 0;
        m_cal_remaining   = EXP_CAL_BURST_MSGS;
        SEGGER_RTT_printf(0, "# [ACTION] Bat dau CAL_PRE (%u goi, seq=%u..%u).\r\n",
                          (unsigned)EXP_CAL_BURST_MSGS,
                          (unsigned)(EXP_CAL_SEQ_BASE + 1),
                          (unsigned)(EXP_CAL_SEQ_BASE + EXP_CAL_BURST_MSGS));
    }
    else if (m_cal_phase == CAL_PHASE_MAIN_DONE)
    {
        m_logging_enabled = true;
        m_cal_remaining   = EXP_CAL_BURST_MSGS;
        SEGGER_RTT_printf(0, "# [ACTION] Bat dau CAL_POST (%u goi).\r\n", (unsigned)EXP_CAL_BURST_MSGS);
    }
    else
    {
        SEGGER_RTT_printf(0, "# [LOI] Trang thai cal_phase=%u: hay bam Nut 2 chay 1500 goi chinh (hoac Nut 3 xa log).\r\n",
                          (unsigned)m_cal_phase);
    }
}

static void exp_toggle_run(void)
{
    if (periodic_message_allowed)
    {
        periodic_message_allowed = false;
        m_cal_phase = CAL_PHASE_ABORTED;
        SEGGER_RTT_WriteString(0, "# [INFO] Da dung phat giua chung (ABORTED). Bam Nut 3 (xa) hoac Nut 4 (xoa RAM) truoc khi chay lai.\r\n");
        return;
    }
    if (m_cal_remaining > 0)
    {
        SEGGER_RTT_WriteString(0, "# [LOI] Dang phat dot CAL, hay cho xong.\r\n");
        return;
    }
    if (!mesh_stack_is_device_provisioned())
    {
        SEGGER_RTT_WriteString(0, "# [LOI] Chua provision.\r\n");
        return;
    }
#if EXP_HOP_WHITELIST
    if (m_my_chain_index == -1)
    {
        SEGGER_RTT_WriteString(0, "# [LOI] MAC khong co trong EXP_TOPOLOGY_CHAIN!\r\n");
        return;
    }
#endif

    exp_cfg_capture();
    bool is_tx = (EXP_READ_CFG == 0) || (m_cfg_cli_pub == 1);

    if (!is_tx)
    {
        if (!exp_verify_relay_cfg())
        {
            hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
            return;
        }
        m_role_tx = false;
        exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
        SEGGER_RTT_WriteString(0, m_cfg_buf);
        SEGGER_RTT_WriteString(0, "\r\n# [ACTION] BO THU/RELAY: cau hinh OK & san sang nhan.\r\n");
        return;
    }

    bool ok_to_start = ((m_cal_phase == CAL_PHASE_IDLE) && (m_offline_count == 0)) ||
                       (m_cal_phase == CAL_PHASE_PRE_DONE);
    if (!ok_to_start)
    {
        SEGGER_RTT_printf(0, "# [LOI] Khong the bat run (cal_phase=%u, log=%u). "
                             "Bam Nut 3 (xa) hoac Nut 4 (xoa RAM / CAL_POST) truoc.\r\n",
                          (unsigned)m_cal_phase, (unsigned)m_offline_count);
        return;
    }

    if (!exp_verify_tx_cfg())
    {
        hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
        return;
    }

    if (m_cal_phase == CAL_PHASE_IDLE)
    {
        m_trig_cnt[1] = 0;
        m_trig_cnt[2] = 0;
        m_trig_cnt[3] = 0;
    }

    m_offline_dropped = 0;
    m_logging_enabled = true;
    m_tx_packet_count = 0;
    m_global_seq      = 0;
    m_tx_fail_total   = 0;
    m_tx_fail_consec  = 0;
    m_last_err        = 0;
    m_run_done        = false;
    m_run_aborted     = false;
    m_role_tx         = true;

    exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
    SEGGER_RTT_WriteString(0, m_cfg_buf);
    SEGGER_RTT_WriteString(0, "\r\n");
    SEGGER_RTT_printf(0, "# [ACTION] Bat dau run TN2 (%u goi chinh, cal_phase=%u)\r\n",
                      (unsigned)EXP_NUM_MSGS, (unsigned)m_cal_phase);

    periodic_message_allowed = true;
}

static void exp_dump_log(void)
{
    bool aborted = false;
    uint16_t n = m_offline_count;

    m_logging_enabled = false;
    periodic_message_allowed = false;
    m_cal_remaining = 0;

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    exp_cfg_capture();
    exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
    if (!rtt_write_line_reliable(m_cfg_buf, (unsigned)strlen(m_cfg_buf)) ||
        !rtt_write_line_reliable("\r\n", 2))
    {
        aborted = true;
    }

    if (!aborted)
    {
        int l = snprintf(m_cfg_buf, sizeof(m_cfg_buf),
                         "# TRIG enabled=%u;tx_pulse=%u;fwd_pulse=%u;rx_pulse=%u;tx_ok=%u;cal_tx=%u;tx_fail=%u;"
                         "width_us=%u;gap_us=%u;hops=%u;ttl_rx_exp=%d\r\n",
                         (unsigned)EXP_ENABLE_TRIG,
                         (unsigned)m_trig_cnt[EXP_TRIG_TX], (unsigned)m_trig_cnt[EXP_TRIG_FWD],
                         (unsigned)m_trig_cnt[EXP_TRIG_RX],
                         (unsigned)m_tx_packet_count, (unsigned)m_cal_tx_cnt, (unsigned)m_tx_fail_total,
                         (unsigned)TRIG_WIDTH_US, (unsigned)TRIG_GAP_US,
                         (unsigned)g_exp_hops, (int)EXP_TTL - ((int)g_exp_hops - 1));
        if (l > 0 && !rtt_write_line_reliable(m_cfg_buf, (unsigned)l))
        {
            aborted = true;
        }
    }

#if EXP_HOP_WHITELIST
    if (!aborted && exp_is_dest())
    {
        const int exp_ttl_rx = (int)EXP_TTL - ((int)g_exp_hops - 1);
        uint32_t rx_rows = 0, rx_ok = 0;
        for (uint16_t i = 0; i < n; i++)
        {
            uint8_t  evt = UNPACK_EVT(m_offline_logs[i].ttl_evt);
            uint16_t seq = m_offline_logs[i].seq_alarm & 0x7FFFu;
            if (evt == EXP_EVT_RX && seq < EXP_CAL_SEQ_BASE)
            {
                rx_rows++;
                if ((int)UNPACK_TTL(m_offline_logs[i].ttl_evt) == exp_ttl_rx)
                {
                    rx_ok++;
                }
            }
        }
        const uint32_t pct10 = (rx_rows > 0) ? ((rx_ok * 1000u) / rx_rows) : 0u;
        int l = snprintf(m_cfg_buf, sizeof(m_cfg_buf),
                         "# HOPCHECK hops=%u;ttl_rx_exp=%d;match=%u;rx_rows=%u;pct=%u.%u;verdict=%s\r\n",
                         (unsigned)g_exp_hops, exp_ttl_rx, (unsigned)rx_ok, (unsigned)rx_rows,
                         (unsigned)(pct10 / 10u), (unsigned)(pct10 % 10u),
                         (rx_rows > 0 && pct10 >= 950u) ? "PASS" : "FAIL");
        if (l > 0 && !rtt_write_line_reliable(m_cfg_buf, (unsigned)l))
        {
            aborted = true;
        }
    }
#endif

    int len = snprintf(m_line_buf, sizeof(m_line_buf),
                       "--- BAT DAU XA LOG CSV --- SO DONG: %u; BI TRAN BUFFER: %u ---\r\n",
                       (unsigned)n, (unsigned)m_offline_dropped);
    if (!aborted && len > 0 && !rtt_write_line_reliable(m_line_buf, (unsigned)len))
    {
        aborted = true;
    }

    /* 11 cot dau GIU NGUYEN thu tu cu; net_seq16 noi o cot 12 de khong lech logger_v2/analyze_v2 */
    static const char header[] =
        "t_local_us,node_id,event,src,dst,seq,ttl_tx,ttl_rx,rssi,class,payload_len,net_seq16\r\n";
    if (!aborted && !rtt_write_line_reliable(header, sizeof(header) - 1))
    {
        aborted = true;
    }

    uint16_t my_id = (uint16_t)(node_address.address_start & 0xFFFFu);

    for (uint16_t i = 0; (i < n) && !aborted; i++)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);

        uint16_t seq      = m_offline_logs[i].seq_alarm & 0x7FFFu;
        bool     is_alarm = (m_offline_logs[i].seq_alarm >> 15) & 0x01u;
        uint8_t  evt      = UNPACK_EVT(m_offline_logs[i].ttl_evt);
        uint8_t  ttl_val  = UNPACK_TTL(m_offline_logs[i].ttl_evt);
        uint16_t nseq16   = m_offline_logs[i].net_seq16;

        const char * evt_str = (evt == EXP_EVT_TX) ? "TX" :
                               ((evt == EXP_EVT_RELAY) ? "RELAY" : "RX");

        uint16_t src      = (evt == EXP_EVT_TX) ? my_id : m_offline_logs[i].src;
        uint16_t dst      = (evt == EXP_EVT_RX) ? my_id : m_tx_dst;
        uint8_t  ttl_tx   = (evt == EXP_EVT_TX) ? ttl_val :
                            ((evt == EXP_EVT_RELAY && ttl_val > 0) ? (uint8_t)(ttl_val - 1u) : 0u);
        uint8_t  ttl_rx   = (evt == EXP_EVT_TX) ? 0u : ttl_val;
        int      rssi_out = (evt == EXP_EVT_TX) ? 0 : (int)m_offline_logs[i].rssi;
        const char * cls  = (evt == EXP_EVT_RELAY) ? "RELAY" :
                            ((seq >= EXP_CAL_SEQ_BASE) ? "CAL" :
                             (is_alarm ? "ALARM" : "TELEMETRY"));

        len = snprintf(m_line_buf, sizeof(m_line_buf),
                       "%u,0x%04X,%s,0x%04X,0x%04X,%u,%u,%u,%d,%s,%u,%u\r\n",
                       (unsigned)m_offline_logs[i].t_us,
                       (unsigned)my_id,
                       evt_str,
                       (unsigned)src,
                       (unsigned)dst,
                       (unsigned)seq,
                       (unsigned)ttl_tx,
                       (unsigned)ttl_rx,
                       rssi_out,
                       cls,
                       (unsigned)EXP_PAYLOAD_LEN,
                       (unsigned)nseq16);

        if (len > 0 && !rtt_write_line_reliable(m_line_buf, (unsigned)len))
        {
            aborted = true;
        }
    }

    if (aborted)
    {
        SEGGER_RTT_WriteString(0, "\r\n[LOI] RTT bi nghen! Du lieu van giu trong RAM, hay bam Nut 3 lai.\r\n");
        return;
    }

    (void)rtt_write_line_reliable("--- KET THUC XA LOG ---\r\n", sizeof("--- KET THUC XA LOG ---\r\n") - 1);
    exp_clear_ram_state();
}

static void exp_requests_service(void)
{
    /* Tu kiem tra cau hinh (dac biet la relay_tx=3 & hops=) ngay sau khi khoi dong 2s */
    if (!m_boot_cfg_checked && (m_secondCounter >= 2) && mesh_stack_is_device_provisioned())
    {
        m_boot_cfg_checked = true;
        exp_cfg_capture();
        m_role_tx = exp_is_originator();
        exp_cfg_line(m_cfg_buf, sizeof(m_cfg_buf));
        SEGGER_RTT_WriteString(0, m_cfg_buf);
        SEGGER_RTT_WriteString(0, "\r\n");
        if (!exp_verify_relay_cfg())
        {
            hal_led_blink_ms(YELLOW_LED_pos, 100, 6);
        }
    }

#if EXP_HOPS_RUNTIME
    if (m_req_hops)
    {
        m_req_hops = false;
        exp_hops_cycle();
    }
#endif

    if (m_req_cal)
    {
        m_req_cal = false;
        exp_start_cal_burst();
    }

    if (m_req_clear_ram)
    {
        m_req_clear_ram = false;
        if (!periodic_message_allowed && m_cal_remaining == 0)
        {
            exp_clear_ram_state();
            hal_led_blink_ms(BLUE_LED_pos, 150, 1);
            SEGGER_RTT_WriteString(0, "# [ACTION] Nut 4: Da xoa sach bo dem RAM tren Relay/GW.\r\n");
        }
    }

    if (m_req_toggle)
    {
        m_req_toggle = false;
        exp_toggle_run();
    }

    if (m_cal_done)
    {
        m_cal_done = false;
        if (m_cal_phase == CAL_PHASE_IDLE)
        {
            m_cal_phase = CAL_PHASE_PRE_DONE;
            SEGGER_RTT_printf(0, "# [INFO] Xong CAL_PRE (%u goi). Rut day PPK2, dat ra hanh lang roi bam Nut 2.\r\n",
                              (unsigned)EXP_CAL_BURST_MSGS);
        }
        else if (m_cal_phase == CAL_PHASE_MAIN_DONE)
        {
            m_cal_phase = CAL_PHASE_POST_DONE;
            SEGGER_RTT_printf(0, "# [INFO] Xong CAL_POST (%u goi). Cam J-Link va bam Nut 3 de xa log.\r\n",
                              (unsigned)EXP_CAL_BURST_MSGS);
        }
        hal_led_blink_stop();
        hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
        hal_led_blink_ms(BLUE_LED_pos, 200, 2);
    }

    if (m_run_done)
    {
        m_run_done = false;
        if (m_cal_phase == CAL_PHASE_PRE_DONE)
        {
            m_cal_phase = CAL_PHASE_MAIN_DONE;
            SEGGER_RTT_printf(0, "# [INFO] Du %u goi MAIN. Be cac node ve ban, cam PPK2 roi bam Nut 4 de CAL_POST.\r\n",
                              (unsigned)EXP_NUM_MSGS);
        }
        else
        {
            SEGGER_RTT_printf(0, "# [INFO] Du %u goi. Bam Nut 3 de xa log.\r\n", (unsigned)EXP_NUM_MSGS);
        }
    }

    if (m_run_aborted)
    {
        m_run_aborted = false;
        SEGGER_RTT_printf(0, "# [LOI] Gui loi %u lan lien tiep (err=0x%08X). Dung run (ABORTED).\r\n",
                          (unsigned)EXP_MAX_TX_FAIL, (unsigned)m_last_err);
    }

    if (m_dump_requested)
    {
        SEGGER_RTT_WriteString(0, "[ACTION] Nhan Nut 3 -> Dang xa log...\r\n");
        exp_dump_log();
        m_dump_requested = false;
    }
}

int main(void)
{
    log_config_init();
    _getResetProvisionKey = checktoResetProvisioned();

    all_parameters_initialization();
    uart_init();
    mq2_saadc_init();

    glcd_display_initialization();
    glcd_printText("Nodes:", 1, 40, TFT_WHITE, TFT_BLACK);

    initialize();
    start();

    wdt_init();
    custom_timer_init();
    egu_init();
    twi_init();

    static uint8_t _i2cDevices[255];
    detect_i2cDevice(_i2cDevices);
    loadExtEEPROM_Nodes();

    nrf_drv_wdt_channel_feed(m_channel_id);

    if (aht10_begin() == true)
    {
        aht10Available = true;
    }

    if (mq2Available)
    {
        _clientMQ2Value = read_mq2_sensor();
    }

    if (aht10Available)
    {
        _clientTemperature = aht10_readTemperature(AHT10_FORCE_READ_DATA);
        _clientHumidity = aht10_readHumidity(false);
        glcd_printSensor(_clientTemperature, _clientHumidity, _clientMQ2Value);
    }

    if (_getResetProvisionKey)
    {
        trigger_egu30_event();
    }

    SEGGER_RTT_WriteString(0, "-> Vao vong lap for(;;) TN2 thanh cong!\r\n");

    for (;;)
    {
        exp_requests_service();

        if (m_second_changed)
        {
            /* Phat goi ngay dau tick de triet tieu jitter 140 ms tu LCD/AHT10 */
            if ((periodic_message_allowed || m_cal_remaining > 0) && mesh_stack_is_device_provisioned())
            {
                trigger_egu30_event();
            }

            (void)get_local_time_us();   /* keep-alive bo cong don RTC (< 512 s) */

            char rtt_buf[112];
            const char * mode = (m_cal_remaining > 0) ? "CAL" :
                                (periodic_message_allowed ? "TX=RUN" : (m_role_tx ? "TX=IDLE" : "RX"));
            snprintf(rtt_buf, sizeof(rtt_buf), "[TICK] s=%u | %s sent=%u cal=%u | LOG=%u | hops=%u ph=%u\r\n",
                     (unsigned)m_secondCounter, mode,
                     (unsigned)m_tx_packet_count, (unsigned)m_cal_tx_cnt,
                     (unsigned)m_offline_count, (unsigned)g_exp_hops, (unsigned)m_cal_phase);
            SEGGER_RTT_WriteString(0, rtt_buf);

            glcd_integer_print(m_secondCounter);
            display_ClientStatus();

            if (clearAllDisplayNodes)
            {
                clearSavedExtEEPROMNodes();
                clearNodesScreen();
                display_nodes();
                clearAllDisplayNodes = false;
            }

            bool is_timeout = check_connection_nodes();
            bool is_poll_5s = (m_secondCounter % 5 == 0);

            if (!is_timeout && is_poll_5s)
            {
                if (mq2Available)
                {
                    _clientMQ2Value = read_mq2_sensor();
                }

                if (aht10Available)
                {
                    _clientTemperature = aht10_readTemperature(AHT10_FORCE_READ_DATA);
                    _clientHumidity = aht10_readHumidity(false);
                    glcd_printSensor(_clientTemperature, _clientHumidity, _clientMQ2Value);
                }
                else
                {
                    aht10Available = aht10_begin();
                }
            }

            if ((refreshDisplayCounter > 0) || is_timeout || is_poll_5s)
            {
                display_nodes();
                refreshDisplayCounter = 0;
            }

            m_second_changed = false;
        }

        nrf_drv_wdt_channel_feed(m_channel_id);
        (void)sd_app_evt_wait();
    }
}
