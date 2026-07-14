#include <GNSS/MAXM10S.h>
#include <Radio/SX1272.h>
#include <CAN/CAN_Frames.h>
#include <SPI/SPI_STM.h>
#include <UART/UART_STM.h>
#include <ogma_radio_protocol.h>
#include <teachtaire_radio_config.h>

#include <stm32f0xx_hal.h>

#include <cstdint>
#include <cstddef>

// Must match teammate ground-station decoder:
//
//     struct.unpack("<iihhBB", buf)
//
// Packet length = 14 bytes:
//     int32 latitude_e7
//     int32 longitude_e7
//     int16 altitude_dm       altitude metres * 10
//     int16 velocity_dm_s     velocity m/s * 10
//     uint8 satellites
//     uint8 gps_valid
constexpr std::size_t OGMA_GPS_PACKET_LEN = 14U;
#if defined(TEACHTAIRE_LORA_TX_TEST) && defined(TEACHTAIRE_LORA_RX_TEST)
#error "Select one Teachtaire LoRa test mode"
#endif

#if defined(TEACHTAIRE_LORA_RX_TEST)
constexpr bool LORA_RECEIVE_ONLY = true;
#else
constexpr bool LORA_RECEIVE_ONLY = false;
#endif

#if defined(TEACHTAIRE_LORA_TX_TEST)
constexpr bool LORA_TRANSMIT_TEST = true;
#else
constexpr bool LORA_TRANSMIT_TEST = false;
#endif

struct teachtaire_report_t {
    uint32_t magic;
    uint32_t loops;
    uint32_t clock_hz;

    uint8_t clock_ok;
    uint8_t gpio_ok;
    uint8_t spi_ok;
    uint8_t uart_ok;

    uint8_t lora_init_ok;
    uint8_t lora_rx_ok;
    uint8_t lora_version;
    uint8_t lora_error;
    uint8_t lora_irq;
    int16_t lora_rssi_dbm;
    int8_t lora_snr_db;
    uint32_t lora_tx_count;
    uint32_t lora_tx_done_count;
    uint32_t lora_rx_count;
    uint32_t lora_rx_bad_count;
    uint32_t lora_last_counter;

    uint32_t gnss_seen;
    uint32_t gnss_parsed;
    uint32_t gnss_checksum_bad;
    uint32_t gnss_bytes;
    uint32_t gnss_starts;
    uint32_t gnss_overflows;
    uint32_t gnss_txt;
    uint32_t gnss_nav_sat;
    uint8_t gnss_fix;
    uint8_t gnss_sats;
    uint8_t gnss_sats_in_view;
    uint8_t gnss_ant_status;
    uint8_t gnss_nav_sat_count;
    uint8_t gnss_nav_sat_signal;
    uint8_t gnss_nav_sat_max_cno;
    uint8_t version;
    int32_t gnss_latitude_e7;
    int32_t gnss_longitude_e7;
    int32_t gnss_altitude_mm;
    int32_t gnss_velocity_mm_s;

    uint32_t spi_status;
    uint32_t spi_error;
    uint32_t uart_status;
    uint32_t uart_error;
    uint32_t usart1_isr;
    uint32_t gpioa_idr;
    uint32_t gpioa_odr;
    uint32_t gpiob_idr;
    uint32_t gpiob_odr;
    uint32_t fault;
    uint32_t clock_source;
    uint32_t can_init_ok;
    uint32_t can_bus_off;
    uint32_t can_error;
    uint32_t can_tx_count;
    uint32_t can_rx_count;
    uint32_t can_tx_drops;
    uint32_t can_tx_queue_depth;
    uint32_t heartbeat_tx_count;
    uint32_t gps_can_tx_count;
    uint32_t tx_status_can_tx_count;
    uint32_t croi_last_seen_ms;
    uint32_t can_esr;
    uint32_t lora_reinit_count;
    uint32_t lora_tx_timeout_count;
    uint32_t gnss_last_fix_ms;
    uint32_t gnss_fix_age_ms;
    uint32_t can_rx_overruns;
    uint32_t watchdog_init_ok;
    uint32_t watchdog_refresh_count;
    uint32_t reset_flags;
    uint32_t telemetry_core_tx_count;
    uint32_t telemetry_gps_tx_count;
    uint32_t telemetry_slow_tx_count;
    uint32_t telemetry_event_tx_count;
    uint32_t telemetry_deep_tx_count;
    uint32_t telemetry_event_drop_count;
    uint32_t gnss_uart_overrun_recoveries;
    uint32_t radio_config_magic;
    uint32_t radio_config_schema_version;
    uint32_t radio_config_crc32;
};
static_assert(sizeof(teachtaire_report_t) == 268U,
              "teachtaire_report_t wire contract changed");

extern "C" {

struct OgmaBoardIdentity {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t struct_size;
    uint32_t board_id;
    uint32_t capabilities;
    uint32_t firmware_version;
    uint32_t firmware_build;
    uint32_t reserved0;
    uint32_t reserved1;
};

__attribute__((used)) volatile OgmaBoardIdentity ogma_board_identity{
    0x4F474944U,
    1U,
    sizeof(OgmaBoardIdentity),
    0x04U,
    0x19U,
    20260714U,
    0U,
    0U,
    0U,
};

volatile teachtaire_report_t report{};
}

namespace {

constexpr uint32_t REPORT_MAGIC = 0x54434854U; // 'TCHT'

constexpr uint16_t LORA_RESET_PIN = GPIO_PIN_1;
constexpr uint16_t LORA_NSS_PIN = GPIO_PIN_2;
constexpr uint16_t LORA_SCK_PIN = GPIO_PIN_5;
constexpr uint16_t LORA_MISO_PIN = GPIO_PIN_6;
constexpr uint16_t LORA_MOSI_PIN = GPIO_PIN_7;
constexpr uint16_t LORA_RX_SWITCH_PIN = GPIO_PIN_8;
constexpr uint16_t LORA_TX_SWITCH_PIN = GPIO_PIN_15;

constexpr uint16_t GNSS_RESET_PIN = GPIO_PIN_8;
constexpr uint16_t GNSS_TX_PIN = GPIO_PIN_9;
constexpr uint16_t GNSS_RX_PIN = GPIO_PIN_10;
constexpr uint32_t CAN_HEARTBEAT_PERIOD_MS = 1000U;
constexpr uint32_t CAN_TX_STATUS_PERIOD_MS = 1000U;
constexpr uint32_t CAN_BUS_RECOVERY_PERIOD_MS = 250U;
constexpr uint32_t CAN_CROI_TIMEOUT_MS = 5000U;
constexpr uint32_t LORA_REINIT_PERIOD_MS = 2000U;
constexpr uint32_t LORA_TX_TIMEOUT_MS = 5000U;
constexpr uint32_t GNSS_FIX_TIMEOUT_MS = 5000U;
constexpr uint32_t GPS_CAN_PERIOD_MS = 1000U;
constexpr uint32_t RADIO_CORE_PERIOD_MS = TEACHTAIRE_RADIO_CORE_PERIOD_MS;
constexpr uint32_t RADIO_GPS_PERIOD_MS = TEACHTAIRE_RADIO_GPS_PERIOD_MS;
constexpr uint32_t RADIO_SLOW_PERIOD_MS = TEACHTAIRE_RADIO_SLOW_PERIOD_MS;
constexpr uint32_t RADIO_DEEP_PERIOD_MS = TEACHTAIRE_RADIO_DEEP_PERIOD_MS;
constexpr uint32_t RADIO_CACHE_MAX_AGE_MS = 5000U;
constexpr uint8_t RADIO_EVENT_QUEUE_LEN = 8U;
constexpr uint8_t CAN_TX_QUEUE_LEN = 8U;
constexpr uint8_t CAN_TX_DRAIN_BUDGET = 3U;

SPI_HandleTypeDef hspi1{};
UART_HandleTypeDef huart1{};
CAN_HandleTypeDef hcan{};
IWDG_HandleTypeDef hiwdg{};

struct CANFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

struct CachedCanFrame {
    ogma_radio::RawCanRecord record{};
    uint32_t last_seen_ms{0U};
    bool valid{false};
};

constexpr uint16_t CORE_CAN_IDS[] = {
    CAN_ID_FLIGHT_STATE,
    CAN_ID_KALMANN,
    CAN_ID_BARO,
    CAN_ID_IMU_ACCEL,
    CAN_ID_IMU_GYRO,
};

constexpr uint16_t SLOW_CAN_IDS[] = {
    CAN_ID_POWER_MAIN,
    CAN_ID_POWER_SERVO,
    CAN_ID_PYRO_STATUS,
    CAN_ID_ACTUATOR_COMMAND,
};

constexpr std::size_t CORE_CAN_COUNT = sizeof(CORE_CAN_IDS) / sizeof(CORE_CAN_IDS[0]);

CachedCanFrame core_cache[CORE_CAN_COUNT]{};
CachedCanFrame slow_cache[4]{};
CachedCanFrame heartbeat_cache[6]{};
ogma_radio::RawCanRecord event_queue[RADIO_EVENT_QUEUE_LEN]{};
uint8_t event_head = 0U;
uint8_t event_tail = 0U;
uint8_t event_count = 0U;
std::size_t heartbeat_page_start = 0U;
std::size_t core_page_start = 0U;

bool init_watchdog()
{
#if defined(__HAL_DBGMCU_FREEZE_IWDG)
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 2499U;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    return HAL_IWDG_Init(&hiwdg) == HAL_OK;
}

bool can_ready = false;
uint32_t can_tx_count = 0U;
uint32_t can_rx_count = 0U;
uint32_t can_tx_drops = 0U;
uint32_t heartbeat_tx_count = 0U;
uint32_t gps_can_tx_count = 0U;
uint32_t tx_status_can_tx_count = 0U;
uint32_t croi_last_seen_ms = 0U;
uint32_t last_heartbeat_ms = 0U;
uint32_t last_tx_status_ms = 0U;
uint32_t last_bus_recovery_ms = 0U;
uint32_t can_rx_overruns = 0U;
CANFrame tx_queue[CAN_TX_QUEUE_LEN]{};
uint8_t tx_head = 0U;
uint8_t tx_tail = 0U;
uint8_t tx_count = 0U;

void cache_frame(CachedCanFrame& cache,
                 const CAN_RxHeaderTypeDef& header,
                 const uint8_t* data,
                 uint32_t now_ms)
{
    cache.record.id = static_cast<uint16_t>(header.StdId);
    cache.record.dlc = static_cast<uint8_t>(header.DLC);
    for (uint8_t index = 0U; index < 8U; ++index) {
        cache.record.data[index] = index < header.DLC ? data[index] : 0U;
    }
    cache.last_seen_ms = now_ms;
    cache.valid = true;
}

template<std::size_t N>
bool cache_known_frame(CachedCanFrame (&cache)[N],
                       const uint16_t (&ids)[N],
                       const CAN_RxHeaderTypeDef& header,
                       const uint8_t* data,
                       uint32_t now_ms)
{
    for (std::size_t index = 0U; index < N; ++index) {
        if (header.StdId == ids[index]) {
            cache_frame(cache[index], header, data, now_ms);
            return true;
        }
    }
    return false;
}

std::size_t collect_records(const CachedCanFrame* cache,
                            std::size_t cache_count,
                            uint32_t now_ms,
                            ogma_radio::RawCanRecord* out,
                            std::size_t max_records,
                            std::size_t start_index = 0U)
{
    if (cache_count == 0U) {
        return 0U;
    }
    std::size_t count = 0U;
    for (std::size_t scanned = 0U; scanned < cache_count && count < max_records; ++scanned) {
        const std::size_t index = (start_index + scanned) % cache_count;
        if (!cache[index].valid ||
            (now_ms - cache[index].last_seen_ms) > RADIO_CACHE_MAX_AGE_MS) {
            continue;
        }
        out[count++] = cache[index].record;
    }
    return count;
}

bool cached_payload_differs(const CachedCanFrame& cache,
                            const CAN_RxHeaderTypeDef& header,
                            const uint8_t* data,
                            uint8_t bytes_to_compare = 8U)
{
    if (!cache.valid || cache.record.id != header.StdId || cache.record.dlc != header.DLC) {
        return true;
    }
    if (header.DLC < bytes_to_compare) {
        return true;
    }
    for (uint8_t index = 0U; index < bytes_to_compare; ++index) {
        if (cache.record.data[index] != data[index]) {
            return true;
        }
    }
    return false;
}

void queue_event(const CAN_RxHeaderTypeDef& header, const uint8_t* data)
{
    if (event_count >= RADIO_EVENT_QUEUE_LEN) {
        event_head = static_cast<uint8_t>((event_head + 1U) % RADIO_EVENT_QUEUE_LEN);
        --event_count;
        ++report.telemetry_event_drop_count;
    }
    ogma_radio::RawCanRecord& record = event_queue[event_tail];
    record.id = static_cast<uint16_t>(header.StdId);
    record.dlc = static_cast<uint8_t>(header.DLC);
    for (uint8_t index = 0U; index < 8U; ++index) {
        record.data[index] = index < header.DLC ? data[index] : 0U;
    }
    event_tail = static_cast<uint8_t>((event_tail + 1U) % RADIO_EVENT_QUEUE_LEN);
    ++event_count;
}

std::size_t collect_heartbeat_records(uint32_t now_ms,
                                      ogma_radio::RawCanRecord* out,
                                      std::size_t max_records)
{
    std::size_t count = 0U;
    for (std::size_t scanned = 0U;
         scanned < 6U && count < max_records;
         ++scanned) {
        const std::size_t index = (heartbeat_page_start + scanned) % 6U;
        const CachedCanFrame& cache = heartbeat_cache[index];
        if (cache.valid && (now_ms - cache.last_seen_ms) <= RADIO_CACHE_MAX_AGE_MS) {
            out[count++] = cache.record;
        }
    }
    heartbeat_page_start = (heartbeat_page_start + max_records) % 6U;
    return count;
}

void gpio_output(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState initial)
{
    HAL_GPIO_WritePin(port, pin, initial);

    GPIO_InitTypeDef gpio{};
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);
}

void gpio_af(GPIO_TypeDef* port, uint16_t pin, uint32_t af, uint32_t pull)
{
    GPIO_InitTypeDef gpio{};
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = pull;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = af;
    HAL_GPIO_Init(port, &gpio);
}

void SystemClock_Config()
{
    auto configure_hse_48mhz = []() -> bool {
        RCC_OscInitTypeDef osc{};
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        osc.HSEState = RCC_HSE_ON;
        osc.PLL.PLLState = RCC_PLL_ON;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        osc.PLL.PREDIV = RCC_PREDIV_DIV1;
        osc.PLL.PLLMUL = RCC_PLL_MUL6;

        if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
            return false;
        }

        RCC_ClkInitTypeDef clk{};
        clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
        clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
        clk.APB1CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
            return false;
        }

        SystemCoreClockUpdate();
        report.clock_source = 1U;
        return true;
    };

    auto configure_hsi48 = []() -> bool {
#if defined(RCC_OSCILLATORTYPE_HSI48)
        RCC_OscInitTypeDef osc{};
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
        osc.HSI48State = RCC_HSI48_ON;
        osc.PLL.PLLState = RCC_PLL_NONE;

        if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
            return false;
        }

        RCC_ClkInitTypeDef clk{};
        clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
        clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
        clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
        clk.APB1CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
            return false;
        }

        SystemCoreClockUpdate();
        report.clock_source = 2U;
        return true;
#else
        return false;
#endif
    };

    if (configure_hse_48mhz() || configure_hsi48()) {
        report.clock_ok = 1U;
        report.clock_hz = SystemCoreClock;
        return;
    }

    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        report.fault = 1U;
        return;
    }

    RCC_ClkInitTypeDef clk{};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
        report.fault = 2U;
        return;
    }

    SystemCoreClockUpdate();
    report.clock_ok = 1U;
    report.clock_hz = SystemCoreClock;
    report.clock_source = 3U;
}

void MX_GPIO_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio_output(GPIOA, LORA_NSS_PIN, GPIO_PIN_SET);
    gpio_output(GPIOA, LORA_RESET_PIN, GPIO_PIN_RESET);
    gpio_output(GPIOA, LORA_RX_SWITCH_PIN, GPIO_PIN_RESET);
    gpio_output(GPIOA, LORA_TX_SWITCH_PIN, GPIO_PIN_RESET);
    gpio_output(GPIOB, GNSS_RESET_PIN, GPIO_PIN_SET);

    gpio_af(GPIOA, LORA_SCK_PIN | LORA_MISO_PIN | LORA_MOSI_PIN, GPIO_AF0_SPI1, GPIO_NOPULL);
    gpio_af(GPIOA, GNSS_TX_PIN | GNSS_RX_PIN, GPIO_AF1_USART1, GPIO_PULLUP);

    report.gpio_ok = 1U;
}

void MX_SPI1_Init()
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7U;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    report.spi_ok = (HAL_SPI_Init(&hspi1) == HAL_OK) ? 1U : 0U;
}

void MX_USART1_UART_Init()
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600U;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) == HAL_OK) {
        report.uart_ok = 1U;

        // Force HAL state to READY.
        huart1.gState = HAL_UART_STATE_READY;
        huart1.RxState = HAL_UART_STATE_READY;
    }
}

bool MX_CAN_Init()
{
    hcan.Instance = CAN;
    hcan.Init.Prescaler = 6U;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        return false;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    return HAL_CAN_ConfigFilter(&hcan, &filter) == HAL_OK;
}

bool can_bus_off()
{
    return hcan.Instance != nullptr && (hcan.Instance->ESR & CAN_ESR_BOFF) != 0U;
}

uint32_t can_error()
{
    return hcan.Instance == nullptr ? HAL_CAN_ERROR_PARAM : HAL_CAN_GetError(&hcan);
}

template<typename Payload>
CANFrame make_can_frame(uint32_t id, const Payload& payload)
{
    static_assert(sizeof(Payload) <= 8U, "CAN payload too large");
    CANFrame frame{};
    frame.id = id;
    frame.dlc = 8U;
    const auto* bytes = reinterpret_cast<const uint8_t*>(&payload);
    for (uint8_t index = 0U; index < sizeof(Payload); ++index) {
        frame.data[index] = bytes[index];
    }
    return frame;
}

CANFrame pack_gps_frame(const gps_data& gps, bool gps_valid)
{
    const int32_t lat = gps_encode(gps.latitude);
    const int32_t lon = gps_encode(gps.longitude);
    CANFrame frame{};
    frame.id = CAN_ID_GPS;
    frame.dlc = 8U;
    frame.data[0] = static_cast<uint8_t>((lat >> 16) & 0xFF);
    frame.data[1] = static_cast<uint8_t>((lat >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(lat & 0xFF);
    frame.data[3] = static_cast<uint8_t>((lon >> 16) & 0xFF);
    frame.data[4] = static_cast<uint8_t>((lon >> 8) & 0xFF);
    frame.data[5] = static_cast<uint8_t>(lon & 0xFF);
    frame.data[6] = gps.satellites;
    frame.data[7] = gps_valid ? 1U : 0U;
    return frame;
}

bool can_send_now(CANFrame& frame)
{
    if (!can_ready || can_bus_off() || frame.id > 0x7FFU || frame.dlc > 8U) {
        return false;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
        return false;
    }

    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.dlc;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0U;
    return HAL_CAN_AddTxMessage(&hcan, &header, frame.data, &mailbox) == HAL_OK;
}

bool queue_can_frame(const CANFrame& frame)
{
    if (tx_count >= CAN_TX_QUEUE_LEN) {
        tx_head = static_cast<uint8_t>((tx_head + 1U) % CAN_TX_QUEUE_LEN);
        --tx_count;
        ++can_tx_drops;
    }
    tx_queue[tx_tail] = frame;
    tx_tail = static_cast<uint8_t>((tx_tail + 1U) % CAN_TX_QUEUE_LEN);
    ++tx_count;
    return true;
}

bool send_can_frame(CANFrame& frame)
{
    if (!can_ready) {
        return false;
    }
    if (tx_count != 0U) {
        return queue_can_frame(frame);
    }
    if (can_send_now(frame)) {
        ++can_tx_count;
        return true;
    }
    return queue_can_frame(frame);
}

void flush_can_queue()
{
    if (!can_ready) {
        return;
    }
    for (uint8_t sent = 0U; sent < CAN_TX_DRAIN_BUDGET && tx_count > 0U; ++sent) {
        CANFrame& frame = tx_queue[tx_head];
        if (!can_send_now(frame)) {
            return;
        }
        tx_head = static_cast<uint8_t>((tx_head + 1U) % CAN_TX_QUEUE_LEN);
        --tx_count;
        ++can_tx_count;
    }
}

void process_can_frame(const CAN_RxHeaderTypeDef& header, const uint8_t* data, uint32_t now_ms)
{
    if (header.DLC > 8U) {
        return;
    }

    if (CAN_ID_IS_HEARTBEAT(header.StdId) && header.DLC >= 4U) {
        if (data[0] == NODE_CROI) {
            croi_last_seen_ms = now_ms;
        }
        CachedCanFrame* destination = nullptr;
        for (CachedCanFrame& cache : heartbeat_cache) {
            if (cache.valid && cache.record.data[0] == data[0]) {
                destination = &cache;
                break;
            }
            if (!cache.valid && destination == nullptr) {
                destination = &cache;
            }
        }
        if (destination != nullptr) {
            cache_frame(*destination, header, data, now_ms);
        }
        return;
    }

    bool event = header.StdId == CAN_ID_PYRO_ACK;
    if (header.StdId == CAN_ID_FLIGHT_STATE) {
        event = cached_payload_differs(core_cache[0], header, data, 2U);
    } else if (header.StdId == CAN_ID_PYRO_STATUS) {
        event = cached_payload_differs(slow_cache[2], header, data);
    }

    (void)cache_known_frame(core_cache, CORE_CAN_IDS, header, data, now_ms);
    (void)cache_known_frame(slow_cache, SLOW_CAN_IDS, header, data, now_ms);

    if (event) {
        queue_event(header, data);
    }
}

void service_can_rx(uint32_t now_ms)
{
    if (!can_ready) {
        return;
    }
    if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0) != RESET) {
        __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FOV0);
        ++can_rx_overruns;
    }
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef header{};
        uint8_t data[8]{};
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            return;
        }
        ++can_rx_count;
        if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA) {
            process_can_frame(header, data, now_ms);
        }
    }
}

void service_can_bus(uint32_t now_ms)
{
    if (!can_ready || !can_bus_off()) {
        return;
    }
    if ((now_ms - last_bus_recovery_ms) < CAN_BUS_RECOVERY_PERIOD_MS) {
        return;
    }
    (void)HAL_CAN_Stop(&hcan);
    HAL_CAN_ResetError(&hcan);
    (void)HAL_CAN_Start(&hcan);
    last_bus_recovery_ms = now_ms;
}

void send_heartbeat(uint32_t now_ms)
{
    uint8_t err = 0U;
    if (can_bus_off()) {
        err |= CAN_HEARTBEAT_ERR_BUS_OFF;
    }
    if (can_error() != 0U) {
        err |= CAN_HEARTBEAT_ERR_CAN_ERROR;
    }
    if (can_tx_drops != 0U) {
        err |= CAN_HEARTBEAT_ERR_TX_DROP;
    }
    if (croi_last_seen_ms == 0U || (now_ms - croi_last_seen_ms) >= CAN_CROI_TIMEOUT_MS) {
        err |= CAN_HEARTBEAT_ERR_NODE_TIMEOUT;
    }

    HEARTBEAT_Payload payload{
        static_cast<uint8_t>(NODE_TEACHTAIRE),
        0U,
        err,
        static_cast<uint8_t>((now_ms / 1000U) & 0xFFU),
    };
    CANFrame frame = make_can_frame(CAN_ID_HEARTBEAT_NODE(NODE_TEACHTAIRE), payload);
    if (send_can_frame(frame)) {
        ++heartbeat_tx_count;
    }
}

void send_tx_status(SX1272& lora)
{
    TX_STATUS_Payload payload{
        static_cast<int8_t>(lora.packet_rssi_dbm()),
        static_cast<int8_t>(lora.packet_snr_db()),
        tx_count,
        static_cast<uint8_t>((report.lora_init_ok != 0U ? 0x01U : 0U) |
                             (report.gnss_fix != 0U ? 0x02U : 0U)),
    };
    CANFrame frame = make_can_frame(CAN_ID_TX_STATUS, payload);
    if (send_can_frame(frame)) {
        ++tx_status_can_tx_count;
    }
}

void update_can_report()
{
    report.can_init_ok = can_ready ? 1U : 0U;
    report.can_bus_off = can_ready && can_bus_off() ? 1U : 0U;
    report.can_error = can_ready ? can_error() : 0U;
    report.can_tx_count = can_tx_count;
    report.can_rx_count = can_rx_count;
    report.can_tx_drops = can_tx_drops;
    report.can_tx_queue_depth = tx_count;
    report.heartbeat_tx_count = heartbeat_tx_count;
    report.gps_can_tx_count = gps_can_tx_count;
    report.tx_status_can_tx_count = tx_status_can_tx_count;
    report.croi_last_seen_ms = croi_last_seen_ms;
    report.can_esr = hcan.Instance != nullptr ? hcan.Instance->ESR : 0U;
    report.can_rx_overruns = can_rx_overruns;
}

void lora_reset_write(bool high, void*)
{
    HAL_GPIO_WritePin(GPIOA, LORA_RESET_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void lora_delay_ms(uint32_t ms, void*)
{
    HAL_Delay(ms);
}

void lora_switch_write(sx1272_switch_mode_t mode, void*)
{
    HAL_GPIO_WritePin(
        GPIOA,
        LORA_RX_SWITCH_PIN,
        mode == sx1272_switch_mode_t::RX ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOA,
        LORA_TX_SWITCH_PIN,
        mode == sx1272_switch_mode_t::TX ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

void gnss_reset()
{
    HAL_GPIO_WritePin(GPIOB, GNSS_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(GPIOB, GNSS_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(100U);
}

int32_t scale_double(double value, double scale)
{
    return static_cast<int32_t>(value * scale);
}

int16_t clamp_i16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }

    if (value < -32768) {
        return -32768;
    }

    return static_cast<int16_t>(value);
}

void write_u32_le(uint8_t* out, uint32_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    out[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    out[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

void write_u16_le(uint8_t* out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void write_i32_le(uint8_t* out, int32_t value)
{
    write_u32_le(out, static_cast<uint32_t>(value));
}

void write_i16_le(uint8_t* out, int16_t value)
{
    write_u16_le(out, static_cast<uint16_t>(value));
}

std::size_t build_ogma_gps_packet(
    uint8_t* out, std::size_t max_len, bool gps_valid, const gps_data& gps)
{
    if ((out == nullptr) || (max_len < OGMA_GPS_PACKET_LEN)) {
        return 0U;
    }

    // This must match teammate's Python:
    //
    //     lat, lon, alt, vel, sats, gps_valid = struct.unpack("<iihhBB", buf)
    //
    const int32_t latitude_e7 = scale_double(gps.latitude, 10000000.0);
    const int32_t longitude_e7 = scale_double(gps.longitude, 10000000.0);
    const int16_t altitude_dm = clamp_i16(static_cast<int32_t>(gps.altitude * 10.0f));
    const int16_t velocity_dm_s = clamp_i16(static_cast<int32_t>(gps.velocity * 10.0f));
    const uint8_t satellites = gps.satellites;
    const uint8_t gps_valid_byte = gps_valid ? 1U : 0U;

    write_i32_le(&out[0], latitude_e7);
    write_i32_le(&out[4], longitude_e7);
    write_i16_le(&out[8], altitude_dm);
    write_i16_le(&out[10], velocity_dm_s);
    out[12] = satellites;
    out[13] = gps_valid_byte;

    return OGMA_GPS_PACKET_LEN;
}

void update_gpio_report()
{
    report.gpioa_idr = GPIOA->IDR;
    report.gpioa_odr = GPIOA->ODR;
    report.gpiob_idr = GPIOB->IDR;
    report.gpiob_odr = GPIOB->ODR;
}

void update_gnss_report(
    const MAXM10S& gnss, const gps_data& gps, bool gps_valid,
    uint32_t last_fix_ms, uint32_t now_ms)
{
    report.gnss_seen = gnss.messages_seen();
    report.gnss_parsed = gnss.sentences_parsed();
    report.gnss_checksum_bad = gnss.checksum_failures();
    report.gnss_bytes = gnss.bytes_seen();
    report.gnss_starts = gnss.sentences_started();
    report.gnss_overflows = gnss.line_overflows();
    report.gnss_txt = gnss.text_messages_seen();
    report.gnss_nav_sat = gnss.navigation_satellite_messages_seen();

    report.gnss_fix = gps_valid ? 1U : 0U;
    report.gnss_sats = gps.satellites;
    report.gnss_sats_in_view = gnss.satellites_in_view();
    report.gnss_ant_status = gnss.antenna_status();
    report.gnss_nav_sat_count = gnss.navigation_satellites_reported();
    report.gnss_nav_sat_signal = gnss.navigation_satellites_with_signal();
    report.gnss_nav_sat_max_cno = gnss.navigation_satellite_max_cno();

    report.gnss_latitude_e7 = scale_double(gps.latitude, 10000000.0);
    report.gnss_longitude_e7 = scale_double(gps.longitude, 10000000.0);
    report.gnss_altitude_mm = static_cast<int32_t>(gps.altitude * 1000.0f);
    report.gnss_velocity_mm_s = static_cast<int32_t>(gps.velocity * 1000.0f);
    report.gnss_last_fix_ms = last_fix_ms;
    report.gnss_fix_age_ms = last_fix_ms == 0U ? UINT32_MAX : now_ms - last_fix_ms;
}

void update_lora_report(SX1272& lora)
{
    report.lora_version = lora.version();
    report.lora_error = lora.last_error();
    report.lora_irq = lora.irq_flags();
    report.lora_rssi_dbm = lora.packet_rssi_dbm();
    report.lora_snr_db = lora.packet_snr_db();
}

void update_bus_report(SPI_STM& spi, UART_STM& uart)
{
    report.spi_status = spi.last_status();
    report.spi_error = spi.last_error();
    report.uart_status = uart.last_status();
    report.uart_error = uart.last_error();
    report.gnss_uart_overrun_recoveries = uart.rx_overrun_recoveries();
    report.usart1_isr = USART1->ISR;
    update_gpio_report();
}

} // namespace

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

[[noreturn]] void Error_Handler();

int main()
{
    (void)ogma_board_identity.magic;
    report.magic = REPORT_MAGIC;
    report.version = 6U;
    report.radio_config_magic = TEACHTAIRE_RADIO_CONFIG_MAGIC;
    report.radio_config_schema_version = TEACHTAIRE_RADIO_CONFIG_SCHEMA_VERSION;
    report.radio_config_crc32 = TEACHTAIRE_RADIO_CONFIG_CRC32;
    report.reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    HAL_Init();
    SystemClock_Config();
    report.watchdog_init_ok = init_watchdog() ? 1U : 0U;
    if (report.watchdog_init_ok == 0U) {
        Error_Handler();
    }
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    can_ready = MX_CAN_Init() && HAL_CAN_Start(&hcan) == HAL_OK;
    gnss_reset();

    SPI_STM spi(&hspi1, GPIOA, LORA_NSS_PIN);
    UART_STM uart(&huart1);

    sx1272_pins_t lora_pins{};
    lora_pins.reset_write = lora_reset_write;
    lora_pins.delay_ms = lora_delay_ms;
    lora_pins.switch_write = lora_switch_write;
    lora_pins.reset_active_high = true;

    SX1272 lora(spi, 0, lora_pins);
    MAXM10S gnss(uart);

    gnss.init();

    sx1272_config_t lora_config{};
    report.lora_init_ok = lora.init(lora_config) ? 1U : 0U;
    report.lora_rx_ok =
        (report.lora_init_ok != 0U && LORA_RECEIVE_ONLY && lora.start_receive()) ? 1U : 0U;

    gps_data gps{};

    uint32_t last_tx_ms = 0U;
    uint32_t last_report_ms = 0U;
    uint32_t last_nav_sat_poll_ms = 0U;
    uint32_t last_gps_can_ms = 0U;
    uint32_t last_valid_fix_ms = 0U;
    uint32_t last_lora_init_attempt_ms = 0U;
    uint32_t lora_tx_started_ms = 0U;
    uint32_t tx_counter = 0U;
    uint32_t last_radio_core_ms = 0U;
    uint32_t last_radio_gps_ms = 0U;
    uint32_t last_radio_slow_ms = 0U;
    uint32_t last_radio_deep_ms = 0U;
    last_heartbeat_ms = HAL_GetTick();
    last_tx_status_ms = last_heartbeat_ms;
    last_bus_recovery_ms = last_heartbeat_ms;

    bool lora_tx_busy = false;

    while (true) {
        report.loops++;

        // Keep draining GPS UART. Do not add HAL_Delay() here.
        bool gnss_updated = gnss.update(&gps);
        uint32_t now = HAL_GetTick();
        if (gnss_updated && gnss.fix_valid()) {
            last_valid_fix_ms = now;
        }
        const bool gps_valid = gnss.fix_valid() && last_valid_fix_ms != 0U &&
                               (now - last_valid_fix_ms) < GNSS_FIX_TIMEOUT_MS;
        service_can_bus(now);
        flush_can_queue();
        service_can_rx(now);

        if ((now - last_nav_sat_poll_ms) >= 1000U) {
            last_nav_sat_poll_ms = now;
            (void)gnss.poll_navigation_satellites();
        }

        if (lora_tx_busy) {
            if (lora.tx_done()) {
                lora_tx_busy = false;
                report.lora_tx_done_count++;
            } else if ((now - lora_tx_started_ms) >= LORA_TX_TIMEOUT_MS) {
                lora_tx_busy = false;
                report.lora_init_ok = 0U;
                ++report.lora_tx_timeout_count;
            }
        }

        if (report.lora_init_ok == 0U &&
            (now - last_lora_init_attempt_ms) >= LORA_REINIT_PERIOD_MS) {
            last_lora_init_attempt_ms = now;
            ++report.lora_reinit_count;
            report.lora_init_ok = lora.init(lora_config) ? 1U : 0U;
            report.lora_rx_ok =
                (report.lora_init_ok != 0U && LORA_RECEIVE_ONLY && lora.start_receive()) ? 1U : 0U;
        }

        if (LORA_RECEIVE_ONLY) {
            uint8_t packet[255]{};
            std::size_t packet_len = 0U;
            if ((report.lora_rx_ok != 0U) && lora.receive(packet, sizeof(packet), &packet_len)) {
                report.lora_rx_count++;
                report.lora_last_counter = static_cast<uint32_t>(packet_len);
            } else if (lora.last_error() == 4U) {
                ++report.lora_rx_bad_count;
            }
        } else {
            if ((report.lora_init_ok != 0U) && !lora_tx_busy) {
                enum class TelemetryClass : uint8_t {
                    None,
                    Core,
                    Gps,
                    Slow,
                    Event,
                    Deep,
                    Test,
                };

                uint8_t packet[64]{};
                std::size_t packet_len = 0U;
                TelemetryClass packet_class = TelemetryClass::None;
                const uint16_t radio_sequence = static_cast<uint16_t>(tx_counter & 0xFFFFU);

                if (LORA_TRANSMIT_TEST) {
                    if ((now - last_tx_ms) >= 1000U) {
                        packet_len = ogma_radio::build_test_packet(
                            packet, sizeof(packet), radio_sequence, now, tx_counter);
                        packet_class = TelemetryClass::Test;
                    }
                } else if (event_count > 0U) {
                    packet_len = ogma_radio::build_can_bundle(
                        packet, sizeof(packet), radio_sequence, now,
                        &event_queue[event_head], 1U, 0x01U);
                    packet_class = TelemetryClass::Event;
                }
                if (!LORA_TRANSMIT_TEST && packet_len == 0U &&
                    (now - last_radio_core_ms) >= RADIO_CORE_PERIOD_MS) {
                    ogma_radio::RawCanRecord records[ogma_radio::kMaxCanRecords]{};
                    const std::size_t count = collect_records(
                        core_cache, CORE_CAN_COUNT, now, records,
                        ogma_radio::kMaxCanRecords, core_page_start);
                    if (count > 0U) {
                        packet_len = ogma_radio::build_can_bundle(
                            packet, sizeof(packet), radio_sequence, now,
                            records, count, 0x02U);
                        packet_class = TelemetryClass::Core;
                    }
                }
                if (!LORA_TRANSMIT_TEST && packet_len == 0U &&
                    (now - last_radio_gps_ms) >= RADIO_GPS_PERIOD_MS) {
                    uint8_t gps_payload[OGMA_GPS_PACKET_LEN]{};
                    const std::size_t gps_length = build_ogma_gps_packet(
                        gps_payload, sizeof(gps_payload), gps_valid, gps);
                    packet_len = ogma_radio::build_gps_packet(
                        packet, sizeof(packet), radio_sequence, now,
                        gps_payload, gps_length, gps_valid ? 0x01U : 0U);
                    packet_class = TelemetryClass::Gps;
                }
                if (!LORA_TRANSMIT_TEST && packet_len == 0U &&
                    (now - last_radio_slow_ms) >= RADIO_SLOW_PERIOD_MS) {
                    ogma_radio::RawCanRecord records[ogma_radio::kMaxCanRecords]{};
                    const std::size_t count = collect_records(
                        slow_cache, 4U, now, records, ogma_radio::kMaxCanRecords);
                    if (count > 0U) {
                        packet_len = ogma_radio::build_can_bundle(
                            packet, sizeof(packet), radio_sequence, now,
                            records, count, 0x04U);
                        packet_class = TelemetryClass::Slow;
                    }
                }
                if (!LORA_TRANSMIT_TEST && packet_len == 0U &&
                    (now - last_radio_deep_ms) >= RADIO_DEEP_PERIOD_MS) {
                    ogma_radio::RawCanRecord records[ogma_radio::kMaxCanRecords]{};
                    const std::size_t count = collect_heartbeat_records(
                        now, records, ogma_radio::kMaxCanRecords);
                    if (count > 0U) {
                        packet_len = ogma_radio::build_can_bundle(
                            packet, sizeof(packet), radio_sequence, now,
                            records, count, 0x08U);
                        packet_class = TelemetryClass::Deep;
                    }
                }

                if ((packet_len > 0U) && lora.send(packet, packet_len)) {
                    lora_tx_busy = true;
                    lora_tx_started_ms = now;
                    last_tx_ms = now;
                    report.lora_tx_count++;
                    report.lora_last_counter = tx_counter;
                    tx_counter++;
                    switch (packet_class) {
                    case TelemetryClass::Core:
                        last_radio_core_ms = now;
                        core_page_start =
                            (core_page_start + ogma_radio::kMaxCanRecords) % CORE_CAN_COUNT;
                        ++report.telemetry_core_tx_count;
                        break;
                    case TelemetryClass::Gps:
                        last_radio_gps_ms = now;
                        ++report.telemetry_gps_tx_count;
                        break;
                    case TelemetryClass::Slow:
                        last_radio_slow_ms = now;
                        ++report.telemetry_slow_tx_count;
                        break;
                    case TelemetryClass::Event:
                        event_head = static_cast<uint8_t>((event_head + 1U) % RADIO_EVENT_QUEUE_LEN);
                        --event_count;
                        ++report.telemetry_event_tx_count;
                        break;
                    case TelemetryClass::Deep:
                        last_radio_deep_ms = now;
                        ++report.telemetry_deep_tx_count;
                        break;
                    default:
                        break;
                    }
                }
            }
        }

        if ((now - last_gps_can_ms) >= GPS_CAN_PERIOD_MS) {
            last_gps_can_ms = now;
            CANFrame gps_frame = pack_gps_frame(gps, gps_valid);
            if (send_can_frame(gps_frame)) {
                ++gps_can_tx_count;
            }
        }

        if ((now - last_heartbeat_ms) >= CAN_HEARTBEAT_PERIOD_MS) {
            send_heartbeat(now);
            last_heartbeat_ms = now;
        }

        if ((now - last_tx_status_ms) >= CAN_TX_STATUS_PERIOD_MS) {
            send_tx_status(lora);
            last_tx_status_ms = now;
        }

        if (gnss_updated || ((now - last_report_ms) >= 250U)) {
            last_report_ms = now;

            update_gnss_report(gnss, gps, gps_valid, last_valid_fix_ms, now);
            update_lora_report(lora);
            update_bus_report(spi, uart);
            update_can_report();
        }
        if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK) {
            ++report.watchdog_refresh_count;
        }
    }
}

[[noreturn]] void Error_Handler()
{
    __disable_irq();
    for (;;) {
    }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* handle)
{
    if (handle->Instance != CAN) {
        return;
    }

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* handle)
{
    if (handle->Instance != CAN) {
        return;
    }

    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}
