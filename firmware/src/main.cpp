#include <GNSS/MAXM10S.h>
#include <Radio/SX1272.h>
#include <SPI/SPI_STM.h>
#include <UART/UART_STM.h>

#include <stm32f0xx_hal.h>

#include <cstdint>

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

    uint32_t gnss_seen;
    uint32_t gnss_parsed;
    uint32_t gnss_checksum_bad;
    uint8_t gnss_fix;
    uint8_t gnss_sats;
    int32_t gnss_latitude_e7;
    int32_t gnss_longitude_e7;
    int32_t gnss_altitude_mm;
    int32_t gnss_velocity_mm_s;

    uint32_t spi_status;
    uint32_t spi_error;
    uint32_t uart_status;
    uint32_t uart_error;
    uint32_t gpioa_idr;
    uint32_t gpioa_odr;
    uint32_t gpiob_idr;
    uint32_t gpiob_odr;
    uint32_t fault;
};

extern "C" {
volatile teachtaire_report_t report{};
}

namespace {

constexpr uint32_t REPORT_MAGIC = 0x54434854U;

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

SPI_HandleTypeDef hspi1{};
UART_HandleTypeDef huart1{};

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
    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        report.fault = 1U;
        return;
    }

    RCC_ClkInitTypeDef clk{};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
        report.fault = 2U;
        return;
    }

    SystemCoreClockUpdate();
    report.clock_ok = 1U;
    report.clock_hz = SystemCoreClock;
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
    report.uart_ok = (HAL_UART_Init(&huart1) == HAL_OK) ? 1U : 0U;
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
    HAL_GPIO_WritePin(GPIOA, LORA_RX_SWITCH_PIN, mode == sx1272_switch_mode_t::RX ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LORA_TX_SWITCH_PIN, mode == sx1272_switch_mode_t::TX ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

void update_gpio_report()
{
    report.gpioa_idr = GPIOA->IDR;
    report.gpioa_odr = GPIOA->ODR;
    report.gpiob_idr = GPIOB->IDR;
    report.gpiob_odr = GPIOB->ODR;
}

void update_gnss_report(const MAXM10S& gnss, const gps_data& gps)
{
    report.gnss_seen = gnss.messages_seen();
    report.gnss_parsed = gnss.sentences_parsed();
    report.gnss_checksum_bad = gnss.checksum_failures();
    report.gnss_fix = gnss.fix_valid() ? 1U : 0U;
    report.gnss_sats = gps.satellites;
    report.gnss_latitude_e7 = scale_double(gps.latitude, 10000000.0);
    report.gnss_longitude_e7 = scale_double(gps.longitude, 10000000.0);
    report.gnss_altitude_mm = static_cast<int32_t>(gps.altitude * 1000.0f);
    report.gnss_velocity_mm_s = static_cast<int32_t>(gps.velocity * 1000.0f);
}

void update_lora_report(SX1272& lora)
{
    report.lora_version = lora.version();
    report.lora_error = lora.last_error();
    report.lora_irq = lora.irq_flags();
    report.lora_rssi_dbm = lora.packet_rssi_dbm();
    report.lora_snr_db = lora.packet_snr_db();
}

} // namespace

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main()
{
    report.magic = REPORT_MAGIC;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
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
    report.lora_rx_ok = (report.lora_init_ok != 0U && lora.start_receive()) ? 1U : 0U;

    gps_data gps{};
    while (true) {
        report.loops++;

        if (gnss.update(&gps)) {
            update_gnss_report(gnss, gps);
        } else if ((report.loops & 0x1FU) == 0U) {
            update_gnss_report(gnss, gps);
        }

        if ((report.loops & 0x1FU) == 0U) {
            update_lora_report(lora);
            report.spi_status = spi.last_status();
            report.spi_error = spi.last_error();
            report.uart_status = uart.last_status();
            report.uart_error = uart.last_error();
            update_gpio_report();
        }

        HAL_Delay(10U);
    }
}
