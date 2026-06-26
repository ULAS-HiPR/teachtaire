#ifdef F0
#ifndef STM_F072xB_H
#define STM_F072xB_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_gpio.h"
#include "error_handler.h"



extern SPI_HandleTypeDef hspi1;
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

#define CAN_RX_PIN GPIO_PIN_11
#define CAN_TX_PIN GPIO_PIN_12
#define CAN_GPIO_PORT GPIOA
#define CAN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPI_SCK_PIN GPIO_PIN_5 
#define SPI_MISO_PIN GPIO_PIN_6 
#define SPI_MOSI_PIN GPIO_PIN_7 
#define SPI_GPIO_PORT GPIOA 
#define SPI_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() 

#define RADIO_CS_PIN GPIO_PIN_2
#define RADIO_CS_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_1
#define RADIO_RESET_PORT GPIOA
#define RADIO_DIO0_PIN GPIO_PIN_10
#define RADIO_DIO0_PORT GPIOB
#define RADIO_RX_SW_PIN GPIO_PIN_8
#define RADIO_RX_SW_PORT GPIOA
#define RADIO_TX_SW_PIN GPIO_PIN_15
#define RADIO_TX_SW_PORT GPIOA

#define GNSS_UART_TX_PIN GPIO_PIN_9
#define GNSS_UART_RX_PIN GPIO_PIN_10
#define GNSS_UART_PORT GPIOA
#define GNSS_UART_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define GNSS_RESET_PIN GPIO_PIN_8
#define GNSS_RESET_PORT GPIOB



// Initialization functions
void MX_SPI1_Init();
void MX_CAN_Init();
void MX_USART1_UART_Init();
void MX_GPIO_Init();

#endif // STM_F4_H
#endif // F4