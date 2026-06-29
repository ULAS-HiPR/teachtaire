#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "platform/error_handler.h"
#include "platform/hal_time.h"

//generics
#include <data.h>
#include <sensor.h>

#include <SPI/SPI_Handler.h>
#include <UART/UART_Handler.h>
#include <CAN/CAN_Handler.h>

#include <Radio/Radio.h>
#include <GNSS/GNSS.h>

//specifics
#include <SPI/SPI_STM.h>
#include <UART/UART_STM.h>
#if F4
#include <CAN/CAN_Mock.h>
#elif F0
#include <CAN/CAN_STM.h>
#endif

#include <Radio/SX1272.h>
#include <GNSS/MAXM10S.h>

//tasks
#include "tasks/CAN_task.h"
#include "tasks/telem_task.h"

#define SYNC_WORD 0x50 // O for Ogma, P for Payload


void SystemClock_Config(void);


const osMessageQueueAttr_t canRQueue_attributes = {
  .name = "canReciverQueue"
};

const osMessageQueueAttr_t canSQueue_attributes = {
  .name = "canSenderQueue"
};

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  osKernelInitialize();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  
  SPI_Handler* spi_handler_radio = new SPI_STM(&hspi1, RADIO_CS_PORT, RADIO_CS_PIN);
  UART_Handler* uart_handler_gnss = new UART_STM(&huart1);

  //only for testing
  sx1272_pins_t radio_pins = {
      .reset_write = [](bool high, void* context) {
          GPIO_TypeDef* port = static_cast<GPIO_TypeDef*>(context);
          HAL_GPIO_WritePin(port, RADIO_RESET_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
      },
      .reset_context = RADIO_RESET_PORT,
      .delay_ms = [](std::uint32_t ms, void* context) {
          HAL_Delay(ms);
      },
      .delay_context = nullptr,
      .switch_write = [](sx1272_switch_mode_t mode, void* context) {
          switch (mode) {
              case sx1272_switch_mode_t::OFF:
                  HAL_GPIO_WritePin(RADIO_RX_SW_PORT, RADIO_RX_SW_PIN, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(RADIO_TX_SW_PORT, RADIO_TX_SW_PIN, GPIO_PIN_RESET);
                  break;
              case sx1272_switch_mode_t::RX:
                  HAL_GPIO_WritePin(RADIO_RX_SW_PORT, RADIO_RX_SW_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(RADIO_TX_SW_PORT, RADIO_TX_SW_PIN, GPIO_PIN_RESET);
                  break;
              case sx1272_switch_mode_t::TX:
                  HAL_GPIO_WritePin(RADIO_RX_SW_PORT, RADIO_RX_SW_PIN, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(RADIO_TX_SW_PORT, RADIO_TX_SW_PIN, GPIO_PIN_SET);
                  break;
          }
      },
      .switch_context = nullptr
  };

  Radio* radio = new SX1272(*spi_handler_radio, RADIO_CS_PIN, radio_pins, SYNC_WORD);
  GNSS* gnss = new MAXM10S(*uart_handler_gnss);
  bool init_status_radio = radio->init();
  bool init_status_gnss = gnss->init();


  #if F4
    static CAN_MOCK canbus;
  #elif F0
    MX_CAN_Init();
    static CAN_STM canbus(&hcan);
    if (!canbus.init()) {
       Error_Handler();
    }
  #endif

  osMessageQueueId_t canReciverQueueHandle = osMessageQueueNew(4, sizeof(flight_data), &canRQueue_attributes);
  osMessageQueueId_t canSenderQueueHandle = osMessageQueueNew(4, sizeof(gps_data), &canSQueue_attributes);

  static task::Telem_Task telem_task(*radio, *gnss, canReciverQueueHandle, canSenderQueueHandle);
  static task::CAN_task can_task(canbus, canSenderQueueHandle, canReciverQueueHandle, NODE_TEACHTAIRE);

  telem_task.run();
  can_task.run();

  
  osKernelStart();
  // never get here 
  while (1)
  {
    HAL_Delay(1000);
  }
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;


  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
      Error_Handler();
  }
}

#ifdef F0
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
        HAL_IncTick();
    }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
#endif // F0

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
