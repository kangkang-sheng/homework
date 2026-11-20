/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t header;
  uint8_t run;
  uint8_t dir;
  uint8_t step_l;
  uint8_t step_h;
  uint8_t check;
} UartLedPacket_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_LED_HEADER            0xAAU
#define UART_LED_PACKET_SIZE       6U
#define UART_LED_COUNT             3U
#define UART_LED_STEP_MIN_MS       100U
#define UART_LED_STEP_MAX_MS       2000U
#define UART_LED_COMMAND_QUEUE_SIZE 4U
#define UART_LED_TX_QUEUE_SIZE     4U
#define UART_LED_TX_MESSAGE_SIZE   64U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t led_index = 0U;
static uint8_t running = 0U;
static uint8_t direction = 0U;
static uint16_t step_ms = 300U;
static uint32_t last_step_tick = 0U;

/* UART2 receive state. UART2 uses PA2 (TX) and PA3 (RX). */
static uint8_t uart_rx_byte = 0U;
static uint8_t uart_rx_count = 0U;
static UartLedPacket_t uart_rx_frame;

/* Complete packets are queued here by the UART interrupt. */
static UartLedPacket_t command_queue[UART_LED_COMMAND_QUEUE_SIZE];
static volatile uint8_t command_head = 0U;
static volatile uint8_t command_tail = 0U;

/* English feedback messages sent with HAL_UART_Transmit_IT(). */
static char tx_message[UART_LED_TX_QUEUE_SIZE][UART_LED_TX_MESSAGE_SIZE];
static uint16_t tx_length[UART_LED_TX_QUEUE_SIZE];
static volatile uint8_t tx_head = 0U;
static volatile uint8_t tx_tail = 0U;
static volatile uint8_t tx_busy = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Led_Show(uint8_t index);
static void Oled_ShowAll(void);
static void Key1_Task(void);
static void Key2_Task(void);
static uint8_t UartLed_NextIndex(uint8_t index, uint8_t size);
static uint8_t UartLed_Checksum(const UartLedPacket_t *packet);
static uint8_t UartLed_FrameIsValid(const UartLedPacket_t *packet);
static void UartLed_QueueState(void);
static void UartLed_StartTransmit(void);
static void UartLed_StartReceive(void);
static void UartLed_ApplyPacket(const UartLedPacket_t *packet);
static void UartLed_Process(void);
static void UartLed_HandleRxByte(uint8_t received);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Led_Show(uint8_t index)
{
  HAL_GPIO_WritePin(GPIOA, LED_1_Pin,
                    (index == 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LED_2_Pin,
                    (index == 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LED_3_Pin,
                    (index == 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Oled_ShowAll(void)
{
  OLED_ShowString(1U, 1U, running ? "STATE: RUN  " : "STATE: PAUSE");
  OLED_ShowString(2U, 1U, direction ? "DIR: REV " : "DIR: FWD ");
  OLED_ShowString(3U, 1U, "STEP=");
  OLED_ShowNum(3U, 6U, step_ms, 4U);
  OLED_ShowString(3U, 10U, "ms  ");
  OLED_ShowString(4U, 1U, "LED: ");
  OLED_ShowNum(4U, 6U, (uint32_t)(led_index + 1U), 1U);
}

static void Key1_Task(void)
{
  if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_SET)
  {
    HAL_Delay(20U);
    if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_SET)
    {
      running = running ? 0U : 1U;
      last_step_tick = HAL_GetTick();
      Oled_ShowAll();
      UartLed_QueueState();
      while (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_SET)
      {
      }
    }
  }
}

static void Key2_Task(void)
{
  if (HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == GPIO_PIN_SET)
  {
    HAL_Delay(20U);
    if (HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == GPIO_PIN_SET)
    {
      direction = direction ? 0U : 1U;
      last_step_tick = HAL_GetTick();
      Oled_ShowAll();
      UartLed_QueueState();
      while (HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == GPIO_PIN_SET)
      {
      }
    }
  }
}

static uint8_t UartLed_NextIndex(uint8_t index, uint8_t size)
{
  index++;
  if (index >= size)
  {
    index = 0U;
  }
  return index;
}

static uint8_t UartLed_Checksum(const UartLedPacket_t *packet)
{
  uint16_t sum = UART_LED_HEADER;

  sum += (uint16_t)packet->run;
  sum += (uint16_t)packet->dir;
  sum += (uint16_t)packet->step_l;
  sum += (uint16_t)packet->step_h;
  return (uint8_t)(sum & 0xFFU);
}

static uint8_t UartLed_FrameIsValid(const UartLedPacket_t *packet)
{
  uint16_t requested_step;

  if (packet->header != UART_LED_HEADER)
  {
    return 0U;
  }

  if ((packet->run > 1U) || (packet->dir > 1U))
  {
    return 0U;
  }

  requested_step = (uint16_t)packet->step_l |
                   ((uint16_t)packet->step_h << 8U);
  if ((requested_step < UART_LED_STEP_MIN_MS) ||
      (requested_step > UART_LED_STEP_MAX_MS))
  {
    return 0U;
  }

  return (packet->check == UartLed_Checksum(packet)) ? 1U : 0U;
}

static void UartLed_QueueState(void)
{
  uint8_t next_tail;
  int length;

  next_tail = UartLed_NextIndex(tx_tail, UART_LED_TX_QUEUE_SIZE);
  if (next_tail == tx_head)
  {
    return;
  }

  length = snprintf(tx_message[tx_tail],
                    UART_LED_TX_MESSAGE_SIZE,
                    "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                    running ? "RUN" : "PAUSE",
                    direction ? "REV" : "FWD",
                    (unsigned int)step_ms,
                    (unsigned int)(led_index + 1U));

  if ((length <= 0) ||
      ((size_t)length >= (size_t)UART_LED_TX_MESSAGE_SIZE))
  {
    return;
  }

  tx_length[tx_tail] = (uint16_t)length;
  tx_tail = next_tail;
}

static void UartLed_StartTransmit(void)
{
  if (tx_busy || (tx_head == tx_tail))
  {
    return;
  }

  tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart2,
                           (uint8_t *)tx_message[tx_head],
                           tx_length[tx_head]) != HAL_OK)
  {
    tx_busy = 0U;
  }
}

static void UartLed_StartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
}

static void UartLed_ApplyPacket(const UartLedPacket_t *packet)
{
  if (!UartLed_FrameIsValid(packet))
  {
    return;
  }

  running = packet->run;
  direction = packet->dir;
  step_ms = (uint16_t)packet->step_l |
            ((uint16_t)packet->step_h << 8U);
  last_step_tick = HAL_GetTick();

  Led_Show(led_index);
  Oled_ShowAll();
  UartLed_QueueState();
}

static void UartLed_Process(void)
{
  uint8_t next_head;
  uint32_t now;

  while (command_head != command_tail)
  {
    UartLedPacket_t packet = command_queue[command_head];

    next_head = UartLed_NextIndex(command_head,
                                  UART_LED_COMMAND_QUEUE_SIZE);
    command_head = next_head;
    UartLed_ApplyPacket(&packet);
  }

  now = HAL_GetTick();
  if (running && ((uint32_t)(now - last_step_tick) >= step_ms))
  {
    last_step_tick += step_ms;

    if (direction == 0U)
    {
      led_index = UartLed_NextIndex(led_index, UART_LED_COUNT);
    }
    else
    {
      led_index = (led_index == 0U)
                      ? (UART_LED_COUNT - 1U)
                      : (uint8_t)(led_index - 1U);
    }

    Led_Show(led_index);
    OLED_ShowNum(4U, 6U, (uint32_t)(led_index + 1U), 1U);
  }

  UartLed_StartTransmit();
}

static void UartLed_HandleRxByte(uint8_t received)
{
  uint8_t *frame_bytes = (uint8_t *)&uart_rx_frame;

  if (uart_rx_count == 0U)
  {
    if (received == UART_LED_HEADER)
    {
      frame_bytes[0] = received;
      uart_rx_count = 1U;
    }
    return;
  }

  frame_bytes[uart_rx_count] = received;
  uart_rx_count++;

  if (uart_rx_count == UART_LED_PACKET_SIZE)
  {
    uint8_t next_tail = UartLed_NextIndex(command_tail,
                                          UART_LED_COMMAND_QUEUE_SIZE);

    /*
     * Header and checksum are checked in the interrupt. Field values are
     * checked again before the command changes any application state.
     */
    if ((next_tail != command_head) &&
        (uart_rx_frame.header == UART_LED_HEADER) &&
        (uart_rx_frame.check == UartLed_Checksum(&uart_rx_frame)))
    {
      command_queue[command_tail] = uart_rx_frame;
      command_tail = next_tail;
    }

    uart_rx_count = 0U;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* OLED.c uses software I2C on PB6 (SCL) and PB7 (SDA). */
  OLED_Init();

  Led_Show(led_index);
  Oled_ShowAll();
  last_step_tick = HAL_GetTick();
  UartLed_StartReceive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Key1_Task();
    Key2_Task();
    UartLed_Process();

    HAL_Delay(2U);
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    UartLed_HandleRxByte(uart_rx_byte);
    UartLed_StartReceive();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (tx_head != tx_tail)
    {
      tx_head = UartLed_NextIndex(tx_head, UART_LED_TX_QUEUE_SIZE);
    }
    tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_rx_count = 0U;
    tx_busy = 0U;
    (void)HAL_UART_Abort(&huart2);
    UartLed_StartReceive();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
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
