/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Adam's BDShot Code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DSHOT_BIT_PERIOD_US 1.67f
#define DSHOT_BIT_PERIOD_TICKS 140  // 1.67 µs at 84 MHz (TIM5 clock)
#define DSHOT_T1L_TICKS 105         // Logic 1: LOW for 1.130 µs (inverted)
#define DSHOT_T0L_TICKS 50         // Logic 0: LOW for 0.600 µs (inverted)
#define DSHOT_BUFFER_SIZE 20 // 1 LOW + 16 bits + 1 LOW
#define DSHOT_TIME_GAP_US 500 //Gap between frames (adjustable)
#define DSHOT_FRAME_TIME_US (DSHOT_BIT_PERIOD_US * (16 + 2)) //26.72 us + 3.34 us = 30.06 us
#define DSHOT_TOTAL_PERIOD_US (DSHOT_FRAME_TIME_US + DSHOT_TIME_GAP_US)
#define DEBUG_MSG_MAX_LEN 64 //Max length for debug messages
#define ARMING_DURATION_US 3000000 //3s for robust ESC arming

#define THROTTLE_MINIMUM 70

// 21 bits: 1 start (always 0), 20 data bits
#define TELEMETRY_BITS 21

// This timing must match the ESC's telemetry bit width.
// For BDshot, Bluejay's default is ~2.14 us per bit (for 8kHz DShot).
#define TELEMETRY_BIT_US 1.13f  // Adjust if needed
#define TELEMETRY_TIMEOUT_US 50 // Max wait before abort (for loss of signal)

#define POLE_PAIRS 7

#define SERIAL_QUEUE_LENGTH 10
#define SERIAL_QUEUE_ITEM_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch2;
DMA_HandleTypeDef hdma_tim5_ch3_up;
DMA_HandleTypeDef hdma_tim5_ch4_trig;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t telemetry_buffer[2]; // Make sure it's global!

typedef struct {
    uint8_t data[SERIAL_QUEUE_ITEM_SIZE];
    size_t length;
} SerialMessage_t;

static uint32_t dshot_buffer_ch1[DSHOT_BUFFER_SIZE] __attribute__((aligned(4))); // DMA buffer for TIM5_CH1
static uint32_t dshot_buffer_ch2[DSHOT_BUFFER_SIZE] __attribute__((aligned(4))); // DMA buffer for TIM5_CH2
static uint32_t dshot_buffer_ch3[DSHOT_BUFFER_SIZE] __attribute__((aligned(4))); // DMA buffer for TIM5_CH3
static uint32_t dshot_buffer_ch4[DSHOT_BUFFER_SIZE] __attribute__((aligned(4))); // DMA buffer for TIM5_CH4
static volatile bool dshot_running_ch1 = false;       // Flag for CH1 DMA completion
static volatile bool dshot_running_ch2 = false;       // Flag for CH2 DMA completion
static volatile bool dshot_running_ch3 = false;       // Flag for CH3 DMA completion
static volatile bool dshot_running_ch4 = false;       // Flag for CH4 DMA completion


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void prepare_bdshot_buffer(uint16_t frame, uint32_t *dshot_buffer);
uint16_t make_bdshot_frame(uint16_t value, bool telemetry);
void queue_bdshot_pulse(uint16_t throttle, bool telemetry, uint32_t *dshot_buffer);
void send_bdshot(uint32_t channel);
void delay_task_us(uint32_t us);
void DWT_Init(void);
void delay_us_precise(double us);
uint32_t decode_gcr_mapping(uint32_t value);
int decode_gcr_20_to_16(uint32_t input_20bit, uint16_t *out_value);
int parse_edt_frame(uint16_t frame, char *type_out, float *value_out);
void set_pin_input(GPIO_TypeDef *port, uint16_t pin);
void set_pin_pwm(GPIO_TypeDef *port, uint16_t pin, uint8_t alternate);
void process_bdshot_telemetry(GPIO_TypeDef *port, uint16_t pin, uint8_t *packet_out);
void delay_ns(uint32_t ns);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  DWT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("\nDShotTask Begin.\r\n");
  printf("SystemCoreClock=%lu\r\n", SystemCoreClock);
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t tim5_clk = pclk1;
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
      tim5_clk *= 2;
  printf("TIM5 actual clk: %lu\r\n", tim5_clk);

  queue_bdshot_pulse(0, true, dshot_buffer_ch1);
  queue_bdshot_pulse(0, true, dshot_buffer_ch2);
  queue_bdshot_pulse(0, true, dshot_buffer_ch3);
  queue_bdshot_pulse(0, true, dshot_buffer_ch4);
  for (int i = 0; i < 3000; i++){
  	send_bdshot(TIM_CHANNEL_1);
  	send_bdshot(TIM_CHANNEL_2);
  	send_bdshot(TIM_CHANNEL_3);
  	send_bdshot(TIM_CHANNEL_4);
  	delay_us_precise(1000);
  	}
  	delay_us_precise(50000);

  	queue_bdshot_pulse(100, true, dshot_buffer_ch1);
  	queue_bdshot_pulse(200, true, dshot_buffer_ch2);
  	queue_bdshot_pulse(300, true, dshot_buffer_ch3);
  	queue_bdshot_pulse(400, true, dshot_buffer_ch4);
  	uint8_t packet_PA0[3];
  	uint8_t packet_PA1[3];
  	uint8_t packet_PA2[3];
  	uint8_t packet_PA3[3];
      for (;;){
        while(dshot_running_ch1){delay_us_precise(5);}
        send_bdshot(TIM_CHANNEL_1);
        delay_us_precise(40);
        set_pin_input(GPIOA, GPIO_PIN_0);
        process_bdshot_telemetry(GPIOA, GPIO_PIN_0, packet_PA0);
        set_pin_pwm(GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5);
        delay_us_precise(60);

        while(dshot_running_ch2){delay_us_precise(5);}
        send_bdshot(TIM_CHANNEL_2);
        delay_us_precise(40);
        set_pin_input(GPIOA, GPIO_PIN_1);
        //process_bdshot_telemetry(GPIOA, GPIO_PIN_1, packet_PA1);
        set_pin_pwm(GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5);
        delay_us_precise(60);

        while(dshot_running_ch3){delay_us_precise(5);}
        send_bdshot(TIM_CHANNEL_3);
        delay_us_precise(40);
        set_pin_input(GPIOA, GPIO_PIN_2);
        //process_bdshot_telemetry(GPIOA, GPIO_PIN_2, packet_PA2);
        set_pin_pwm(GPIOA, GPIO_PIN_2, GPIO_AF2_TIM5);
        delay_us_precise(60);

        while(dshot_running_ch4){delay_us_precise(5);}
        send_bdshot(TIM_CHANNEL_4);
        delay_us_precise(40);
        set_pin_input(GPIOA, GPIO_PIN_3);
        //process_bdshot_telemetry(GPIOA, GPIO_PIN_3, packet_PA3);
        set_pin_pwm(GPIOA, GPIO_PIN_3, GPIO_AF2_TIM5);
        delay_us_precise(60);

        //HAL_UART_Transmit(&huart6, packet_PA0, 3, HAL_MAX_DELAY);
        //HAL_UART_Transmit(&huart6, packet_PA1, 3, HAL_MAX_DELAY);
        //HAL_UART_Transmit(&huart6, packet_PA2, 3, HAL_MAX_DELAY);
        //HAL_UART_Transmit(&huart6, packet_PA3, 3, HAL_MAX_DELAY);
      }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 139;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 230400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void uart_print_int(UART_HandleTypeDef *huart, int value) {
    char buf[12]; // Enough for -2147483648 + \0
    int idx = 0;

    // Handle negative numbers
    if (value < 0) {
        buf[idx++] = '-';
        value = -value;
    }

    // Convert digits to ASCII in reverse
    char temp[10];
    int temp_idx = 0;
    if (value == 0) {
        temp[temp_idx++] = '0';
    } else {
        while (value > 0 && temp_idx < sizeof(temp)) {
            temp[temp_idx++] = '0' + (value % 10);
            value /= 10;
        }
    }

    // Copy digits in correct order
    while (temp_idx > 0) {
        buf[idx++] = temp[--temp_idx];
    }

    // Send buffer via UART
    HAL_UART_Transmit(huart, (uint8_t *)buf, idx, HAL_MAX_DELAY);
}


static inline uint8_t read_telemetry_pin(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) ? 1 : 0;
}

int receive_bdshot_telemetry(uint32_t *telemetry_out, GPIO_TypeDef *port, uint16_t pin) {
    uint32_t value = 0;

    // Wait for line to go low (start bit)
    uint32_t timeout = 0;
    while (read_telemetry_pin(port, pin)) {
        delay_us_precise(1);
        if (++timeout > TELEMETRY_TIMEOUT_US)
            return -1; // Timeout
    }

    delay_ns(500);

    // LSB-first: capture 20 bits
    for (int i = 0; i < 20; i++) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        value |= (read_telemetry_pin(port, pin) << (19-i)); // LSB-first
        //delay_ns(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    }

    *telemetry_out = value;
    return 0;
}

void process_bdshot_telemetry(GPIO_TypeDef *port, uint16_t pin, uint8_t *packet_out) {
	uint32_t telemetry;
	uint16_t telemetry_16bit;
	char telemetry_type;
	float telemetry_value;
    if (receive_bdshot_telemetry(&telemetry, port, pin) == 0) {
  	  uint32_t gcr = decode_gcr_mapping(telemetry);
  	  if (!decode_gcr_20_to_16(gcr, &telemetry_16bit)) {
  		  printf("Invalid GCR encoding.\r\n");
  		  delay_us_precise(10);
  	  }
  	  else {
            int type = parse_edt_frame(telemetry_16bit, &telemetry_type, &telemetry_value);
            if (type == 2) {
          	  uint16_t rpm = (uint16_t)(telemetry_value / 7.0);
          	  packet_out[0] = 0xAA;                      // Start byte
          	  packet_out[1] = rpm & 0xFF;               // LSB
          	  packet_out[2] = (rpm >> 8) & 0xFF;        // MSB
          	  printf("PA0 RPM: %d\r\n", rpm);
            }

            else if (type == 1) {
            	printf("EDT\r\n");
                //printf("EDT: %s = %d\r\n", telemetry_type, (int)telemetry_value);
            }
            else if (type == -1){
                printf("Invalid Telemetry frame.\r\n");
            }
            else if (type == -2){
            	printf("Invalid CRC.\r\n");
            }
            else if (type == -3){
            	printf("Something went wrong.\r\n");
            }
            else {
            	printf("Unknown Error.\r\n");
            }
  	  }
    }
    else {
    	printf("Invalid Telemetry.\r\n");
    }
}

//One cycle is ~6 ns. This is lower limit of function
void delay_ns(uint32_t ns) {
    uint32_t cycles = (SystemCoreClock / 1e9f) * ns;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}
//Telemetry Input
void set_pin_input(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

//DShot Generation
void set_pin_pwm(GPIO_TypeDef *port, uint16_t pin, uint8_t alternate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = alternate;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void prepare_bdshot_buffer(uint16_t frame, uint32_t *dshot_buffer)
{
    uint32_t buffer_index = 0;

    dshot_buffer[buffer_index++] = 0;  // preload entry
    dshot_buffer[buffer_index++] = 0;  // preload entry
    dshot_buffer[buffer_index++] = 0;  // preload entry

    //2Build the actual DSHOT waveform entries
    for (int i = 15; i >= 0; i--)
    {
        uint8_t bit = (frame >> i) & 0x01;
        if (bit)
        {
            dshot_buffer[buffer_index++] = DSHOT_T1L_TICKS;  // logic 1
            //dshot_buffer[buffer_index++] = 0;
        }
        else
        {
            dshot_buffer[buffer_index++] = DSHOT_T0L_TICKS;  // logic 0
            //dshot_buffer[buffer_index++] = 0;
        }
    }

    dshot_buffer[buffer_index++] = 0;  // Final LOW
}


// value_12bit should be 0...4095
uint8_t bdshot_crc(uint16_t value_12bit)
{
    // Optional: Uncomment to assert 12-bit range in debug
    // assert(value_12bit < 4096);

    uint16_t crc = value_12bit ^ (value_12bit >> 4) ^ (value_12bit >> 8);
    crc = ~crc;           // Bitwise NOT
    crc = crc & 0x0F;     // Mask to 4 bits
    return (uint8_t)crc;
}

// Create 16-bit DSHOT frame with correct CRC
uint16_t make_bdshot_frame(uint16_t value, bool telemetry) {
    if (value == 0) {
        return 0x000F; // hardware expects this special frame
    }
    value &= 0x07FF; // 11 bits
    uint16_t frame_no_crc = (value << 1) | (telemetry ? 1 : 0);
    uint8_t crc = bdshot_crc(frame_no_crc);
    return (frame_no_crc << 4) | crc;
}

void queue_bdshot_pulse(uint16_t throttle, bool telemetry, uint32_t *dshot_buffer){
	uint16_t frame = make_bdshot_frame(throttle, telemetry);
	prepare_bdshot_buffer(frame, dshot_buffer);
}

void send_bdshot(uint32_t channel){
	uint32_t *buffer = NULL;
	switch(channel) {
	  case TIM_CHANNEL_1: buffer = dshot_buffer_ch1; break;
	  case TIM_CHANNEL_2: buffer = dshot_buffer_ch2; break;
	  case TIM_CHANNEL_3: buffer = dshot_buffer_ch3; break;
	  case TIM_CHANNEL_4: buffer = dshot_buffer_ch4; break;
	}
    switch (channel) {
        case TIM_CHANNEL_1: while (dshot_running_ch1) delay_us_precise(1); break;
        case TIM_CHANNEL_2: while (dshot_running_ch2) delay_us_precise(1); break;
        case TIM_CHANNEL_3: while (dshot_running_ch3) delay_us_precise(1); break;
        case TIM_CHANNEL_4: while (dshot_running_ch4) delay_us_precise(1); break;
    }
	if (buffer == NULL) Error_Handler();
    if (HAL_TIM_PWM_Start_DMA(&htim5, channel, (uint32_t*)buffer, DSHOT_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
        printf("Error in send_bdshot()\r\n");
    }
    switch (channel) {
        case TIM_CHANNEL_1: dshot_running_ch1 = true; break;
        case TIM_CHANNEL_2: dshot_running_ch2 = true; break;
        case TIM_CHANNEL_3: dshot_running_ch3 = true; break;
        case TIM_CHANNEL_4: dshot_running_ch4 = true; break;
    }
}


void delay_us_precise(double us) {
    uint32_t cycles = (uint32_t)(SystemCoreClock * us / 1e6f);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}


void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

void print_binary(unsigned long value, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        putchar((value & (1UL << i)) ? '1' : '0');
    }
    printf("\r\n");
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        __disable_irq();
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
            dshot_running_ch1 = false;
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
            dshot_running_ch2 = false;
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
            dshot_running_ch3 = false;
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
            dshot_running_ch4 = false;
        }
        __enable_irq();
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    //printf("UART error callback!\r\n");
}

uint8_t calculate_crc(uint16_t value_12bit, const char *protocol) {
    if (value_12bit > 0x0FFF) return 0xFF; // invalid input

    if (protocol[0] == 'B') { // BDShot
        uint16_t value_16bit = value_12bit << 4;
        uint16_t csum = value_16bit;
        csum ^= (csum >> 8);
        csum ^= (csum >> 4);
        return (~csum) & 0xF;
    } else if (protocol[0] == 'D') { // DShot
        uint16_t value = value_12bit;
        uint8_t crc = value ^ (value >> 4) ^ (value >> 8);
        return (~crc) & 0xF;
    }

    return 0xFF; // unsupported protocol
}

/*
 * @param A value that represents a 20 bit GCR that has been mapped to 21 bits.
 * The first "starting" bit can be excluded for compatibility with a slightly cleaned 20 bit value.
 *
 * @return Returns the 20 bit GCR
 */
uint32_t decode_gcr_mapping(uint32_t value) {
    return value ^ (value >> 1);
}

/*
 * @param a 20 bit GCR
 * @param b
 *
 * @return Returns the 20 bit GCR
 *
 * Initialized the whole map to 0xFF. This way, only explicitly defined entries are accepted
 */
int decode_gcr_20_to_16(uint32_t input_20bit, uint16_t *out_value) {
    uint8_t decoding_map[32];
    for (int i = 0; i < 32; ++i) decoding_map[i] = 0xFF; // mark all invalid
    decoding_map[0x19] = 0x0; decoding_map[0x1B] = 0x1; decoding_map[0x12] = 0x2;
    decoding_map[0x13] = 0x3; decoding_map[0x1D] = 0x4; decoding_map[0x15] = 0x5;
    decoding_map[0x16] = 0x6; decoding_map[0x17] = 0x7; decoding_map[0x1A] = 0x8;
    decoding_map[0x09] = 0x9; decoding_map[0x0A] = 0xA; decoding_map[0x0B] = 0xB;
    decoding_map[0x1E] = 0xC; decoding_map[0x0D] = 0xD; decoding_map[0x0E] = 0xE;
    decoding_map[0x0F] = 0xF;

    uint16_t result = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t chunk = (input_20bit >> (15 - i * 5)) & 0x1F;
        if (decoding_map[chunk] == 0xFF) return 0; // invalid chunk
        result = (result << 4) | decoding_map[chunk];
    }

    *out_value = result;
    return 1;
}

int parse_edt_frame(uint16_t frame, char *type_out, float *value_out) {
    if (frame > 0xFFFF || frame < 0){
    	return -1;
    }

    uint16_t data = (frame >> 4) & 0x0FFF;
    uint8_t crc_received = frame & 0x0F;
    uint8_t crc_calculated = calculate_crc(data, "BDShot");
    if (crc_received != crc_calculated) return -2;

    uint8_t exponent = (data >> 9) & 0x07;
    uint16_t base_period = data & 0x1FF;
    if (exponent > 12) return -3;

    uint32_t period_us = base_period << exponent;
    if (period_us == 0) return -3;

    bool is_edt = ((exponent & 1) == 0) && ((base_period & 0x100) == 0);

    if (is_edt) {
        uint8_t telemetry_type = (data >> 8) & 0xF;
        uint8_t telemetry_value = data & 0xFF;

        switch (telemetry_type) {
            case 0x04:
                *value_out = (float)(telemetry_value) / 0.25f;
                if (type_out) strcpy(type_out, "Voltage (V)");
                break;
            case 0x0E:
                if (type_out) strcpy(type_out, "Status Frame");
                *value_out = telemetry_value; // raw value, parse later if needed
                break;
            default:
                *value_out = telemetry_value;
                if (type_out) sprintf(type_out, "Unknown (0x%X)", telemetry_type);
        }

        return 1; // EDT frame
    } else {
        if (base_period == 0 || base_period == 0x1FF) {
            *value_out = 0;
        } else {
            float erpm = 60000000.0f / (float)period_us;
            *value_out = erpm;
        }

        if (type_out) strcpy(type_out, "eRPM");
        return 2; // eRPM frame
    }

    return -3;
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM13 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM13)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  //printf("PeriodElapsedCallback\r\n");
  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
