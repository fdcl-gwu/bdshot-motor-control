/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_tim5_ch1;

extern DMA_HandleTypeDef hdma_tim5_ch2;

extern DMA_HandleTypeDef hdma_tim5_ch3_up;

extern DMA_HandleTypeDef hdma_tim5_ch4_trig;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                    /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief TIM_PWM MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_pwm: TIM_PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM5)
  {
    /* USER CODE BEGIN TIM5_MspInit 0 */

    /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* TIM5 DMA Init */
    /* TIM5_CH1 Init */
    hdma_tim5_ch1.Instance = DMA1_Stream2;
    hdma_tim5_ch1.Init.Channel = DMA_CHANNEL_6;
    hdma_tim5_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim5_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim5_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim5_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim5_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim5_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim5_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim5_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim5_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC1],hdma_tim5_ch1);

    /* TIM5_CH2 Init */
    hdma_tim5_ch2.Instance = DMA1_Stream4;
    hdma_tim5_ch2.Init.Channel = DMA_CHANNEL_6;
    hdma_tim5_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim5_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim5_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim5_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim5_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim5_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim5_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim5_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim5_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC2],hdma_tim5_ch2);

    /* TIM5_CH3_UP Init */
    hdma_tim5_ch3_up.Instance = DMA1_Stream0;
    hdma_tim5_ch3_up.Init.Channel = DMA_CHANNEL_6;
    hdma_tim5_ch3_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim5_ch3_up.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim5_ch3_up.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim5_ch3_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim5_ch3_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim5_ch3_up.Init.Mode = DMA_NORMAL;
    hdma_tim5_ch3_up.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim5_ch3_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim5_ch3_up) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC3],hdma_tim5_ch3_up);
    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_UPDATE],hdma_tim5_ch3_up);

    /* TIM5_CH4_TRIG Init */
    hdma_tim5_ch4_trig.Instance = DMA1_Stream1;
    hdma_tim5_ch4_trig.Init.Channel = DMA_CHANNEL_6;
    hdma_tim5_ch4_trig.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim5_ch4_trig.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim5_ch4_trig.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim5_ch4_trig.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim5_ch4_trig.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim5_ch4_trig.Init.Mode = DMA_NORMAL;
    hdma_tim5_ch4_trig.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim5_ch4_trig.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim5_ch4_trig) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC4],hdma_tim5_ch4_trig);
    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_TRIGGER],hdma_tim5_ch4_trig);

    /* USER CODE BEGIN TIM5_MspInit 1 */

    /* USER CODE END TIM5_MspInit 1 */

  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM5)
  {
    /* USER CODE BEGIN TIM5_MspPostInit 0 */

    /* USER CODE END TIM5_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    PA2     ------> TIM5_CH3
    PA3     ------> TIM5_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM5_MspPostInit 1 */

    /* USER CODE END TIM5_MspPostInit 1 */
  }

}
/**
  * @brief TIM_PWM MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_pwm: TIM_PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM5)
  {
    /* USER CODE BEGIN TIM5_MspDeInit 0 */

    /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* TIM5 DMA DeInit */
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC3]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_UPDATE]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC4]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_TRIGGER]);
    /* USER CODE BEGIN TIM5_MspDeInit 1 */

    /* USER CODE END TIM5_MspDeInit 1 */
  }

}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART6)
  {
    /* USER CODE BEGIN USART6_MspInit 0 */

    /* USER CODE END USART6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN USART6_MspInit 1 */

    /* USER CODE END USART6_MspInit 1 */

  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART6)
  {
    /* USER CODE BEGIN USART6_MspDeInit 0 */

    /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USER CODE BEGIN USART6_MspDeInit 1 */

    /* USER CODE END USART6_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
