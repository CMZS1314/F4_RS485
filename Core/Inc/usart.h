/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
typedef enum
{
  KEY_UP   = 0,
  KEY_DOWN = 1,
}KEYState_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/
#define KEY1_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY1_GPIO_PIN                 GPIO_PIN_0
#define KEY1_GPIO                     GPIOE
#define KEY1_DOWN_LEVEL               0  /* 根据原理图设计，KEY1按下时引脚为低电平，所以这里设置为0 */

#define KEY2_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY2_GPIO_PIN                 GPIO_PIN_1
#define KEY2_GPIO                     GPIOE
#define KEY2_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY3_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY3_GPIO_PIN                 GPIO_PIN_2
#define KEY3_GPIO                     GPIOE
#define KEY3_DOWN_LEVEL               0  /* 根据原理图设计，KEY3按下时引脚为低电平，所以这里设置为0 */

#define KEY4_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY4_GPIO_PIN                 GPIO_PIN_3
#define KEY4_GPIO                     GPIOE
#define KEY4_DOWN_LEVEL               0  /* 根据原理图设计，KEY4按下时引脚为低电平，所以这里设置为0 */

#define KEY5_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY5_GPIO_PIN                 GPIO_PIN_4
#define KEY5_GPIO                     GPIOE
#define KEY5_DOWN_LEVEL               0  /* 根据*/

/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
void KEY_GPIO_Init(void);

/* 函数声明 ------------------------------------------------------------------*/
void MX_USARTx_Init(void);
//void KEY_GPIO_Init(void);
KEYState_TypeDef KEY1_StateRead(void);
KEYState_TypeDef KEY2_StateRead(void);
KEYState_TypeDef KEY3_StateRead(void);
KEYState_TypeDef KEY4_StateRead(void);
KEYState_TypeDef KEY5_StateRead(void);
/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
