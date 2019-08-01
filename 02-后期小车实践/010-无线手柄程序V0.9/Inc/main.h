/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Button1_Pin GPIO_PIN_0
#define Button1_GPIO_Port GPIOC
#define Button1_EXTI_IRQn EXTI0_IRQn
#define Button2_Pin GPIO_PIN_1
#define Button2_GPIO_Port GPIOC
#define Button2_EXTI_IRQn EXTI1_IRQn
#define Button3_Pin GPIO_PIN_2
#define Button3_GPIO_Port GPIOC
#define Button3_EXTI_IRQn EXTI2_IRQn
#define Button4_Pin GPIO_PIN_3
#define Button4_GPIO_Port GPIOC
#define Button4_EXTI_IRQn EXTI3_IRQn
#define ADC_POWER_Pin GPIO_PIN_2
#define ADC_POWER_GPIO_Port GPIOA
#define RIGHT1_Pin GPIO_PIN_4
#define RIGHT1_GPIO_Port GPIOA
#define RIGHT2_Pin GPIO_PIN_5
#define RIGHT2_GPIO_Port GPIOA
#define LEFT1_Pin GPIO_PIN_6
#define LEFT1_GPIO_Port GPIOA
#define LEFT2_Pin GPIO_PIN_7
#define LEFT2_GPIO_Port GPIOA
#define Button_LEFT_Pin GPIO_PIN_4
#define Button_LEFT_GPIO_Port GPIOC
#define Button_LEFT_EXTI_IRQn EXTI4_IRQn
#define Button_RIGHT_Pin GPIO_PIN_5
#define Button_RIGHT_GPIO_Port GPIOC
#define Button_RIGHT_EXTI_IRQn EXTI9_5_IRQn
#define Button5_Pin GPIO_PIN_6
#define Button5_GPIO_Port GPIOC
#define Button5_EXTI_IRQn EXTI9_5_IRQn
#define Button6_Pin GPIO_PIN_7
#define Button6_GPIO_Port GPIOC
#define Button6_EXTI_IRQn EXTI9_5_IRQn
#define Button7_Pin GPIO_PIN_8
#define Button7_GPIO_Port GPIOC
#define Button7_EXTI_IRQn EXTI9_5_IRQn
#define Button8_Pin GPIO_PIN_9
#define Button8_GPIO_Port GPIOC
#define Button8_EXTI_IRQn EXTI9_5_IRQn
#define NRF_CSN_Pin GPIO_PIN_15
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_8
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_9
#define NRF_CE_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
