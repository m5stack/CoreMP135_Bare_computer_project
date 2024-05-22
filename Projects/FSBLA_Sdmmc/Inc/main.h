/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    FSBLA/FSBLA_Sdmmc1/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32mp13xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32mp13xx_ll_etzpc.h"

// #include "stm32mp13xx_disco.h"
// #include "stm32mp13xx_disco_stpmic1.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
#define AF_SECURE_USART_TEST

extern int Debug_s;
extern uint8_t msgBuffer[1024];
extern USART_HandleTypeDef husart6;
#define LOG_PRINT(fmt, ...) do{\
	if(Debug_s)\
	{\
		int msg_size = sprintf((char*)msgBuffer,fmt"  [info:%s:%d] [%s]\n", ##__VA_ARGS__, __FILE__, __LINE__, __FUNCTION__);\
        HAL_USART_Transmit(&husart6, (uint8_t *)msgBuffer, msg_size, 0xFFFF);\
	}\
}while(0);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
