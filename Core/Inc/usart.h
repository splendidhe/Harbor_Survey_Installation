/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

typedef struct {
  float roll;  //翻滚角
  float pitch; //俯仰角
  float yaw;   //偏航角
  float accX;  //X轴加速度
  float accY;  //Y轴加速度
  float accZ;  //Z轴加速度
  float angX;  //X轴角速度
  float angY;  //Y轴角速度
  float angZ;  //Z轴角速度
} AttitudeSensor_TypeDef;

/* USART1 */
#define USART1_MAX_RECV_LEN     200                        /* 最大接收缓存字节数 200 */
#define USART1_MAX_SEND_LEN     200                        /* 最大发送缓存字节数 200 */
extern uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];         /* 接收缓冲,最大USART1_MAX_RECV_LEN个字节.末字节为换行符 */
extern uint16_t USART1_RX_STA;                             /* 接收状态标记 */
extern uint8_t USART1_SingleByte;                          /* 单字节接收 */
extern uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN];     /* 发送缓冲,最大USART1_MAX_SEND_LEN个字节 */

/* USART2 */
#define USART2_MAX_RECV_LEN     400                        /* 最大接收缓存字节数 1024 */
#define USART2_SEN_LEN          400                        /* 最大发送缓存字节数 1024 */
extern uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN];         /* 接收缓冲,最大USART2_MAX_RECV_LEN个字节.末字节为换行符 */
extern uint16_t USART2_RX_STA;                             /* 接收状态标记 */
extern __align(8) uint8_t USART2_TX_BUF[USART2_SEN_LEN];   /* 发送缓冲,最大USART2_MAX_SEND_LEN个字节 */
extern uint8_t USART2_RX_Flag;                         	   /* 串口2接收标志位 */
extern uint8_t USART2_SingleByte;                          /* 单字节接收 */
extern uint8_t EC20_BUF[];

#define USART2_DMA_REC_SIZE 400
extern uint8_t USART2_RX_DMA_BUF[USART2_DMA_REC_SIZE];                   /* DMA接收缓冲,最大USART2_MAX_RECV_LEN个字节.末字节为换行符 */
extern uint16_t USART2_RX_DMA_STA;                                        /* DMA接收状态标记 */


/* UART4 */
#define UART4_MAX_RECV_LEN      33                        /* 最大接收缓存字节数 200 */
#define UART4_MAX_SEND_LEN      20                        /* 最大发送缓存字节数 200 */
extern uint8_t UART4_RX_BUF[UART4_MAX_RECV_LEN];           /* 接收缓冲,最大UART4_MAX_RECV_LEN个字节.末字节为换行符 */
extern uint16_t UART4_RX_STA;                              /* 接收状态标记 */
extern uint8_t UART4_SingleByte;                           /* 接收单个字节 */
extern uint8_t UART4_TX_TempBuf[UART4_MAX_SEND_LEN];       /* 发送缓冲,最大UART4_MAX_SEND_LEN个字节 */

extern int waterlevel;

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

void sendcmd(char data[3]);
void EnableUsart3_It(void);
void printf1(char *fmt, ...);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void USER_UART2_IRQHander(UART_HandleTypeDef *huart);

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

