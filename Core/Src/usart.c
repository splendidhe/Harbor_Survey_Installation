/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stdlib.h"
#include "jy61p.h"

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART2->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */
    USART2->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/***********************************************END*******************************************/

/* 串口1发送数据 */
void printf1(char * fmt,...)
{
    char buffer[400];//数组大小表示数据的大小上限，可视需求更改
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buffer,400,fmt,arg_ptr);
    while(i<399&&buffer[i])
    {
        HAL_UART_Transmit(&huart1,(uint8_t *)&buffer[i],1,0xFFFF);
        i++;
    }
    va_end(arg_ptr);
}

/* 串口1变量初始化 */
uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];         /* 接收缓冲,最大USART1_MAX_RECV_LEN个字节.末字节为换行符 */
uint16_t USART1_RX_STA = 0;                         /* 接收状态标记 */
uint8_t USART1_SingleByte;                          /* 单字节接收 */
uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN];     /* 发送缓冲,最大USART1_MAX_SEND_LEN个字节 */

/* 串口2变量初始化 */
uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN];         /* 接收缓冲,最大USART2_MAX_RECV_LEN个字节.末字节为换行符 */
uint16_t USART2_RX_STA = 0;                         /* 接收状态标记 */
uint8_t USART2_RX_Flag = 0;                         /* 串口2接收标志位 */
uint8_t USART2_SingleByte;                          /* 单字节接收 */
__align(8) uint8_t USART2_TX_BUF[USART2_SEN_LEN];   /* 发送缓冲,最大USART2_SEN_LEN个字节. */

/* 串口4变量初始化 */
uint8_t UART4_RX_BUF[UART4_MAX_RECV_LEN];           /* 接收缓冲,最大UART4_MAX_RECV_LEN个字节.末字节为换行符 */
uint16_t UART4_RX_STA = 0;                          /* 接收状态标记 */
uint8_t UART4_SingleByte;                           /* 单字节接收 */
uint8_t UART4_TX_TempBuf[UART4_MAX_SEND_LEN];       /* 发送缓冲,最大UART4_MAX_SEND_LEN个字节 */

// 串口4发送命令
void sendcmd(char data[3])
{
	static uint8_t tx_buff;
	for(int i=0;i<3;i++)
	{
		tx_buff = data[i];
		HAL_UART_Transmit(&huart4,&tx_buff,1,0Xff);
	}
}

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

AttitudeSensor_TypeDef JY61;  

void USER_USART2_IDLECallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(&huart2);  //停止DMA接收
    USART2_RX_STA = USART2_MAX_RECV_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);  //得到此次接收数据长度
    // for(uint16_t i = 0; i < USART2_RX_DMA_STA; i++)
    // {
    //     USART2_RX_BUF[i] = USART2_RX_DMA_BUF[i];  //把接收到的数据从DMA缓存中读出
    // }
    // for(uint16_t i = USART2_RX_DMA_STA; i < USART2_RX_STA; i++)
    // {
    //     USART2_RX_BUF[i] = 0;  //接收数据不够的情况下，后面的数据补0
    // }
    USART2_RX_Flag = 1;  //接收完成标志位置1
    // printf1("<--EC20_DMA接收的数据--<< \r\n");
    // printf1("%s \r\n", USART2_RX_BUF);

    // if(USART2_RX_Flag == 0)
    // {
    //     HAL_UART_Receive_DMA(&huart2, USART2_RX_BUF, USART2_MAX_RECV_LEN);  //重新开启DMA接收
    // }
    HAL_UART_Receive_DMA(&huart2, USART2_RX_BUF, USART2_MAX_RECV_LEN);  //重新开启DMA接收
    // memset(USART2_RX_DMA_BUF, 0, USART2_DMA_REC_SIZE);  //清空DMA缓存
    // USART2_RX_DMA_STA = 0;  //清空接收状态标记位
    
}

void USER_UART2_IRQHander(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)   
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))  //检测到空闲中断标志
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);  //清除空闲中断标志（否则会一直不断进入中断）
            USER_USART2_IDLECallback(&huart2);  //空闲中断处理函数
        }
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)  //串口打印输出
	{
        USART1_RX_BUF[USART1_RX_STA++] = USART1_SingleByte;
        if(USART1_RX_BUF[USART1_RX_STA-1] == 0x0A && USART1_RX_BUF[USART1_RX_STA-2] == 0x0D)
        {
            
            // HAL_UART_Transmit_IT(&huart1, (uint8_t *)USART1_RX_BUF, USART1_MAX_RECV_LEN);
            memset(USART1_RX_BUF,0x00,USART1_MAX_RECV_LEN);
        }
        // memset(USART1_RX_BUF,0,USART1_MAX_RECV_LEN);
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&USART1_SingleByte, 1);
	}

	// if(huart->Instance==USART2)  //EC20数据上报
	// {
    //     USART2_RX_BUF[USART2_RX_STA++] = USART2_SingleByte;  //接收数据存入缓冲区
    //     if(USART2_RX_STA > USART2_MAX_RECV_LEN)  //防止数据溢出
    //     {
    //         USART2_RX_STA = 0;  //重新开始接收
    //         memset(USART2_RX_BUF,0x00,USART2_MAX_RECV_LEN);  //清空缓冲区
    //     }
    //     if ((USART2_RX_BUF[USART2_RX_STA-1] == 0x0A) && (USART2_RX_BUF[USART2_RX_STA-2] == 0x0D))  //接收到数据
    //     {
    //         // for (uint16_t i = USART2_RX_STA; i < USART2_MAX_RECV_LEN; i++)
    //         // {
    //         //     USART2_RX_BUF[i] = 0x00;  //清空缓冲区后面的数据
    //         // }
    //         // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&USART2_RX_BUF, strlen(USART2_RX_BUF));  //将接收到的数据打印到串口1
    //         for (size_t i = USART2_RX_STA; i <= USART2_MAX_RECV_LEN; i++)
    //         {
    //             USART2_RX_BUF[i] = 0;
    //         }
    //         // memset(USART2_RX_BUF,0x00,USART2_MAX_RECV_LEN);  //清空缓冲区
    //         // printf1("<--EC20--<< %s",USART2_RX_BUF);
    //         USART2_RX_Flag = 1;  //接收到数据
    //     }
	// 	HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_SingleByte, 1);  //再开启接收中断
	// }


	if(huart->Instance==UART4)  //姿态传感器
	{
        if(0X55 != UART4_RX_BUF[0] || 0X51 != UART4_RX_BUF[1])
		{
			memset(UART4_RX_BUF,0x00,sizeof(UART4_RX_BUF)); //清空数组
			HAL_UART_Receive_DMA(&huart4, (uint8_t *)UART4_RX_BUF, 33*2);   //开启接收中断，并保证开启成功 
			return;
		}

		if(0X51 == UART4_RX_BUF[1])
		{
            JY61.angX = (float)(((UART4_RX_BUF[3]<<8)|UART4_RX_BUF[2])/32768.0*16*9.8);
            JY61.angY = (float)(((UART4_RX_BUF[5]<<8)|UART4_RX_BUF[4])/32768.0*16*9.8);
            JY61.angZ = (float)(((UART4_RX_BUF[7]<<8)|UART4_RX_BUF[6])/32768.0*16*9.8);
            // printf("aX:%f,  aY:%f,  aZ:%f  \r\n",aX,aY,aZ);
			
		}
		if(0X52 == UART4_RX_BUF[12])
		{
            JY61.accX = (float)(((UART4_RX_BUF[14]<<8)|UART4_RX_BUF[13])/32768.0*2000);
            JY61.accY = (float)(((UART4_RX_BUF[16]<<8)|UART4_RX_BUF[15])/32768.0*2000);
            JY61.accZ = (float)(((UART4_RX_BUF[18]<<8)|UART4_RX_BUF[17])/32768.0*2000);
            // printf("wX:%f,  wY:%f,  wZ:%f  \r\n",wX,wY,wZ);
        
		}
		if(0X53 == UART4_RX_BUF[23])
		{
            JY61.roll  = (float)(((UART4_RX_BUF[25]<<8)|UART4_RX_BUF[24])/32768.0*180);
            JY61.pitch = (float)(((UART4_RX_BUF[27]<<8)|UART4_RX_BUF[26])/32768.0*180);
            JY61.yaw   = (float)(((UART4_RX_BUF[29]<<8)|UART4_RX_BUF[28])/32768.0*180);
            // printf("RollX:%f,  PitchY:%f,  YawZ:%f  \r\n",RollX,PitchY,YawZ);
		}
		memset(UART4_RX_BUF,0x00,sizeof(UART4_RX_BUF)); //清空数组
		HAL_UART_Receive_DMA(&huart4, (uint8_t *)UART4_RX_BUF, 33*2);
	}

	// if(huart->Instance==USART6)  //激光测距
	// {
	// 	static uint8_t *p;
	// 	static int distance;
	// 	// if(UART5_Flag == 1)
	// 	// {
    //   	// UART5_Flag = 0;
    //   	UART6_RX_STA = strlen((char*)UART6_RX_BUF);
	// 	if(UART6_RX_BUF[UART6_RX_STA-1] == 'm')
	// 	{
	// 		p = (unsigned char*)strstr((char*)UART6_RX_BUF,"d:");
	// 		while(*p != 'm')
	// 		{
	// 			if(*p >= '0' && *p <= '9')
	// 			{
	// 				distance = distance * 10 + (*p - '0');
	// 			}
	// 			p++;		
	// 		}
	// 	}
	// 	waterlevel = distance;
	// 	HAL_UART_Receive_IT(&huart6, (uint8_t *)UART6_RX_BUF, UART6_MAX_RECV_LEN*2);
	// }
}
/* USER CODE END 1 */
