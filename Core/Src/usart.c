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
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");          /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while ((USART2->SR & 0X40) == 0);               /* �ȴ���һ���ַ�������� */
    USART2->DR = (uint8_t)ch;                       /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}
#endif
/***********************************************END*******************************************/

/* ����1�������� */
void printf1(char * fmt,...)
{
    char buffer[400];//�����С��ʾ���ݵĴ�С���ޣ������������
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

/* ����1������ʼ�� */
uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];         /* ���ջ���,���USART1_MAX_RECV_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
uint16_t USART1_RX_STA = 0;                         /* ����״̬��� */
uint8_t USART1_SingleByte;                          /* ���ֽڽ��� */
uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN];     /* ���ͻ���,���USART1_MAX_SEND_LEN���ֽ� */

/* ����2������ʼ�� */
uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN];         /* ���ջ���,���USART2_MAX_RECV_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
uint16_t USART2_RX_STA = 0;                         /* ����״̬��� */
uint8_t USART2_RX_Flag = 0;                         /* ����2���ձ�־λ */
uint8_t USART2_SingleByte;                          /* ���ֽڽ��� */
__align(8) uint8_t USART2_TX_BUF[USART2_SEN_LEN];   /* ���ͻ���,���USART2_SEN_LEN���ֽ�. */

/* ����4������ʼ�� */
uint8_t UART4_RX_BUF[UART4_MAX_RECV_LEN];           /* ���ջ���,���UART4_MAX_RECV_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
uint16_t UART4_RX_STA = 0;                          /* ����״̬��� */
uint8_t UART4_SingleByte;                           /* ���ֽڽ��� */
uint8_t UART4_TX_TempBuf[UART4_MAX_SEND_LEN];       /* ���ͻ���,���UART4_MAX_SEND_LEN���ֽ� */

// ����4��������
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
    HAL_UART_DMAStop(&huart2);  //ֹͣDMA����
    USART2_RX_STA = USART2_MAX_RECV_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);  //�õ��˴ν������ݳ���
    // for(uint16_t i = 0; i < USART2_RX_DMA_STA; i++)
    // {
    //     USART2_RX_BUF[i] = USART2_RX_DMA_BUF[i];  //�ѽ��յ������ݴ�DMA�����ж���
    // }
    // for(uint16_t i = USART2_RX_DMA_STA; i < USART2_RX_STA; i++)
    // {
    //     USART2_RX_BUF[i] = 0;  //�������ݲ���������£���������ݲ�0
    // }
    USART2_RX_Flag = 1;  //������ɱ�־λ��1
    // printf1("<--EC20_DMA���յ�����--<< \r\n");
    // printf1("%s \r\n", USART2_RX_BUF);

    // if(USART2_RX_Flag == 0)
    // {
    //     HAL_UART_Receive_DMA(&huart2, USART2_RX_BUF, USART2_MAX_RECV_LEN);  //���¿���DMA����
    // }
    HAL_UART_Receive_DMA(&huart2, USART2_RX_BUF, USART2_MAX_RECV_LEN);  //���¿���DMA����
    // memset(USART2_RX_DMA_BUF, 0, USART2_DMA_REC_SIZE);  //���DMA����
    // USART2_RX_DMA_STA = 0;  //��ս���״̬���λ
    
}

void USER_UART2_IRQHander(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)   
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))  //��⵽�����жϱ�־
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);  //��������жϱ�־�������һֱ���Ͻ����жϣ�
            USER_USART2_IDLECallback(&huart2);  //�����жϴ�����
        }
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)  //���ڴ�ӡ���
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

	// if(huart->Instance==USART2)  //EC20�����ϱ�
	// {
    //     USART2_RX_BUF[USART2_RX_STA++] = USART2_SingleByte;  //�������ݴ��뻺����
    //     if(USART2_RX_STA > USART2_MAX_RECV_LEN)  //��ֹ�������
    //     {
    //         USART2_RX_STA = 0;  //���¿�ʼ����
    //         memset(USART2_RX_BUF,0x00,USART2_MAX_RECV_LEN);  //��ջ�����
    //     }
    //     if ((USART2_RX_BUF[USART2_RX_STA-1] == 0x0A) && (USART2_RX_BUF[USART2_RX_STA-2] == 0x0D))  //���յ�����
    //     {
    //         // for (uint16_t i = USART2_RX_STA; i < USART2_MAX_RECV_LEN; i++)
    //         // {
    //         //     USART2_RX_BUF[i] = 0x00;  //��ջ��������������
    //         // }
    //         // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&USART2_RX_BUF, strlen(USART2_RX_BUF));  //�����յ������ݴ�ӡ������1
    //         for (size_t i = USART2_RX_STA; i <= USART2_MAX_RECV_LEN; i++)
    //         {
    //             USART2_RX_BUF[i] = 0;
    //         }
    //         // memset(USART2_RX_BUF,0x00,USART2_MAX_RECV_LEN);  //��ջ�����
    //         // printf1("<--EC20--<< %s",USART2_RX_BUF);
    //         USART2_RX_Flag = 1;  //���յ�����
    //     }
	// 	HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_SingleByte, 1);  //�ٿ��������ж�
	// }


	if(huart->Instance==UART4)  //��̬������
	{
        if(0X55 != UART4_RX_BUF[0] || 0X51 != UART4_RX_BUF[1])
		{
			memset(UART4_RX_BUF,0x00,sizeof(UART4_RX_BUF)); //�������
			HAL_UART_Receive_DMA(&huart4, (uint8_t *)UART4_RX_BUF, 33*2);   //���������жϣ�����֤�����ɹ� 
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
		memset(UART4_RX_BUF,0x00,sizeof(UART4_RX_BUF)); //�������
		HAL_UART_Receive_DMA(&huart4, (uint8_t *)UART4_RX_BUF, 33*2);
	}

	// if(huart->Instance==USART6)  //������
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
