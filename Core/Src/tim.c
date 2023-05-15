/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "gpio.h"
unsigned int  dta;

/* USER CODE END 0 */

TIM_HandleTypeDef htim6;

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


void for_delay_us(uint16_t nus)//微秒级延时（循环）
{
	uint16_t i;
	for(i=0;i<nus;i++){
	 __NOP();}
}
 
void  Start(void) //启动超声波模块
{
    HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,SET); //拉高
    for_delay_us(15);          //15us延时
    HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,RESET);  //拉低
}

int Get(void)
{       
    return HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin); //获得Echo的电平高低
}

void start(void)//开始计时
{
    __HAL_TIM_SetCounter(&htim6,0);   //清0开始计数
    HAL_TIM_Base_Start_IT(&htim6);   // 开启定时器更新中断       
}
 
unsigned int get_stop(void)//获取停止之后的数据
{  
   unsigned int  num,dta;
   HAL_TIM_Base_Stop_IT (&htim6);       //关闭定时器更新中断
   num = __HAL_TIM_GetCounter(&htim6);  //获得计数值
   dta = (num*346)/20000;               //cm为单位
    return dta;
}


int HCSR04_measure(void)
{
	Start();            //启动超声波模块
	while(!Get());      //当Echo返回0，等待,变为1继续        
	start();            //开始计时
	while(Get());		//当Echo为1计数并等待  ,变为0继续     
	dta = get_stop();   //获取停止之后的数据
	return dta;
}

/* USER CODE END 1 */
