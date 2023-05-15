/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht11.h"
#include "string.h"
#include "stdio.h"
#include "sys.h"
#include "delay.h"
#include "jy61p.h"
#include "ec20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 uint8_t temperature;  // �¶�
 uint8_t humidity;   // ʪ��
 uint16_t Adc_buffer[6]; // ADC�ɼ����ݻ���
/*
    ��������
    Adc_buffer[0]  --  �׵紫����
    Adc_buffer[1]  --  ���ٴ�����
    Adc_buffer[2]  --  ���򴫸���
    Adc_buffer[3]  --  �Ƕȴ�����
    Adc_buffer[4]  --  ph������
    Adc_buffer[5]  --  ��δ�����
*/

// ��̬У׼����
char YAWCMD1[3] = {0XFF,0XAA,0X52};
char ACCCMD1[3] = {0XFF,0XAA,0X67};
extern AttitudeSensor_TypeDef JY61;  
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

float wind_direction_check(float winddirection);  // ������
void ADC_Process_Function(void);  // ADC���ݴ�����
float turbidity_check(float turbidity);  // �Ƕȼ��
float PH_check(float PH);  // PHֵ���
float raindrop_check(float raindrop);  // ����ǿ�ȼ��
void JY61P_Output(void);  // JY61P�������
void HCSR04_Detect(void);
uint8_t data_len,datastr[10],send_jason[200];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
    delay_init(168);  // ��ʼ����ʱ����
    // while(dht11_init())  // ��ʼ��DHT11
    // {
    //     printf("DHT11 init failed!\r\n");
    //     HAL_Delay(1000);
    // }
    sendcmd(ACCCMD1);				// �ȴ�JY61��ʼ�����
    delay_ms(300);  
    sendcmd(YAWCMD1);
    delay_ms(300);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)Adc_buffer,6);                 // ����ADC DMA����
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&USART1_SingleByte, 1);     // ����1�жϽ���
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                        // ����2ʹ�ܿ����ж�
    HAL_UART_Receive_DMA(&huart2, USART2_RX_BUF, USART2_MAX_RECV_LEN);  // ����2�жϽ���  DMA
    HAL_UART_Receive_DMA(&huart4, (uint8_t *)UART4_RX_BUF, 33);         // ����4�жϽ���  DMA
    while(EC20_init_check(0) != 0x0F);  // EC20��ʼ�����

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        dht11_read_data(&temperature, &humidity);                                 // ��ȡ��ʪ��
        printf1("temperature: %d, humidity: %d\r\n", temperature, humidity);    // ��ӡ��ʪ��
        ADC_Process_Function();   // ADC���ݴ�����
        JY61P_Output();     // JY61P���
        HCSR04_Detect();    // ���������
        PARSE_GPRMC();      // ����GPRMC
        // GPS_Analysis();     // GPS���ݽ���
        Mqtt_Senddata();    // MQTT��������                                                                                                                          // ����3�жϽ���
        HAL_UART_RxCpltCallback(&huart4);
        HAL_Delay(1000);          // ��ʱ1s
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */


void HCSR04_Detect(void)  //���������
{  
    distance = HCSR04_measure();  //��������
    printf1("Distance = %dMM\r\n",distance);   //��ӡ����
}

void JY61P_Output(void)
{
    static uint8_t updata = 0;
    // char str[40];
    updata++;
    if(updata>151)          // �Զ�У׼ʱ�� = 150 * ��������
    {	                    // �ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
        updata = 0;
        printf1("���ڽ��м��ٶ�У׼\r\n");
        sendcmd(ACCCMD1);
        printf1("���ٶ�У׼���\r\n");
        delay_ms(300);
        printf1("����Z��Ƕ�����\r\n");
        sendcmd(YAWCMD1);
        printf1("Z��Ƕ��������\r\n");
        delay_ms(300);
    }
    printf1("-JY61P:");
    printf1("Pitch = %.3f 	",JY61.pitch);
    printf1("Roll = %.3f 	",JY61.roll);
    printf1("Yaw = %.3f 	\r\n",JY61.yaw);
}

/**
  * @brief  ADC���ݴ�����
  * @param  None
  * @retval None
  */
void ADC_Process_Function(void)
{
    static float temp = 0;
    printf1("XX������: %d\r\n", Adc_buffer[0]);

    printf1("���ٴ�����: %d     ", Adc_buffer[1]);
    temp = (3.3f * (float)Adc_buffer[1] / 4095.0f) * 1000.0f;  // ��ѹֵ
    windspeed = 0.027f * temp;  // ����ֵ
    printf1("����ֵ: %f\r\n", windspeed);

    /* ���򴫸��� */
    printf1("���򴫸���: %d     ", Adc_buffer[2]);
    temp = Adc_buffer[2] * 360 / 2482;  // �Ƕ�ֵ
    winddirection = wind_direction_check(temp);  // ����ֵ
    printf1("����ֵ: %f\r\n", winddirection);

    /* �Ƕȴ����� */
    printf1("�Ƕȴ�����: %d     ", Adc_buffer[3]);
    temp = (float)Adc_buffer[3] * 3.3f / 4095.0f;  // ��ѹֵ
    temp = temp * 100 / 3.3f;  // �Ƕ�ֵ
    turbidity = turbidity_check(temp);  // �Ƕ�ֵ
    printf1("�Ƕ�ֵ: %f\r\n", turbidity);

    /* PH������ */
    printf1("PH������: %d     ", Adc_buffer[4]);
    temp = (float)Adc_buffer[4] * 3.3f / 4095.0f;  // ��ѹֵ
    temp = -5.887 * temp + 21.258;  // PHֵ
    ph = PH_check(temp);  // PHֵ
    printf1("PHֵ: %f\r\n", ph);

    /* ��δ����� */
    printf1("��δ�����: %d     ", Adc_buffer[5]);
    temp = (float)Adc_buffer[5] * 3.3f / 4095.0f;  // ��ѹֵ
    raindrop = raindrop_check(temp);  // ��εȼ�
    printf1("��εȼ�: %f\r\n", raindrop);

}

float wind_direction_check(float winddirection)  // ������ֵ���
{
    if(winddirection > 360)
    {
        winddirection = 360;
    }

    return winddirection;
}

float turbidity_check(float turbidity)  // �Ƕ���ֵ���
{
    if(turbidity > 100)
    {
        turbidity = 100;
    }

    return turbidity;
}

float PH_check(float PH)  // PH��ֵ���
{
    if(PH > 14)
    {
        PH = 14;
    }
    else if(PH < 0)
    {
        PH = 0;
    }

    return PH;
}

float raindrop_check(float rain)  // ��εȼ��ж�
{
    if(rain < 0.0f)
    {
        rain = 0.0f;
    }
    else if(rain < 1.1f)
    {
        rain = 1.0f;
    }
    else if(rain < 2.2f)
    {
        rain = 2.0f;
    }
    else
    {
        rain = 3.0f;
    }

    return rain;
}

// void printf1(char *data)
// {
//     HAL_UART_Transmit(&huart1, (const uint8_t *)data, sizeof(data), 0xffff);
// }

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
