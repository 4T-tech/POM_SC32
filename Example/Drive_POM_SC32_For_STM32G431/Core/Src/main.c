/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USER_REG_CMD 1
//#define USER_HEX_CMD 1
//#define USER_STR_CMD 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef USER_REG_CMD
//REG list
#define RESET_REG 0x00
#define SERVO1_REG 0x01
#define SERVO2_REG 0x02
#define SERVO3_REG 0x03
#define SERVO4_REG 0x04
#define SERVO5_REG 0x05
#define SERVO6_REG 0x06
#define SERVO7_REG 0x07
#define SERVO8_REG 0x08
#define SERVO9_REG 0x09
#define SERVO10_REG 0x0A
#define SERVO11_REG 0x0B
#define SERVO12_REG 0x0C
#define SERVO13_REG 0x0D
#define SERVO14_REG 0x0E
#define SERVO15_REG 0x0F
#define SERVO16_REG 0x10
#define SERVO17_REG 0x11
#define SERVO18_REG 0x12
#define SERVO19_REG 0x13
#define SERVO20_REG 0x14
#define SERVO21_REG 0x15
#define SERVO22_REG 0x16
#define SERVO23_REG 0x17
#define SERVO24_REG 0x18
#define SERVO25_REG 0x19
#define SERVO26_REG 0x1A
#define SERVO27_REG 0x1B
#define SERVO28_REG 0x1C
#define SERVO29_REG 0x1D
#define SERVO30_REG 0x1E
#define SERVO31_REG 0x1F
#define SERVO32_REG 0x20
#define RED_REG 0x21
#define GREEN_REG 0x22
#define BLUE_REG 0x23
#define LED_REG 0x24
#define GROUP1_REG 0x25
#define GROUP2_REG 0x26
#define GROUP3_REG 0x27
#define GROUP4_REG 0x28
#define GROUP5_REG 0x29
#define GROUP6_REG 0x2A
#define TEMP1_REG 0x2B
#define TEMP2_REG 0x2C
#define TEMP3_REG 0x2D
#define TEMP4_REG 0x2E
#define TEMP5_REG 0x2F
#define TEMP6_REG 0x30
#define TEMP7_REG 0x31
#define TEMP8_REG 0x32
#define TEMP9_REG 0x33
#define TEMP10_REG 0x34
#define TEMP11_REG 0x35
#define TEMP12_REG 0x36
#define TEMP13_REG 0x37
#define TEMP14_REG 0x38
#define TEMP15_REG 0x39
#define TEMP16_REG 0x3A
#define TEMP17_REG 0x3B
#define TEMP18_REG 0x3C
#define TEMP19_REG 0x3D
#define TEMP20_REG 0x3E
#define TEMP21_REG 0x3F
#define TEMP22_REG 0x40
#define TEMP23_REG 0x41
#define TEMP24_REG 0x42
#define TEMP25_REG 0x43
#define TEMP26_REG 0x44
#define TEMP27_REG 0x45
#define TEMP28_REG 0x46
#define TEMP29_REG 0x47
#define TEMP30_REG 0x48
#define TEMP31_REG 0x49
#define TEMP32_REG 0x4A
#define ACTION_TIME_REG 0x4B
#define ACTION_GROUP_REG 0x4C
#define ACTION_NUM_REG 0x4D
#define FUNCTION_WRITE_REG 0x4E
#define FUNCTION_READ_REG 0x4F
#endif
#ifdef USER_HEX_CMD
uint8_t HEX_CMD_FUNC(uint16_t *NUM)
{
    uint8_t TX_BUF[65];
     uint8_t RX_BUF[2] = {0x00,0x00};
    memset(TX_BUF,0,65);
    for(uint8_t Temp_num = 0;Temp_num < 32;Temp_num++)
    {
        TX_BUF[Temp_num*2] = NUM[Temp_num] >> 8;
        TX_BUF[Temp_num*2+1] = NUM[Temp_num] &= 0xFF;
        TX_BUF[64] = TX_BUF[64] + TX_BUF[Temp_num*2] +TX_BUF[Temp_num*2+1];
    }
    HAL_UART_Transmit(&huart2,TX_BUF,65,5);
    HAL_UART_Receive(&huart2,RX_BUF,2,10);
    if(RX_BUF[0] == 'E' && RX_BUF[1] == 'R')
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
#endif


double Temp_Time;
//frame head
#define FRAME_HEAD 0xAA


uint8_t REG_CMD_FUNC(uint8_t ADDR,uint16_t NUM)
{
    uint8_t TX_BUF[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t RX_BUF[2] = {0x00,0x00};
    TX_BUF[0] = FRAME_HEAD;
    TX_BUF[1] = 0x00;
    TX_BUF[2] = ADDR;
    TX_BUF[3] = NUM >> 8;
    TX_BUF[4] = NUM &= 0xFF;
    TX_BUF[5] = TX_BUF[0]+TX_BUF[1]+TX_BUF[2]+TX_BUF[3]+TX_BUF[4];
    HAL_UART_Transmit(&huart2,TX_BUF,6,5);
    HAL_UART_Receive(&huart2,RX_BUF,2,10);
    if(RX_BUF[0] == 'E' && RX_BUF[1] == 'R')
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

uint8_t Angle_to_Duty(float Angle)
{
    if(Angle > 180.0)Angle = 180.0;
    return (uint8_t)round((double)(Angle * 5) / 9 +25);
}


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    //SET_GROUP_FREQ
    #if USER_REG_CMD || USER_HEX_CMD
    REG_CMD_FUNC(0x25,50);
    REG_CMD_FUNC(0x26,50);
    REG_CMD_FUNC(0x27,50);
    REG_CMD_FUNC(0x28,50);
    REG_CMD_FUNC(0x29,50);
    REG_CMD_FUNC(0x2A,50);
    #elif USER_STR_CMD
    HAL_UART_Transmit(&huart2,"PWMGroup1=50,PWMGroup2=50,PWMGroup3=50,PWMGroup4=50,PWMGroup5=50,PWMGroup6=50",strlen("PWMGroup1=50,PWMGroup2=50,PWMGroup3=50,PWMGroup4=50,PWMGroup5=50,PWMGroup6=50"),5);
    #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      
      #if USER_REG_CMD
      //SET_SERVO
      Temp_Time = HAL_GetTick() / 1000.0;
      REG_CMD_FUNC(SERVO1_REG,Angle_to_Duty((sin(Temp_Time)+1)*90));
      HAL_Delay(20);
      #endif
      
      #if USER_HEX_CMD
      Temp_Time = HAL_GetTick() / 1000.0;
      uint16_t servo_buf[32];
      for(uint8_t Temp_num_for_main1 = 0 ;Temp_num_for_main1 < 32 ;Temp_num_for_main1++)
      {
          servo_buf[Temp_num_for_main1] = Angle_to_Duty((sin(Temp_Time)+1)*90);
      }
      HEX_CMD_FUNC(servo_buf);
      HAL_Delay(20);
      #endif
      
      #if USER_STR_CMD
      Temp_Time = HAL_GetTick() / 28000.0;
      uint16_t servo_buf[32];
      uint8_t Tx_Buf[512];
      for(uint8_t Temp_num_for_main1 = 0 ;Temp_num_for_main1 < 32 ;Temp_num_for_main1++)
      {
          servo_buf[Temp_num_for_main1] = Angle_to_Duty((sin(Temp_Time)+1)*90);
      }
      sprintf(Tx_Buf,   "PWM1_Duty=%d,PWM2_Duty=%d,PWM3_Duty=%d,PWM4_Duty=%d,PWM5_Duty=%d,PWM6_Duty=%d,PWM7_Duty=%d,PWM8_Duty=%d,PWM9_Duty=%d,PWM10_Duty=%d,PWM11_Duty=%d,PWM12_Duty=%d,PWM13_Duty=%d,PWM14_Duty=%d,PWM15_Duty=%d,PWM16_Duty=%d,PWM17_Duty=%d,PWM18_Duty=%d,PWM19_Duty=%d,PWM20_Duty=%d,PWM21_Duty=%d,PWM22_Duty=%d,PWM23_Duty=%d,PWM24_Duty=%d,PWM25_Duty=%d,PWM26_Duty=%d,PWM27_Duty=%d,PWM28_Duty=%d,PWM29_Duty=%d,PWM30_Duty=%d,PWM31_Duty=%d,PWM32_Duty=%d",
                        servo_buf[0],servo_buf[1],servo_buf[2],servo_buf[3],servo_buf[4],servo_buf[5],servo_buf[6],servo_buf[7],\
                        servo_buf[8],servo_buf[9],servo_buf[10],servo_buf[11],servo_buf[12],servo_buf[13],servo_buf[14],servo_buf[15],\
                        servo_buf[16],servo_buf[17],servo_buf[18],servo_buf[19],servo_buf[20],servo_buf[21],servo_buf[22],servo_buf[23],\
                        servo_buf[24],servo_buf[25],servo_buf[26],servo_buf[27],servo_buf[28],servo_buf[29],servo_buf[30],servo_buf[31]);
      HAL_UART_Transmit(&huart2,Tx_Buf,strlen(Tx_Buf),15);
      HAL_Delay(500);
      
      
      #endif
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
