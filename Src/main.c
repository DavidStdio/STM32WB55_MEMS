/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "rf.h"
#include "rtc.h"
#include "app_entry.h"
#include "app_common.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32_seq.h"
#include <ism330dlc_reg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME             15 //ms

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
stmdev_ctx_t dev_ctx;

static void platform_delay(uint32_t ms);

int __io_putchar(int ch)
{
	ITM_SendChar(ch);
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
  MX_DMA_Init();
  MX_RF_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //UTIL_SEQ_Init();

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  //whoamI = 0;

  //while( whoamI != ISM330DLC_ID )
  //{
//	  ism330dlc_device_id_get(&dev_ctx, &whoamI);
  //}

  ///* Restore default configuration */
  //ism330dlc_reset_set(&dev_ctx, PROPERTY_ENABLE);
  //do {
  //  ism330dlc_reset_get(&dev_ctx, &rst);
  //} while (rst);

  ///* Enable Block Data Update */
  //ism330dlc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  ///* Set Output Data Rate */
  ///* davidbo
  //ism330dlc_xl_data_rate_set(&dev_ctx, ISM330DLC_XL_ODR_12Hz5);
  //*/
  //ism330dlc_gy_data_rate_set(&dev_ctx, ISM330DLC_GY_ODR_12Hz5);

  ///* Set full scale */
  ///* davidbo
  //ism330dlc_xl_full_scale_set(&dev_ctx, ISM330DLC_2g);
  //*/
  //ism330dlc_gy_full_scale_set(&dev_ctx, ISM330DLC_2000dps);

  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  /* davidbo
  ism330dlc_xl_filter_analog_set(&dev_ctx, ISM330DLC_XL_ANA_BW_400Hz);
  */

  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //ism330dlc_xl_lp1_bandwidth_set(&dev_ctx, ISM330DLC_XL_LP1_ODR_DIV_4);

  /* Accelerometer - LPF1 + LPF2 path */
  /* davidbo
  ism330dlc_xl_lp2_bandwidth_set(&dev_ctx, ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100);
  */

  /* Accelerometer - High Pass / Slope path */
  //ism330dlc_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  //ism330dlc_xl_hp_bandwidth_set(&dev_ctx, ISM330DLC_XL_HP_ODR_DIV_100);

  /* Gyroscope - filtering chain */
  //ism330dlc_gy_band_pass_set(&dev_ctx, ISM330DLC_HP_260mHz_LP1_STRONG);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 
  APPE_Init();
  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //ism330dlc_reg_t reg;
    //ism330dlc_status_reg_get(&dev_ctx, &reg.status_reg);

    //if (reg.status_reg.xlda)
    //{
    //  /* Read magnetic field data */
    //  memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
    //  ism330dlc_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
    //  acceleration_mg[0] = ism330dlc_from_fs2g_to_mg( data_raw_acceleration.i16bit[0]);
    //  acceleration_mg[1] = ism330dlc_from_fs2g_to_mg( data_raw_acceleration.i16bit[1]);
    //  acceleration_mg[2] = ism330dlc_from_fs2g_to_mg( data_raw_acceleration.i16bit[2]);

    //  //sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
    //   //       acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    //  //tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    //}

    //if (reg.status_reg.gda)
    {
      /* Read magnetic field data */
      //memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
      //ism330dlc_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      //angular_rate_mdps[0] = ism330dlc_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      //angular_rate_mdps[1] = ism330dlc_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      //angular_rate_mdps[2] = ism330dlc_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

     // sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
     //         angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
     // tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }

	  //UTIL_SEQ_Run(~0);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
