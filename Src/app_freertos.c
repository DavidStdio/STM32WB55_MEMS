/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "i2c.h"
#include <ism330dlc_reg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
  /* Hardware_Service_STM */
  uint8_t               Motion_Notification_Status;
  uint8_t AccGyroMag_Update_Timer_Id;
  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

#define ACC_GYRO_MAG_UPDATE_PERIOD      (uint32_t)(0.05*1000*1000/CFG_TS_TICK_VAL) /*50ms (20Hz)*/
#define ACC_BYTES               (2)
#define GYRO_BYTES              (2)
#define MAG_BYTES               (2)

#define VALUE_LEN_MOTION        (2+3*ACC_BYTES+3*GYRO_BYTES+3*MAG_BYTES)
#define SENSOR_BUS hi2c1
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BOOT_TIME             15 //ms
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t whoamI, rst;
stmdev_ctx_t dev_ctx;
static volatile uint32_t txrxinproccess = 0;
/* USER CODE END Variables */
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Custom_Motion_Send_Notification(void);
static void MOTENV_AccGyroMagUpdate_Timer_Callback(void);
static void platform_delay(uint32_t ms);
/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{

  switch(pNotification->Custom_Evt_Opcode)
  {
  /* Hardware_Service_STM */

    case CUSTOM_STM_MOTION_READ_EVT:

      break;

    case CUSTOM_STM_MOTION_NOTIFY_ENABLED_EVT:
    	 Custom_App_Context.Motion_Notification_Status = 1;
    	 HW_TS_Stop(Custom_App_Context.AccGyroMag_Update_Timer_Id);
		 HW_TS_Start(Custom_App_Context.AccGyroMag_Update_Timer_Id, ACC_GYRO_MAG_UPDATE_PERIOD);
      break;

    case CUSTOM_STM_MOTION_NOTIFY_DISABLED_EVT:
    	 Custom_App_Context.Motion_Notification_Status  = 0;
    	 HW_TS_Stop(Custom_App_Context.AccGyroMag_Update_Timer_Id);
      break;

    default:

      break;
  }

  return;
}
/* USER CODE END FunctionPrototypes */

void StartImuTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
 	Custom_App_Context.Motion_Notification_Status = 0;
	/* Create timer to get the AccGyroMag params and update charecteristic */
	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
      &(Custom_App_Context.AccGyroMag_Update_Timer_Id),
      hw_ts_Repeated,
      MOTENV_AccGyroMagUpdate_Timer_Callback);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartImuTask */
/**
  * @brief  Function implementing the imuTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  whoamI = 0;

  while( whoamI != ISM330DLC_ID )
  {
	  ism330dlc_device_id_get(&dev_ctx, &whoamI);
  }

  /* Restore default configuration */
  ism330dlc_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    ism330dlc_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  ism330dlc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate */
  ism330dlc_gy_data_rate_set(&dev_ctx, ISM330DLC_GY_ODR_12Hz5);

  /* Set full scale */
  ism330dlc_gy_full_scale_set(&dev_ctx, ISM330DLC_2000dps);

  /* Gyroscope - filtering chain */
  ism330dlc_gy_band_pass_set(&dev_ctx, ISM330DLC_HP_260mHz_LP1_STRONG);

  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever);
	Custom_Motion_Send_Notification();
  }
  /* USER CODE END StartImuTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void MOTENV_AccGyroMagUpdate_Timer_Callback(void)
{
  osThreadFlagsSet( imuTaskHandle, 1 );
}

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

static axis3bit16_t data_raw_angular_rate;
extern stmdev_ctx_t dev_ctx;

void Custom_Motion_Send_Notification(void) // Property Notification
{
  uint8_t value[VALUE_LEN_MOTION];

  memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
  ism330dlc_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

  ///* Timestamp */
  STORE_LE_16(value, (HAL_GetTick()>>3));


  STORE_LE_16(value+8, data_raw_angular_rate.i16bit[0]);
  STORE_LE_16(value+10, data_raw_angular_rate.i16bit[1]);
  STORE_LE_16(value+12, data_raw_angular_rate.i16bit[2]);

  if(Custom_App_Context.Motion_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MOTION, (uint8_t *)value);
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

void i2c_cmd_resp_release(uint32_t flag)
{
  //UTIL_SEQ_SetEvt( 1<< CFG_IDLEEVT_I2C_CMD_EVT_RSP_ID  );
  txrxinproccess = 0;
  return;
}

void i2c_cmd_resp_wait(uint32_t timeout)
{
  //UTIL_SEQ_WaitEvt( 1<< CFG_IDLEEVT_I2C_CMD_EVT_RSP_ID );
	txrxinproccess = 1;
	while (txrxinproccess);

	return;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_cmd_resp_release(0);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_cmd_resp_release(0);
}

int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write_IT(handle, ISM330DLC_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len);

    i2c_cmd_resp_wait(33000);
  }

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Read_IT(handle, ISM330DLC_I2C_ADD_H, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len);

    i2c_cmd_resp_wait(33000);
  }

  return 0;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
