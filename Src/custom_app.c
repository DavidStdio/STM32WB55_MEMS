/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : custom_app.c
 * Description        : Custom Example Application (Server)
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
//#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ism330dlc_reg.h>

typedef struct
{
	uint16_t TimeStamp;
	uint16_t Value;
}Custom_TemperatureCharValue_t;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* Hardware_Service_STM */
  //uint8_t               Env_Notification_Status;
  uint8_t               Motion_Notification_Status;
/* USER CODE BEGIN CUSTOM_APP_Context_t */
  //Custom_TemperatureCharValue_t Temperature;
  //int16_t ChangeStep;
  //uint8_t Update_timer_Id;
  uint8_t AccGyroMag_Update_Timer_Id;
/* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TEMPERATURE_CHANGE_STEP    		  10
//#define TEMPERATURE_CHANGE_PERIOD  		  (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
//#define TEMPERATURE_VALUE_MAX_THRESHOLD   350
//#define TEMPERATURE_VALUE_MIN_THRESHOLD   100

#define ACC_GYRO_MAG_UPDATE_PERIOD      (uint32_t)(0.05*1000*1000/CFG_TS_TICK_VAL) /*50ms (20Hz)*/
#define ACC_BYTES               (2)
#define GYRO_BYTES              (2)
#define MAG_BYTES               (2)

#define VALUE_LEN_MOTION        (2+3*ACC_BYTES+3*GYRO_BYTES+3*MAG_BYTES)
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */
uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

uint8_t SecureReadData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
  /* Hardware_Service_STM */
static void Custom_Motion_Update_Char(void);
static void Custom_Motion_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void MOTENV_AccGyroMagUpdate_Timer_Callback(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
//void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
//{
//  switch(pNotification->Custom_Evt_Opcode)
//  {
//  /* Hardware_Service_STM */
//    case CUSTOM_STM_MOTION_READ_EVT:
//      break;
//
//    case CUSTOM_STM_MOTION_NOTIFY_ENABLED_EVT:
///* USER CODE BEGIN CUSTOM_STM_MOTION_NOTIFY_ENABLED_EVT */
//    	 //Custom_App_Context.Motion_Notification_Status = 1;
//		 //HW_TS_Start(Custom_App_Context.AccGyroMag_Update_Timer_Id, ACC_GYRO_MAG_UPDATE_PERIOD);
///* USER CODE END CUSTOM_STM_MOTION_NOTIFY_ENABLED_EVT */
//      break;
//
//    case CUSTOM_STM_MOTION_NOTIFY_DISABLED_EVT:
///* USER CODE BEGIN CUSTOM_STM_MOTION_NOTIFY_DISABLED_EVT */
//    	 //Custom_App_Context.Motion_Notification_Status  = 0;
//    	 //HW_TS_Stop(Custom_App_Context.AccGyroMag_Update_Timer_Id);
///* USER CODE END CUSTOM_STM_MOTION_NOTIFY_DISABLED_EVT */
//      break;
//
//    default:
//      break;
//  }
//
//  return;
//}

//void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
//{
///* USER CODE BEGIN CUSTOM_APP_Notification_1 */
//
///* USER CODE END CUSTOM_APP_Notification_1 */
//
//  switch(pNotification->Custom_Evt_Opcode)
//  {
//  case CUSTOM_CONN_HANDLE_EVT :
//    break;
//
//    case CUSTOM_DISCON_HANDLE_EVT :
//    break;
//
//    default:
//      break;
//  }
//
//  return;
//}

//void Custom_APP_Init(void)
//{
///* USER CODE BEGIN CUSTOM_APP_Init */
//   	Custom_App_Context.Motion_Notification_Status = 0;
//	//Custom_App_Context.Temperature.TimeStamp = 0;
//	//Custom_App_Context.Temperature.Value = 0;
//	//Custom_App_Context.ChangeStep = TEMPERATURE_CHANGE_STEP;
////	UTIL_SEQ_RegTask( 1<<CFG_TASK_NOTIFY_TEMPERATURE, UTIL_SEQ_RFU, Custom_Env_Send_Notification);
//
//	//HW_TS_Create(CFG_TIM_PROC_ID_ISR,
//	//		&(Custom_App_Context.Update_timer_Id),
//	//		hw_ts_Repeated,
//	//		Custom_TemperatureChange_Timer_Callback);
//
////	UTIL_SEQ_RegTask( 1<<CFG_TASK_NOTIFY_ACC_GYRO_MAG_ID, UTIL_SEQ_RFU, Custom_Motion_Send_Notification);
//	/* Create timer to get the AccGyroMag params and update charecteristic */
//	//HW_TS_Create(CFG_TIM_PROC_ID_ISR,
//    //    &(Custom_App_Context.AccGyroMag_Update_Timer_Id),
//    //    hw_ts_Repeated,
//    //    MOTENV_AccGyroMagUpdate_Timer_Callback);
//
///* USER CODE END CUSTOM_APP_Init */
//  return;
//}

/* USER CODE BEGIN FD */
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

  /* Hardware_Service_STM */
//void Custom_Env_Update_Char(void) //Property Read
//{
////  Custom_STM_App_Update_Char(CUSTOM_STM_ENV, (uint8_t *)UpdateCharData);
//  return;
//}

void Custom_Env_Send_Notification(void) // Property Notification
{
  //  NotifyCharData[0] = (uint8_t)(Custom_App_Context.Temperature.TimeStamp & 0x00FF);
  //  NotifyCharData[1] = (uint8_t)(Custom_App_Context.Temperature.TimeStamp >> 8);
  //  NotifyCharData[2] = (uint8_t)(Custom_App_Context.Temperature.Value & 0x00FF);
  //  NotifyCharData[3] = (uint8_t)(Custom_App_Context.Temperature.Value >> 8);
  //  NotifyCharData[4] = (uint8_t)(Custom_App_Context.Temperature.Value & 0x00FF);
  //  NotifyCharData[5] = (uint8_t)(Custom_App_Context.Temperature.Value >> 8);
  //  NotifyCharData[6] = (uint8_t)(Custom_App_Context.Temperature.Value & 0x00FF);
  //  NotifyCharData[7] = (uint8_t)(Custom_App_Context.Temperature.Value >> 8);
  //  NotifyCharData[8] = (uint8_t)(Custom_App_Context.Temperature.Value & 0x00FF);
  //  NotifyCharData[9] = (uint8_t)(Custom_App_Context.Temperature.Value >> 8);
  //  NotifyCharData[10] = (uint8_t)(Custom_App_Context.Temperature.Value & 0x00FF);
  //  NotifyCharData[11] = (uint8_t)(Custom_App_Context.Temperature.Value >> 8);

  //  Custom_App_Context.Temperature.Value += Custom_App_Context.ChangeStep;
  //  Custom_App_Context.Temperature.TimeStamp += TEMPERATURE_CHANGE_STEP;

  //  if(Custom_App_Context.Temperature.Value > TEMPERATURE_VALUE_MAX_THRESHOLD )
  //  {
  //                Custom_App_Context.ChangeStep = -TEMPERATURE_CHANGE_STEP;
  //  }

  //  if(Custom_App_Context.Temperature.Value == 0)

  //  {
  //                Custom_App_Context.ChangeStep = +TEMPERATURE_CHANGE_STEP;
  //  }

  //if(Custom_App_Context.Env_Notification_Status)
  //{
  //  Custom_STM_App_Update_Char(CUSTOM_STM_ENV, (uint8_t *)NotifyCharData);
  //}
  //else
  //{
  //  APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  //}
  return;
}

void Custom_Motion_Update_Char(void) //Property Read
{ 
  Custom_STM_App_Update_Char(CUSTOM_STM_MOTION, (uint8_t *)UpdateCharData);
  return;
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

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

static void MOTENV_AccGyroMagUpdate_Timer_Callback(void)
{
  //UTIL_SEQ_SetTask(1<<CFG_TASK_NOTIFY_ACC_GYRO_MAG_ID, CFG_SCH_PRIO_0);
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
