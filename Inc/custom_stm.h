/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : custom_stm.h
  * Description        : Header for custom_stm.c module.
  *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_STM_H
#define __CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* Environmental_Service_STM */
  CUSTOM_STM_TEMP,
  CUSTOM_STM_TEMPLATE,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* HW Service Chars related events */
  CUSTOM_MOTION_NOTIFY_ENABLED_EVT,
  CUSTOM_MOTION_NOTIFY_DISABLED_EVT,
  CUSTOM_ENV_NOTIFY_ENABLED_EVT,
  CUSTOM_ENV_NOTIFY_DISABLED_EVT,
  CUSTOM_ENV_READ_EVT,
  CUSTOM_ACC_EVENT_NOTIFY_ENABLED_EVT,
  CUSTOM_ACC_EVENT_NOTIFY_DISABLED_EVT,
  CUSTOM_ACC_EVENT_READ_EVT,
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t     Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
} Custom_STM_App_Notification_evt_t;

/* Exported constants --------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc( void );
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
//tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
tBleStatus Custom_STM_App_Update_Char(uint16_t UUID, uint8_t *pPayload);

#ifdef __cplusplus
}
#endif

#endif /*__CUSTOM_STM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
