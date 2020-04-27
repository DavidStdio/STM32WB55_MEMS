/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : custom_stm.c
  * Description        : Custom Example Service.
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

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "custom_stm.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomEnvironmentalsrvcHdle;                   /**< Environmental_Service_STM handle */
  uint16_t  CustomTempHdle;                   /**< Temperature handle */
  uint16_t  CustomTemplateHdle;                   /**< Template handle */
}CustomContext_t;

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const uint8_t SizeTemp=4;
static const uint8_t SizeTemplate=1;
/**
 * START of Section BLE_DRIVER_CONTEXT
 */
PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */
/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
/*
 The following 128bits UUIDs have been generated from the random UUID
 generator:
 D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
 D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
 D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
 */
#define COPY_ENVIRONMENTAL_SERVICE_STM_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xE1,0x9A,0xB4,0x00,0x02,0xA5,0xD5,0xC5,0x1B)
#define COPY_TEMPERATURE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x04,0x00,0x00,0x00,0x01,0x11,0xE1,0xAC,0x36,0x00,0x02,0xA5,0xD5,0xC5,0x1B)
#define COPY_TEMPLATE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0xAA,0xCC,0x8E,0x22,0x45,0x41,0x9D,0x4c,0x21,0xED,0xAE,0x82,0xED,0x19)

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;
  aci_gatt_attribute_modified_event_rp0    * attribute_modified;
  aci_gatt_read_permit_req_event_rp0       * read_req;
  Custom_STM_App_Notification_evt_t Notification;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case EVT_VENDOR:
      blue_evt = (evt_blue_aci*)event_pckt->data;
      switch(blue_evt->ecode)
      {

        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED */
        {
            attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blue_evt->data;
              if(attribute_modified->Attr_Handle == (CustomContext.CustomTempHdle + 2))
              {
                /**
                 * Descriptor handle
                 */
                return_value = SVCCTL_EvtAckFlowEnable;
                /**
                 * Notify to application
                 */
                if(attribute_modified->Attr_Data[0] & COMSVC_Notification)
                {
                  Notification.Custom_Evt_Opcode = CUSTOM_STM_TEMP_NOTIFY_ENABLED_EVT;
                  Custom_STM_App_Notification(&Notification);
                }
                else
                {
                  Notification.Custom_Evt_Opcode = CUSTOM_STM_TEMP_NOTIFY_DISABLED_EVT;
				  Custom_STM_App_Notification(&Notification);
                }
              }

        }
          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED */
          break;
        case EVT_BLUE_GATT_READ_PERMIT_REQ :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ */
          break;
        case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ */
          break;

        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* EVT_VENDOR */

    /* USER CODE BEGIN EVENT_PCKT_CASES*/

    /* USER CODE END EVENT_PCKT_CASES*/

    default:
      break;
  }

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{
 
  Char_UUID_t  uuid;

  /**
   *	Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

    /*
     *          Environmental_Service_STM
     *
     * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property
     * service_max_attribute_record = 1 for Environmental_Service_STM +
     *                                2 for Temperature +
     *                                2 for Template +
     *                                1 for Temperature configuration descriptor +
     *                              = 6
     */

    COPY_ENVIRONMENTAL_SERVICE_STM_UUID(uuid.Char_UUID_128);
    aci_gatt_add_service(UUID_TYPE_128,
                      (Service_UUID_t *) &uuid,
                      PRIMARY_SERVICE,
                      6,
                      &(CustomContext.CustomEnvironmentalsrvcHdle));

    /**
     *  Temperature
     */
    COPY_TEMPERATURE_UUID(uuid.Char_UUID_128);
    aci_gatt_add_char(CustomContext.CustomEnvironmentalsrvcHdle,
                      UUID_TYPE_128, &uuid,
                      SizeTemp,
                      CHAR_PROP_NOTIFY,
                      ATTR_PERMISSION_NONE,
                      GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                      0x10,
                      CHAR_VALUE_LEN_VARIABLE,
                      &(CustomContext.CustomTempHdle));
    /**
     *  Template
     */
    COPY_TEMPLATE_UUID(uuid.Char_UUID_128);
    aci_gatt_add_char(CustomContext.CustomEnvironmentalsrvcHdle,
                      UUID_TYPE_128, &uuid,
                      SizeTemplate,
                      CHAR_PROP_WRITE,
                      ATTR_PERMISSION_NONE,
                      GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                      0x10,
                      CHAR_VALUE_LEN_CONSTANT,
                      &(CustomContext.CustomTemplateHdle));

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 * 
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload) 
{
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;

  switch(CharOpcode)
  {

    case CUSTOM_STM_TEMP:
      result = aci_gatt_update_char_value(CustomContext.CustomEnvironmentalsrvcHdle,
                            CustomContext.CustomTempHdle,
                            0, /* charValOffset */
                            SizeTemp, /* charValueLen */
                            (uint8_t *)  pPayload);
    /* USER CODE BEGIN CUSTOM_STM_TEMP*/

    /* USER CODE END CUSTOM_STM_TEMP*/
      break;

    case CUSTOM_STM_TEMPLATE:
      result = aci_gatt_update_char_value(CustomContext.CustomEnvironmentalsrvcHdle,
                            CustomContext.CustomTemplateHdle,
                            0, /* charValOffset */
                            SizeTemplate, /* charValueLen */
                            (uint8_t *)  pPayload);
    /* USER CODE BEGIN CUSTOM_STM_TEMPLATE*/

    /* USER CODE END CUSTOM_STM_TEMPLATE*/
      break;

    default:
      break;
  }

  return result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
