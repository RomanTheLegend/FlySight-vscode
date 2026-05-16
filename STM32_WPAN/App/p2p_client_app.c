/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    p2p_client_app.c
  * @author  MCD Application Team
  * @brief   peer to peer Client Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
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
#include "app_common.h"

#include "dbg_trace.h"

#include "ble.h"
#include "p2p_client_app.h"

#include "stm32_seq.h"
#include "app_ble.h"

/* USER CODE BEGIN Includes */
#include "gnss.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

typedef enum
{
  P2P_START_TIMER_EVT,
  P2P_STOP_TIMER_EVT,
  P2P_NOTIFICATION_INFO_RECEIVED_EVT,
} P2P_Client_Opcode_Notification_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t     Length;
}P2P_Client_Data_t;

typedef struct
{
  P2P_Client_Opcode_Notification_evt_t  P2P_Client_Evt_Opcode;
  P2P_Client_Data_t DataTransfered;
}P2P_Client_App_Notification_evt_t;

typedef struct
{
  /**
   * state of the P2P Client
   * state machine
   */
  APP_BLE_ConnStatus_t state;

  /**
   * connection handle
   */
  uint16_t connHandle;

  /**
   * handle of the P2P service
   */
  uint16_t P2PServiceHandle;

  /**
   * end handle of the P2P service
   */
  uint16_t P2PServiceEndHandle;

  /**
   * handle of the Tx characteristic - Write To Server
   *
   */
  uint16_t P2PWriteToServerCharHdle;

  /**
   * handle of the client configuration
   * descriptor of Tx characteristic
   */
  uint16_t P2PWriteToServerDescHandle;

  /**
   * handle of the Rx characteristic - Notification From Server
   *
   */
  uint16_t P2PNotificationCharHdle;

  /**
   * handle of the client configuration
   * descriptor of Rx characteristic
   */
  uint16_t P2PNotificationDescHandle;

}P2P_ClientContext_t;

/* USER CODE BEGIN PTD */
typedef struct{
  uint8_t                                     Device_Led_Selection;
  uint8_t                                     Led1;
}P2P_LedCharValue_t;

typedef struct{
  uint8_t                                     Device_Button_Selection;
  uint8_t                                     Button1;
}P2P_ButtonCharValue_t;

typedef struct
{

  uint8_t       Notification_Status; /* used to check if P2P Server is enabled to Notify */

  P2P_LedCharValue_t         LedControl;
  P2P_ButtonCharValue_t      ButtonStatus;

  uint16_t ConnectionHandle;


} P2P_Client_App_Context_t;

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))
/* USER CODE BEGIN PM */

//#define SERVICE_UUID "0783B03E-8535-B5A0-7140-A304D2495CB7"
//#define CHARACTERISTIC_UUID "0783B03E-8535-B5A0-7140-A304D2495CBA"

#define UUID_128BIT_FORMAT 1
#define SERVICE_UUID (0xB03E)
#define RX_CHAR_UUID (0x5CBA)

//#define UUID_128BIT_FORMAT 0
//#define SERVICE_UUID (0x1111)
//#define RX_CHAR_UUID (0xABCD)
#define FRAME_HEADER 0xFF
#define FRAME_FOOTER 0xAA
#define FRAME_FMT_LEN_2BYTES 0x10
#define FRAME_W 304
#define FRAME_H 256



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static P2P_ClientContext_t aP2PClientContext[BLE_CFG_CLT_MAX_NBR_CB];

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE BEGIN PV */
static P2P_Client_App_Context_t P2P_Client_App_Context;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void Gatt_Notification(P2P_Client_App_Notification_evt_t *pNotification);
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event);
/* USER CODE BEGIN PFP */
static tBleStatus Write_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload, size_t payloadLength);
static void Timer_Trigger_Received( void );
static void Ble_State_Monitoring_Service( void );

uint8_t prev_value = 0;
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void P2PC_APP_Init(void)
{
  uint8_t index =0;
/* USER CODE BEGIN P2PC_APP_Init_1 */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SEARCH_SERVICE_ID, UTIL_SEQ_RFU, Ble_State_Monitoring_Service );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_ACTIVELOOK_DISPLAY_UPDATE_ID, UTIL_SEQ_RFU, Timer_Trigger_Received );

  /**
   * Initialize LedButton Service
   */
  P2P_Client_App_Context.Notification_Status=0;
  P2P_Client_App_Context.ConnectionHandle =  0x00;

  P2P_Client_App_Context.LedControl.Device_Led_Selection=0x00;/* device Led */
  P2P_Client_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Client_App_Context.ButtonStatus.Device_Button_Selection=0x01;/* Device1 */
  P2P_Client_App_Context.ButtonStatus.Button1=0x00;
/* USER CODE END P2PC_APP_Init_1 */
  for(index = 0; index < BLE_CFG_CLT_MAX_NBR_CB; index++)
  {
    aP2PClientContext[index].state= APP_BLE_IDLE;
  }

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterCltHandler(Event_Handler);

#if(CFG_DEBUG_APP_TRACE != 0)
  APP_DBG_MSG("-- P2P CLIENT INITIALIZED \n\r");
#endif

/* USER CODE BEGIN P2PC_APP_Init_2 */

/* USER CODE END P2PC_APP_Init_2 */
  return;
}

void P2PC_APP_Notification(P2PC_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PC_APP_Notification_1 */

/* USER CODE END P2PC_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2P_Evt_Opcode */

/* USER CODE END P2P_Evt_Opcode */

  	case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
    P2P_Client_App_Context.ConnectionHandle = pNotification->ConnectionHandle;
/* USER CODE END PEER_CONN_HANDLE_EVT */
      break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
      {
      uint8_t index = 0;
      P2P_Client_App_Context.ConnectionHandle =  0x00;
      while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].state != APP_BLE_IDLE))
      {
        aP2PClientContext[index].state = APP_BLE_IDLE;
      }
//      BSP_LED_Off(LED_BLUE);

#if OOB_DEMO == 0
      UTIL_SEQ_SetTask(1<<CFG_TASK_CONN_DEV_1_ID, CFG_SCH_PRIO_0);
#endif
      }
/* USER CODE END PEER_DISCON_HANDLE_EVT */
      break;

    default:
/* USER CODE BEGIN P2P_Evt_Opcode_Default */

/* USER CODE END P2P_Evt_Opcode_Default */
      break;
  }
/* USER CODE BEGIN P2PC_APP_Notification_2 */

/* USER CODE END P2PC_APP_Notification_2 */
  return;
}
/* USER CODE BEGIN FD */
//void P2PC_APP_SW1_Button_Action(void)
//{
//
//  UTIL_SEQ_SetTask(1<<CFG_TASK_ACTIVELOOK_DISPLAY_UPDATE_ID, CFG_SCH_PRIO_0);
//
//}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;

  P2P_Client_App_Notification_evt_t Notification;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

//  APP_DBG_MSG("Event_Handler 0x%xU \n\r", event_pckt->evt);

  switch(event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
    {
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
//      APP_DBG_MSG("-- HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE 0x%xU \n\r", blecore_evt->ecode);
      switch(blecore_evt->ecode)
      {

        case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
        {
          aci_att_read_by_group_type_resp_event_rp0 *pr = (void*)blecore_evt->data;
          uint8_t numServ, i, idx;
          uint16_t uuid, handle;

          uint8_t index;
          handle = pr->Connection_Handle;
          index = 0;

//          APP_DBG_MSG("-- ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE (state: 0x%xU )\n\r", aP2PClientContext[index].state);

          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].state != APP_BLE_IDLE))
          {
            APP_BLE_ConnStatus_t status;

//            status = APP_BLE_Get_Client_Connection_Status(aP2PClientContext[index].connHandle);

            if((aP2PClientContext[index].state == APP_BLE_CONNECTED_CLIENT))
            {
              /* Handle deconnected */

              aP2PClientContext[index].state = APP_BLE_IDLE;
              aP2PClientContext[index].connHandle = 0xFFFF;
              break;
            }
            index++;
          }

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {
            aP2PClientContext[index].connHandle= handle;

            numServ = (pr->Data_Length) / pr->Attribute_Data_Length;

            /* the event data will be
             * 2bytes start handle
             * 2bytes end handle
             * 2 or 16 bytes data
             * we are interested only if the UUID is 16 bit.
             * So check if the data length is 6
             */
#if (UUID_128BIT_FORMAT==1)
          if (pr->Attribute_Data_Length == 20)
          {
            idx = 16;
#else
          if (pr->Attribute_Data_Length == 6)
          {
            idx = 4;
#endif
              for (i=0; i<numServ; i++)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);

                APP_DBG_MSG("-- GATT : Checking SERVICE_UUID 0x%x \n\r", uuid);
                if(uuid == SERVICE_UUID)
//				if(uuid == 0x999999)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : P2P_SERVICE_UUID 0x%x FOUND - connection handle 0x%x \n\r", SERVICE_UUID ,aP2PClientContext[index].connHandle);
#endif
#if (UUID_128BIT_FORMAT==1)
                aP2PClientContext[index].P2PServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-16]);
                aP2PClientContext[index].P2PServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-14]);
#else
                aP2PClientContext[index].P2PServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-4]);
                aP2PClientContext[index].P2PServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-2]);
#endif
                  aP2PClientContext[index].state = APP_BLE_DISCOVER_CHARACS ;
                }
                idx += 6;
              }
            }
          }
        }
        break;

        case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
        {

          aci_att_read_by_type_resp_event_rp0 *pr = (void*)blecore_evt->data;
          uint8_t idx;
          uint16_t uuid, handle;

          /* the event data will be
           * 2 bytes start handle
           * 1 byte char properties
           * 2 bytes handle
           * 2 or 16 bytes data
           */

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            /* we are interested in only 16 bit UUIDs */
#if (UUID_128BIT_FORMAT==1)
            idx = 17;
            if (pr->Handle_Value_Pair_Length == 21)
#else
		    idx = 5;
            if (pr->Handle_Value_Pair_Length == 7)
#endif
            {
              pr->Data_Length -= 1;
              while(pr->Data_Length > 0)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                /* store the characteristic handle not the attribute handle */
#if (UUID_128BIT_FORMAT==1)
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-2]);
#else
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-2]);
#endif
//                APP_DBG_MSG("---- GATT : Checking CHAR_UUID 0x%x \n\r", uuid);








                if(uuid == RX_CHAR_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
//                  APP_DBG_MSG("-- GATT : WRITE_UUID FOUND - connection handle 0x%x\n\r", aP2PClientContext[index].connHandle);
#endif
                  aP2PClientContext[index].state = APP_BLE_DISCOVER_WRITE_DESC;
                  aP2PClientContext[index].P2PWriteToServerCharHdle = handle;
                  APP_DBG_MSG("-- GATT : WRITE_UUID FOUND - connection handle 0x%x service handle 0x%x \n\r", aP2PClientContext[index].connHandle, aP2PClientContext[index].P2PWriteToServerCharHdle);
                }







//                else if(uuid == P2P_NOTIFY_CHAR_UUID)
//                {
//#if(CFG_DEBUG_APP_TRACE != 0)
//                  APP_DBG_MSG("-- GATT : NOTIFICATION_CHAR_UUID FOUND  - connection handle 0x%x\n\r", aP2PClientContext[index].connHandle);
//#endif
//                  aP2PClientContext[index].state = APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC;
//                  aP2PClientContext[index].P2PNotificationCharHdle = handle;
//                }
#if (UUID_128BIT_FORMAT==1)
                pr->Data_Length -= 21;
                idx += 21;
#else
                pr->Data_Length -= 7;
                idx += 7;
#endif
              }
            }
          }
        }
        break;

        case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
        {
          aci_att_find_info_resp_event_rp0 *pr = (void*)blecore_evt->data;

          uint8_t numDesc, idx, i;
          uint16_t uuid, handle;

          /*
           * event data will be of the format
           * 2 bytes handle
           * 2 bytes UUID
           */

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))

            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            numDesc = (pr->Event_Data_Length) / 4;
            /* we are interested only in 16 bit UUIDs */
            idx = 0;
            if (pr->Format == UUID_TYPE_16)
            {
              for (i=0; i<numDesc; i++)
              {
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx]);
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx+2]);

                if(uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID- connection handle 0x%x\n\r", aP2PClientContext[index].connHandle);
#endif
                  if( aP2PClientContext[index].state == APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC)
                  {

                    aP2PClientContext[index].P2PNotificationDescHandle = handle;
                    aP2PClientContext[index].state = APP_BLE_ENABLE_NOTIFICATION_DESC;

                  }
                }
                idx += 4;
              }
            }
          }
        }
        break; /*ACI_ATT_FIND_INFO_RESP_VSEVT_CODE*/

        case ACI_GATT_NOTIFICATION_VSEVT_CODE:
        {
          aci_gatt_notification_event_rp0 *pr = (void*)blecore_evt->data;
          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            if ( (pr->Attribute_Handle == aP2PClientContext[index].P2PNotificationCharHdle) &&
                    (pr->Attribute_Value_Length == (2)) )
            {

              Notification.P2P_Client_Evt_Opcode = P2P_NOTIFICATION_INFO_RECEIVED_EVT;
              Notification.DataTransfered.Length = pr->Attribute_Value_Length;
              Notification.DataTransfered.pPayload = &pr->Attribute_Value[0];

              Gatt_Notification(&Notification);

              /* INFORM APPLICATION BUTTON IS PUSHED BY END DEVICE */

            }
          }
        }
        break;/* end ACI_GATT_NOTIFICATION_VSEVT_CODE */

        case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
        {
          aci_gatt_proc_complete_event_rp0 *pr = (void*)blecore_evt->data;
#if(CFG_DEBUG_APP_TRACE != 0)
          APP_DBG_MSG("-- GATT : ACI_GATT_PROC_COMPLETE_VSEVT_CODE \n\r");
          APP_DBG_MSG("\n\r");
#endif

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            UTIL_SEQ_SetTask( 1<<CFG_TASK_SEARCH_SERVICE_ID, CFG_SCH_PRIO_0);

          }
        }
        break; /*ACI_GATT_PROC_COMPLETE_VSEVT_CODE*/
        default:
          break;
      }
    }

    break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

    default:
      break;
  }

  return(return_value);
}/* end BLE_CTRL_Event_Acknowledged_Status_t */

void Gatt_Notification(P2P_Client_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN Gatt_Notification_1*/

/* USER CODE END Gatt_Notification_1 */
  switch(pNotification->P2P_Client_Evt_Opcode)
  {
/* USER CODE BEGIN P2P_Client_Evt_Opcode */

/* USER CODE END P2P_Client_Evt_Opcode */

    case P2P_NOTIFICATION_INFO_RECEIVED_EVT:
/* USER CODE BEGIN P2P_NOTIFICATION_INFO_RECEIVED_EVT */
    {
      P2P_Client_App_Context.LedControl.Device_Led_Selection=pNotification->DataTransfered.pPayload[0];
      switch(P2P_Client_App_Context.LedControl.Device_Led_Selection) {

        case 0x01 : {

          P2P_Client_App_Context.LedControl.Led1=pNotification->DataTransfered.pPayload[1];

          if(P2P_Client_App_Context.LedControl.Led1==0x00){
//            BSP_LED_Off(LED_BLUE);
            APP_DBG_MSG(" -- P2P APPLICATION CLIENT : NOTIFICATION RECEIVED - LED OFF \n\r");
            APP_DBG_MSG(" \n\r");
          } else {
            APP_DBG_MSG(" -- P2P APPLICATION CLIENT : NOTIFICATION RECEIVED - LED ON\n\r");
            APP_DBG_MSG(" \n\r");
//            BSP_LED_On(LED_BLUE);
          }

          break;
        }
        default : break;
      }

    }
/* USER CODE END P2P_NOTIFICATION_INFO_RECEIVED_EVT */
      break;

    default:
/* USER CODE BEGIN P2P_Client_Evt_Opcode_Default */

/* USER CODE END P2P_Client_Evt_Opcode_Default */
      break;
  }
/* USER CODE BEGIN Gatt_Notification_2*/

/* USER CODE END Gatt_Notification_2 */
  return;
}

uint8_t P2P_Client_APP_Get_State( void ) {
  return aP2PClientContext[0].state;
}
/* USER CODE BEGIN LF */
/**
 * @brief  Feature Characteristic update
 * @param  pFeatureValue: The address of the new value to be written
 * @retval None
 */
tBleStatus Write_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload, size_t payloadLength)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t index;

  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aP2PClientContext[index].state != APP_BLE_IDLE))

  {

	  if(aP2PClientContext[index].state == APP_BLE_CONNECTED_CLIENT){

	  switch(UUID)
		{
		  case RX_CHAR_UUID: /* SERVER RX -- so CLIENT TX */



//			APP_DBG_MSG("-- GATT : Write_Char - char connection handle 0x%x service handle 0x%x\n\r", aP2PClientContext[index].connHandle, aP2PClientContext[index].P2PWriteToServerCharHdle);

			tBleStatus result;
	////		char pPayload[] = {0xFF, 0x01, 0x00, 0x05, 0xAA};
	//    	uint8_t pPayload[] = {0xFF, 0x01, 0x00, 0x05, 0xAA};
//			uint8_t payloadLength = strlen(pPayload);
			result = aci_gatt_write_without_resp(aP2PClientContext[index].connHandle,
											 aP2PClientContext[index].P2PWriteToServerCharHdle,
											 payloadLength,
											 pPayload);
	//
	//



	//        uint8_t pPayload[5] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5};  // 5 hex characters

	//        P2P_Client_App_Context.ButtonStatus.Button1 = 0x01;

	//        uint8_t *pPayload = (uint8_t *)&P2P_Client_App_Context.ButtonStatus;


	//        result = aci_gatt_write_without_resp(aP2PClientContext[index].connHandle,
	//                							 aP2PClientContext[index].P2PWriteToServerCharHdle,
	//											 sizeof(pPayload),
	//											 (uint8_t *)  pPayload);

//			if (result == BLE_STATUS_SUCCESS)
//			{
//				APP_DBG_MSG("Hex characters sent successfully\n");
//			}
//			else
//			{
//				APP_DBG_MSG("Failed to send hex characters. Error code: 0x%x\n", result);
//			}
			break;
		  default:
			break;
		}
    }
    index++;
  }

  return ret;
}/* end Write_Char() */


char* uint16_to_str(uint16_t value) {
    static char str[6];  // Max 5 digits + null terminator
    char* ptr = str + 5;
    *ptr = '\0';

    do {
        *--ptr = '0' + (value % 10);
        value /= 10;
    } while (value > 0);

    return ptr;
}


uint8_t* uShortToList(short value)
{
    uint8_t* bt = (uint8_t*)malloc(2 * sizeof(uint8_t));
    if (bt == NULL) {
        // Handle memory allocation failure
        return NULL;
    }
    bt[0] = (value >> 8) & 0xFF;
    bt[1] = value & 0xFF;
    return bt;
}


uint8_t* strToList(const char* str, int maxLen)
{
    size_t len = strlen(str);
    uint8_t* lst = (uint8_t*)malloc(len + 1);

    if (lst == NULL) {
        // Handle memory allocation failure
        return NULL;
    }

    memcpy(lst, str, len);

    if (maxLen == -1 || len < (size_t)maxLen)
    {
        lst[len] = 0;
    }

    return lst;
}

uint8_t* formatFrame(uint8_t cmdId, uint8_t* payload, size_t payloadSize,
                     uint8_t* queryId, size_t querySize, size_t* outSize)
{
    size_t frameLen = 5 + payloadSize + querySize; // 5 = header + cmdId + fmt + len + footer
    int lenNbByte = 1;
    if (frameLen > 0xFF)
    {
        frameLen += 1;
        lenNbByte = 2;
    }

    uint8_t* frame = (uint8_t*)malloc(frameLen);
    if (frame == NULL) {
        *outSize = 0;
        return NULL;
    }

    size_t idx = 0;
    frame[idx++] = FRAME_HEADER; // header
    frame[idx++] = cmdId; // cmdId

    if (lenNbByte == 1)
    {
        uint8_t fmt = querySize;
        frame[idx++] = fmt; // fmt
        frame[idx++] = (uint8_t)frameLen; // len
    }
    else
    {
        uint8_t fmt = FRAME_FMT_LEN_2BYTES | querySize;
        frame[idx++] = fmt; // fmt
        uint8_t* lenBytes = uShortToList((short)frameLen);
        frame[idx++] = lenBytes[0]; // len MSB
        frame[idx++] = lenBytes[1]; // len LSB
        free(lenBytes);
    }

    if (querySize > 0)
    {
        memcpy(frame + idx, queryId, querySize);
        idx += querySize;
    }

    memcpy(frame + idx, payload, payloadSize);
    idx += payloadSize;

    frame[idx++] = FRAME_FOOTER; // footer

    *outSize = frameLen;
    return frame;
}


void text(int x0, int y0, int rot, int font, int color, const char* str, uint8_t* payload)
{
    uint8_t* x0Bytes = uShortToList(x0);
    uint8_t* y0Bytes = uShortToList(y0);

    payload[0] = x0Bytes[0];
    payload[1] = x0Bytes[1];
    payload[2] = y0Bytes[0];
    payload[3] = y0Bytes[1];
    payload[4] = rot;
    payload[5] = font;
    payload[6] = color;

    uint8_t* strBytes = strToList(str, -1);
    size_t strLen = strlen(str);
    memcpy(payload + 7, strBytes, strLen);

    free(x0Bytes);
    free(y0Bytes);
    free(strBytes);
}



void Timer_Trigger_Received(void)
{
//  APP_DBG_MSG("-- P2P APPLICATION CLIENT  : BUTTON PUSHED - WRITE TO SERVER \n ");
//  APP_DBG_MSG(" \n\r");
//  if(P2P_Client_App_Context.ButtonStatus.Button1 == 0x00)
//  {
//    P2P_Client_App_Context.ButtonStatus.Button1 = 0x01;
//  }else {
//    P2P_Client_App_Context.ButtonStatus.Button1 = 0x00;
//  }




	 int i = 0;
	 bool rescan = false;
	 tBleStatus ret;
	 uint8_t  	connStatus[8];

	 while(i < BLE_CFG_CLT_MAX_NBR_CB){
    ret = aci_hal_get_link_status(connStatus, &aP2PClientContext[i].connHandle);

		APP_DBG_MSG("-- GATT : [STATUS] %d - Conn: 0x%x  State: %d Status:%d%d%d%d%d%d%d%d\n\r", i,
				aP2PClientContext[i].connHandle,aP2PClientContext[i].state ,
				connStatus[0],
				connStatus[1],
				connStatus[2],
				connStatus[3],
				connStatus[4],
				connStatus[5],
				connStatus[6],
				connStatus[7]
						   );
		i++;

	 }

	 int index = 0;
	  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
	          (aP2PClientContext[index].state != APP_BLE_IDLE))

	  {

		  if(aP2PClientContext[index].state == APP_BLE_CONNECTED_CLIENT && connStatus[1] == 0){

			  aP2PClientContext[index].state = APP_BLE_IDLE;
			  aci_gap_terminate(aP2PClientContext[i].connHandle, 0x02U);

			  rescan = true;

		  }
	    index++;
	  }


	  if (rescan){
			UTIL_SEQ_SetTask(1<<CFG_TASK_CONN_DEV_1_ID, CFG_SCH_PRIO_0);
			rescan = false;
	  }

	FS_GNSS_Data_t current;
//
//	// Copy to local variable
	memcpy(&current, FS_GNSS_GetData(), sizeof(FS_GNSS_Data_t));

	uint16_t h_speed = 0;
	uint8_t x = 80, y = 50, size = 6, color = 15;



	h_speed = current.gSpeed * 0.036;

	if (h_speed != prev_value){

	char* data_old = uint16_to_str(prev_value);
	char* data_new = uint16_to_str(h_speed);

    uint8_t payload_old[strlen(data_old) - 1 + 8];
    uint8_t payload_new[strlen(data_new) - 1 + 8];

    uint8_t queryId[1];
    size_t frameLen_old, frameLen_new;

    text(FRAME_W - x, FRAME_H - y, 4, size, 0    , data_old, payload_old);
    text(FRAME_W - x, FRAME_H - y, 4, size, color, data_new, payload_new);

    uint8_t* command_old = formatFrame(0x37, payload_old, 7 + strlen(data_old), queryId, 0, &frameLen_old);
    uint8_t* command_new = formatFrame(0x37, payload_new, 7 + strlen(data_new), queryId, 0, &frameLen_new);

   //= {0xFF, 0x01, 0x00, 0x05, 0xAA};

    Write_Char(RX_CHAR_UUID, 0, command_old, frameLen_old);
    Write_Char(RX_CHAR_UUID, 0, command_new, frameLen_new);

    prev_value = h_speed;
	}

  return;
}

void Ble_State_Monitoring_Service()
{
  uint16_t enable = 0x0001;
  uint16_t disable = 0x0000;
  uint8_t index;



  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aP2PClientContext[index].state != APP_BLE_IDLE))
  {


	switch(aP2PClientContext[index].state)
    {
      case APP_BLE_DISCOVER_SERVICES:
        APP_DBG_MSG("P2P_DISCOVER_SERVICES\n\r");
        break;
      case APP_BLE_DISCOVER_CHARACS:
        APP_DBG_MSG("* GATT : Discover P2P Characteristics\n\r");
        aci_gatt_disc_all_char_of_service(aP2PClientContext[index].connHandle,
                                          aP2PClientContext[index].P2PServiceHandle,
                                          aP2PClientContext[index].P2PServiceEndHandle);
        break;
      case APP_BLE_DISCOVER_WRITE_DESC:
        APP_DBG_MSG("* GATT : Discover Descriptor of TX - Write  Characteristic\n\r");
        aci_gatt_disc_all_char_desc(aP2PClientContext[index].connHandle,
                                    aP2PClientContext[index].P2PWriteToServerCharHdle,
                                    aP2PClientContext[index].P2PWriteToServerCharHdle+2);

//
//
        aP2PClientContext[index].state = APP_BLE_CONNECTED_CLIENT;

        //set configuration to "flysight"
        uint8_t set_alook_config[] = {0xff, 0xd2, 0x00, 0x0e, 0x66, 0x6c, 0x79, 0x73, 0x69, 0x67, 0x68, 0x74, 0x00, 0xaa};
        Write_Char( RX_CHAR_UUID, 0, set_alook_config, 14);

        //clear screen
        uint8_t clear_screen[] = {0xFF, 0x01, 0x00, 0x05, 0xAA};
        Write_Char( RX_CHAR_UUID, 0, clear_screen, 5);
        break;
      case APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC:
        APP_DBG_MSG("* GATT : Discover Descriptor of Rx - Notification  Characteristic\n\r");
        aci_gatt_disc_all_char_desc(aP2PClientContext[index].connHandle,
                                    aP2PClientContext[index].P2PNotificationCharHdle,
                                    aP2PClientContext[index].P2PNotificationCharHdle+2);
        break;
      case APP_BLE_ENABLE_NOTIFICATION_DESC:
        APP_DBG_MSG("* GATT : Enable Server Notification\n\r");
        aci_gatt_write_char_desc(aP2PClientContext[index].connHandle,
                                 aP2PClientContext[index].P2PNotificationDescHandle,
                                 2,
                                 (uint8_t *)&enable);
        aP2PClientContext[index].state = APP_BLE_CONNECTED_CLIENT;
//        BSP_LED_Off(LED_RED);
        break;
      case APP_BLE_DISABLE_NOTIFICATION_DESC :
        APP_DBG_MSG("* GATT : Disable Server Notification\n\r");
        aci_gatt_write_char_desc(aP2PClientContext[index].connHandle,
                                 aP2PClientContext[index].P2PNotificationDescHandle,
                                 2,
                                 (uint8_t *)&disable);
        aP2PClientContext[index].state = APP_BLE_CONNECTED_CLIENT;
        break;
      default:
        break;
    }
    index++;
  }
  return;
}
/* USER CODE END LF */
