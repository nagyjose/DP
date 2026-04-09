/**
  ******************************************************************************
 * @file    app_entry.c
 * @author  MCD Application Team
 * @brief   Entry point of the Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "app_entry.h"

#include "app_ffd_mac_802_15_4.h"
#include "app_ble.h"
#include "app_conf.h"
#include "utilities_common.h"
#include "utilities_conf.h"
#include "otp.h"
#include "stm32_seq.h"
#include "stm_logging.h"
#include "stm32wbxx_ll_rcc.h"
#include "shci.h"
#include "shci_tl.h"
#include "stm32_lpm.h"
#include "tl_mac_802_15_4.h"
#include "tl.h"
#include "dbg_trace.h"
#include "stm_logging.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"


#define CFG_USB_INTERFACE_ENABLE  0  // 1 = Zapnuto, 0 = USB se vůbec nekompiluje
#define NBIOT_HARDWARE_CONNECTED 0  // 0 = modul chybí, 1 = modul je zapojen

#define HOST_SYS_EVTCODE                (0xFFU)
#define HOST_SYS_SUBEVTCODE_BASE        (0x9200U)
#define HOST_SYS_SUBEVTCODE_READY        (HOST_SYS_SUBEVTCODE_BASE + 0U)
#define POOL_SIZE (CFG_TL_EVT_QUEUE_LENGTH * 4U * DIVC(( sizeof(TL_PacketHeader_t) + TL_EVENT_FRAME_SIZE ), 4U))

// Globální instance USB
USBD_HandleTypeDef hUsbDeviceFS;

// --- USB Proměnné ---
uint8_t usb_rx_buffer[256];
uint16_t usb_rx_len = 0;

void USB_App_Init(void);


/* Section specific to button management using UART */
#if (NBIOT_HARDWARE_CONNECTED == 1)
static void RxUART_Init(void);
#endif

static void RxCpltCallback(void);
static void UartCmdExecute(void);

#define C_SIZE_CMD_STRING               256U
#define RX_BUFFER_SIZE          8U

#define SWITCH_TMO 1000*1000/CFG_TS_TICK_VAL/2   /* 500 ms */

static uint8_t aRxBuffer[RX_BUFFER_SIZE];
EXTI_HandleTypeDef exti_handle;

/* Error code */
#define ERR_INTERFACE_FATAL_1 1
#define ERR_INTERFACE_FATAL_2 2

extern void APP_FFD_MAC_802_15_4_SetupTask(void);
extern void APP_FFD_MAC_802_15_4_CoordSrvTask(void);
extern void APP_FFD_MAC_802_15_4_CoordDataTask(void);

extern void TestMode_CycleState(void);
extern void APP_MAC_Magnet_Action(void);
extern void System_Execute_Command(uint8_t *payload_data, uint8_t payload_len, uint8_t source);

extern void HW_USB_Clock_Init(void);

extern uint8_t g_srvSerReq;
extern uint8_t g_srvDataReq;
extern RTC_HandleTypeDef hrtc;

// Zpřístupnění bufferu, do kterého nám usbd_cdc_if.c sype data
extern uint8_t usb_rx_buffer[256];
extern uint16_t usb_rx_len;

#if (CFG_USB_INTERFACE_ENABLE != 0)
// Tuto funkci musíš zaregistrovat k tasku v APPE_Init:
// UTIL_SEQ_RegTask(1<<CFG_TASK_USB_PROCESS, UTIL_SEQ_RFU, APP_USB_ProcessTask);
void APP_USB_ProcessTask(void)
{
	APP_DBG(">>> Pokus o odeslani do USB (Delka: %d)", usb_rx_len);

	// Pokusíme se odeslat přijatá data rovnou zpět a uložíme si výsledek
	uint8_t result = CDC_Transmit_FS(usb_rx_buffer, usb_rx_len);

	// Necháme vypsat výsledek do konzole v IDE!
	if (result == USBD_BUSY) {
			APP_DBG("--- CHYBA: USB vraci BUSY! (Je to zablokovane)");
	}
	else if (result == USBD_FAIL) {
			APP_DBG("--- CHYBA: USB vraci FAIL! (Knihovna spadla)");
	}
	else if (result == USBD_OK) {
			APP_DBG("--- OK: Data narvana do HW. Pokud je nevidis, problem je v PC!");
	}

	usb_rx_len = 0;
	memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));


	/*

	// 1. OCHRANA STRINGU: Přidáme na konec dat neviditelnou nulu pro strcmp
	if (usb_rx_len < sizeof(usb_rx_buffer)) {
			usb_rx_buffer[usb_rx_len] = '\0';
	}

	// 2. OZVĚNA PRO TEBE: Pošleme text ZPĚT do tvého USB terminálu, abys ho viděl!
	char echo_msg[64];
	int len = snprintf(echo_msg, sizeof(echo_msg), "\r\n[USB] Prijato: %s\r\n", usb_rx_buffer);
	CDC_Transmit_FS((uint8_t*)echo_msg, len);

	APP_DBG(">>> USB RX: Prijato %d bajtu z PC", usb_rx_len);

	// Tady data jednoduše vezmeš a pošleš do svého existujícího parseru!
	// Přidáš si jen nové makro SOURCE_USB (např. hodnotu 3)
	System_Execute_Command(usb_rx_buffer, usb_rx_len, 2);

	// Vyčištění bufferu pro další příjem
	usb_rx_len = 0;
	memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));*/
}

static void Force_USB_Reenumeration(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 1. Zapnutí hodin pro port A */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* 2. Přepnutí pinu PA12 (USB DP) do běžného výstupu */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 3. Natvrdo stáhnout pin na GND na 10 milisekund */
  /* Počítač zaregistruje fyzické "odpojení" kabelu */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(10);

  /* Jakmile to proběhne, HAL_PCD_MspInit si piny za chvíli zase správně
     přepne do Alternate Function (USB) a počítač uslyší čisté připojení */
}
#endif

/* Global variables  -------------------------------------------------*/
char CommandString[C_SIZE_CMD_STRING];
__IO uint16_t indexReceiveChar = 0U;
__IO uint16_t remainingRxChar = 0U;
__IO uint16_t CptReceiveCmdFromUser = 0U;
__IO uint16_t remainSendChar = 0U;
__IO uint16_t CptSendCmdToUser = 0U;

/* Private function definition -------------------------------------------------*/

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_MAC_802_15_4_Config_t Mac_802_15_4_ConfigBuffer;

PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t Mac_802_15_4_CmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t Mac_802_15_4_NotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];

PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t EvtPool[POOL_SIZE];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t SystemCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t SystemSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];

PLACE_IN_SECTION("MB_MEM2") ALIGN(4) char mac_802_15_4_CnfIndNot[C_SIZE_CMD_STRING];

static SHCI_C2_CONCURRENT_Mode_Param_t ConcurrentMode = MAC_ENABLE;
static uint8_t TS_ID1;


/*----------------------------------------------------------------------------*/
static TL_CmdPacket_t *p_mac_802_15_4_cmdbuffer;
static TL_EvtPacket_t *p_mac_802_15_4_notif_RFCore_to_M4;
static int NbTotalSwitch = 0;
static int FlagSwitchingOnGoing = 0;
static __IO uint32_t  CptReceiveMsgFromRFCore = 0U; /* Debug counter */


/* Global function prototypes -----------------------------------------------*/
size_t DbgTraceWrite(int handle, const unsigned char * buf, size_t bufSize);

/* Private function prototypes -----------------------------------------------*/
static void appe_Tl_Init(void);
static void Led_Init(void);
static void Button_Init( void );
static void Wait_Getting_Ack_From_RFCore(void);
static void Receive_Ack_From_RFCore(void);
static void Receive_Notification_From_RFCore(void);
static void APPE_StatusNot(SHCI_TL_CmdStatus_t status);
static void Process_InitiateSwitchProtocol(void);
static void Process_ActivateNewProtocol(void);
static void ScheduleProcessSwitchProtocol(void);
static void SystemPower_Config( void );
static void Init_Debug(void);
static void APPE_SysUserEvtRx( void * pPayload );
static void APPE_SysEvtReadyProcessing( void);
static void APP_TraceError(char * pMess, uint32_t ErrCode);
static void APP_CheckWirelessFirmwareInfo(void);


/* Functions Definition ------------------------------------------------------*/
void APP_Init( void )
{
 SystemPower_Config(); /**< Configure the system Power Mode */
  
 HW_TS_Init(hw_ts_InitMode_Full, &hrtc); /**< Initialize the TimerServer */

  Init_Debug();

  /* Task common to MAC and BLE used when switching protocol */
  UTIL_SEQ_RegTask( 1U <<CFG_TASK_INIT_SWITCH_PROTOCOL, UTIL_SEQ_RFU,Process_InitiateSwitchProtocol);
  UTIL_SEQ_RegTask( 1U <<CFG_TASK_ACTIVATE_PROTOCOL, UTIL_SEQ_RFU,Process_ActivateNewProtocol);
  /**
   * The Standby mode should not be entered before the initialization is over
   * The default state of the Low Power Manager is to allow the Standby Mode so an request is needed here
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);

#if (CFG_USB_INTERFACE_ENABLE != 0)
  // Toto říká: "Když někdo vzbudí USB task, zavolej funkci APP_USB_ProcessTask"
  UTIL_SEQ_RegTask(1<<CFG_TASK_USB_PROCESS, UTIL_SEQ_RFU, APP_USB_ProcessTask);
  USB_App_Init();
#endif

  Led_Init();
  Button_Init();
  HW_UART_Init(CFG_CLI_UART);

#if (NBIOT_HARDWARE_CONNECTED == 1)
  RxUART_Init();
#else
  APP_DBG(">>> NBIOT UART bypass (Modul neni pripojen)");
#endif
  appe_Tl_Init(); /**< Initialize all transport layers */

  /**
   * From now, the application is waiting for the ready event ( VS_HCI_C2_Ready )
   * received on the system channel before starting the BLE or MAC Stack
   * This system event is received with APPE_UserEvtRx()
   */

  return;
}

/**
 * @brief  Process used to initiate the protocol switch
 * @param  None
 * @retval None
 */
static void Process_InitiateSwitchProtocol(void)
{
   APP_DBG("Process_InitiateSwitchProtocol (nb = %d)",NbTotalSwitch ++);

  /* Send Switch event to M0 */
  /* SWITCH BLE <-> MAC */
  if (FlagSwitchingOnGoing == 1)
      return;
  FlagSwitchingOnGoing = 1;

  if(ConcurrentMode == MAC_ENABLE)
  {
    APP_DBG("STOP MAC");
    APP_FFD_MAC_802_15_4_Stop();

    /* start a timer */
    HW_TS_Create(CFG_TIM_WAIT_BEFORE_SWITCH, &TS_ID1, hw_ts_SingleShot, ScheduleProcessSwitchProtocol);

    HW_TS_Start(TS_ID1, SWITCH_TMO);
    APP_DBG("SWITCH PROTOCOL TO BLE");
  }
  else
  {
    APP_DBG("STOP BLE");
    APP_BLE_Stop();
    APP_DBG("SWITCH PROTOCOL TO MAC");
    ScheduleProcessSwitchProtocol();
  }
}

/**
 * @brief  Schedule the process used to switch protocol
 * @param  None
 * @retval None
 */
static void ScheduleProcessSwitchProtocol(void)
{
    UTIL_SEQ_SetTask(1 << CFG_TASK_ACTIVATE_PROTOCOL,CFG_SCH_PRIO_0);
}

/**
 * @brief  Process the activation of the new protocol
 * @param  None
 * @retval None
 */
static void Process_ActivateNewProtocol(void)
{
  APP_DBG("Process_ActivateNewProtocol");

  /* Toggle Mode flag */
  if(ConcurrentMode == MAC_ENABLE){
    ConcurrentMode = BLE_ENABLE;
  }else{
    ConcurrentMode = MAC_ENABLE;
  }

  /* Once Switch has been acknowledged from M0, starts appropriate Protocol Init */
  if(ConcurrentMode == MAC_ENABLE)
  {
    APP_DBG("INIT MAC");
    APP_FFD_MAC_802_15_4_Init(APP_MAC_802_15_4_FULL, &Mac_802_15_4_CmdBuffer);
  }
  else
  {
    APP_DBG("INIT BLE");
    APP_BLE_Init( );
  }
  FlagSwitchingOnGoing = 0;
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

static void appe_Tl_Init( void )
{
  TL_MM_Config_t tl_mm_config;
  SHCI_TL_HciInitConf_t SHci_Tl_Init_Conf;

  /**< Reference table initialization */
  TL_Init();

  /**< System channel initialization */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SYSTEM_HCI_ASYNCH_EVT, UTIL_SEQ_RFU, shci_user_evt_proc );
  SHci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&SystemCmdBuffer;
  SHci_Tl_Init_Conf.StatusNotCallBack = APPE_StatusNot;
  shci_init(APPE_SysUserEvtRx, (void*) &SHci_Tl_Init_Conf);

  /**< Memory Manager channel initialization */
  tl_mm_config.p_BleSpareEvtBuffer = 0U;
  tl_mm_config.p_SystemSpareEvtBuffer = SystemSpareEvtBuffer;
  tl_mm_config.p_AsynchEvtPool = EvtPool;
  tl_mm_config.AsynchEvtPoolSize = POOL_SIZE;
  TL_MM_Init( &tl_mm_config );

  TL_Enable();

  return;
}

/**
 * Led initialization
 */
static void Led_Init( void )
{
#if (CFG_LED_SUPPORTED == 1U)
  /**
   * Leds Initialization
   */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
#endif
  return;
}

/**
 * Status notification
 */
static void APPE_StatusNot( SHCI_TL_CmdStatus_t status )
{
  return;
}

/**
 * The type of the payload for a system user event is tSHCI_UserEvtRxParam
 * When the system event is both :
 *    - a ready event (subevtcode = SHCI_SUB_EVT_CODE_READY)
 *    - reported by the FUS (sysevt_ready_rsp == FUS_FW_RUNNING)
 * The buffer shall not be released
 * ( eg ((tSHCI_UserEvtRxParam*)pPayload)->status shall be set to SHCI_TL_UserEventFlow_Disable )
 * When the status is not filled, the buffer is released by default
 */
static void APPE_SysUserEvtRx( void * pPayload )
{
  TL_AsynchEvt_t *p_sys_event;
  p_sys_event = (TL_AsynchEvt_t*)(((tSHCI_UserEvtRxParam*)pPayload)->pckt->evtserial.evt.payload);

  switch(p_sys_event->subevtcode)
  {
    case SHCI_SUB_EVT_CODE_READY:
      APPE_SysEvtReadyProcessing();
      break;

    case SHCI_SUB_EVT_ERROR_NOTIF:
      APP_ScheduleError((SCHI_SystemErrCode_t) (p_sys_event->payload[0]));
      break;

    default:
      break;
  }
  return;
}

/**
  * @brief  Trace the error or the warning reported.
  * @param  ErrId :
  * @param  ErrCode
  * @retval None
  */

void APP_ScheduleError(uint32_t ErrId)
{

  switch(ErrId)
  {
  case ERR_APPLI_REC_MULTI_MSG_FROM_RFCore :
    APP_TraceError("ERROR : ERR_REC_MULTI_MSG_FROM_RFCore ",0);
    break;
  case ERR_INTERFACE_IPCC_SEND_ACK_TO_RFCore :
    APP_TraceError("ERROR : ERR_IPCC_SEND_ACK_TO_RFCore ",0);
    break;
  case ERR_WRONG_BINARY :
    APP_TraceError("ERROR : ERR_WRONG_BINARY ",0);
    break;
  case ERR_WRONG_FIRMWARE_CHECK :
    APP_TraceError("ERROR : ERR_WRONG_FIRMWARE_CHECK ",0);
    break;

  default :
    APP_TraceError("ERROR Unknown ",ErrId);
    break;
  }
}

/**
  * @brief  Warn the user that an error has occurred.In this case,
  *         the LEDs on the Board will start blinking.
  *
  * @param  pMess  : Message associated to the error.
  * @param  ErrCode: Error code associated to the module
  * @retval None
  */
static void APP_TraceError(char * pMess, uint32_t ErrCode)
{
  APP_DBG(pMess);
  while(1 == 1)
  {
    BSP_LED_Toggle(LED1);
    HAL_Delay(500);
    BSP_LED_Toggle(LED2);
    HAL_Delay(500);
    BSP_LED_Toggle(LED3);
    HAL_Delay(500);
  }
}


/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports static
 *        mode BLE with Mac
 * @param  None
 * @retval None
 */
static void APP_CheckWirelessFirmwareInfo(void)
{
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t *p_wireless_info = &wireless_info_instance;

  if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success) {
      APP_ScheduleError((uint32_t)ERR_WRONG_FIRMWARE_CHECK);
  }
  else {
    APP_DBG("**********************************************************");
    APP_DBG("WIRELESS COPROCESSOR FW:");
    /* Print version */
    APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

    switch (p_wireless_info->StackType) {
    case INFO_STACK_TYPE_BLE_MAC_STATIC:
      APP_DBG("FW Type : STACK_TYPE_BLE_MAC_STATIC");
      break;
    default:
      APP_ScheduleError((uint32_t)ERR_WRONG_BINARY);
      break;
    }
    APP_DBG("**********************************************************");
  }
}
/**
 * The type of the payload for a system user event is tSHCI_UserEvtRxParam
 * When the system event is both :
 *    - a ready event (subevtcode = SHCI_SUB_EVT_CODE_READY)
 *    - reported by the FUS (sysevt_ready_rsp == FUS_FW_RUNNING)
 * The buffer shall not be released
 * ( eg ((tSHCI_UserEvtRxParam*)pPayload)->status shall be set to SHCI_TL_UserEventFlow_Disable )
 * When the status is not filled, the buffer is released by default
 */
static void APPE_SysEvtReadyProcessing()
{
  /* Traces channel initialization */
  TL_TRACES_Init( );

  /* Check wireless version */
  APP_CheckWirelessFirmwareInfo();

  if(ConcurrentMode == MAC_ENABLE){
    APP_FFD_MAC_802_15_4_Init(APP_MAC_802_15_4_FULL, &Mac_802_15_4_CmdBuffer);
  }else{
     APP_BLE_Init();
  }

  UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);

  return;
}

/**
 * @brief Process the messages coming from the RF Core.
 * @param  None
 * @retval None
 */
void APP_ENTRY_ProcessMsgFromRFCoreTask(void)
{  
  if (CptReceiveMsgFromRFCore != 0U)
  {
    CptReceiveMsgFromRFCore = 0U;
    MAC_802_15_4_CallBack_Processing();
  }
}

/* Received trace buffer from M0 */
void TL_TRACES_EvtReceived( TL_EvtPacket_t * hcievt )
{
#if(CFG_DEBUG_TRACE != 0)
  /* Call write/print function using DMA from dbg_trace */
  /* - Cast to TL_AsynchEvt_t* to get "real" payload (without Sub Evt code 2bytes),
     - (-2) to size to remove Sub Evt Code */
  //DbgTraceWrite(1U, (const unsigned char *) ((TL_AsynchEvt_t *)(hcievt->evtserial.evt.payload))->payload, hcievt->evtserial.evt.plen - 2U);
#endif /* CFG_DEBUG_TRACE */
  /* Release buffer */
  TL_MM_EvtDone( hcievt );
}

/**
  * @brief  This function is called by the scheduler each time an event
  *         is pending.
  *
  * @param  evt_waited_bm : Event pending.
  * @retval None
  */
void UTIL_SEQ_EvtIdle( UTIL_SEQ_bm_t task_id_bm, UTIL_SEQ_bm_t evt_waited_bm )
{
  switch(evt_waited_bm)
  {
  case EVENT_ACK_FROM_RFCore_EVT:
    UTIL_SEQ_Run(0);
    break;
  case EVENT_DEVICE_RESET_CNF:
  case EVENT_SET_CNF :
  case EVENT_DEVICE_STARTED_CNF:
  case EVENT_DATA_CNF:
    UTIL_SEQ_Run(TASK_MSG_FROM_RF_CORE);
    break;
  case EVENT_SYNCHRO_BYPASS_IDLE:
    UTIL_SEQ_SetEvt(EVENT_SYNCHRO_BYPASS_IDLE);
    break;
  case EVENT_SET_SRV_ASSOC_IND:
    g_srvSerReq = CFG_ASSO_PENDING;
    UTIL_SEQ_SetTask(TASK_COORD_SRV, CFG_SCH_PRIO_0);
    UTIL_SEQ_Run(TASK_COORD_SRV);
    break;
  case EVENT_SET_SRV_DATA_DATA_IND:
    g_srvDataReq = CFG_SRV_DATA_REQ_NBR;
    UTIL_SEQ_SetTask(TASK_COORD_DATA, CFG_SCH_PRIO_0);
    break;
  default :
    /* default case */
    UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );
    break;
  }
}

void shci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask( 1U<<CFG_TASK_SYSTEM_HCI_ASYNCH_EVT, CFG_SCH_PRIO_0);
  return;
}

void shci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt( 1U<< CFG_EVT_SYSTEM_HCI_CMD_EVT_RESP);
  return;
}

void shci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt( 1U<< CFG_EVT_SYSTEM_HCI_CMD_EVT_RESP );
  return;
}

/**
  * @brief  Initialisation of the trace mechansimn
  * @param  None
  * @retval None
  */
#if(CFG_DEBUG_TRACE != 0U)
void DbgOutputInit( void )
{
  HW_UART_Init(CFG_DEBUG_TRACE_UART);
}

/**
  * @brief  Management of the traces
  * @param  p_data : data
  * @param  size : size
  * @param  call-back :
  * @retval None
  */
void DbgOutputTraces( uint8_t *p_data, uint16_t size, void (*cb)(void) )
{
  HW_UART_Transmit_DMA(CFG_DEBUG_TRACE_UART, p_data, size, cb);
}

#endif

/**
 * Button Initialization
 */

static void Button_Init( void )
{
#if (CFG_BUTTON_SUPPORTED == 1U)

  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);
#endif

  return;
}

/**
 * @brief This function manage the Push button action
 * @param  GPIO_Pin : GPIO pin which has been activated
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  switch (GPIO_Pin)
  {
  case BUTTON_SW1_PIN:
#if ENABLE_HARDWARE_TEST_MODE
    TestMode_CycleState();
#else
    APP_DBG("BUTTON 1 PUSHED ! : NO ACTION (Produkcni mod)");
#endif
    break;

  case BUTTON_SW2_PIN:
  	// PŘESMĚROVÁNO DO NAŠEHO MAC PROCESU
		APP_MAC_Magnet_Action();
		APP_DBG("BUTTON 2 PUSHED ! : STATE CHANGE");
    break;

  case BUTTON_SW3_PIN:
     APP_DBG("BUTTON 3 PUSHED ! : NO ACTION MAPPED ON SW3");
     break;

  default:
    break;
  }
  return;
}

void APP_ENTRY_RegisterCmdBuffer(TL_CmdPacket_t* p_buffer)
{
  p_mac_802_15_4_cmdbuffer = p_buffer;
}

TL_CmdPacket_t* MAC_802_15_4_GetCmdBuffer(void)
{
  return (TL_CmdPacket_t*)p_mac_802_15_4_cmdbuffer;
}

TL_Evt_t* MAC_802_15_4_GetRspPayEvt(void)
{
  return &((TL_EvtPacket_t *)p_mac_802_15_4_cmdbuffer)->evtserial.evt;
}

TL_Evt_t* MAC_802_15_4_GetNotificationBuffer(void)
{
  return &(p_mac_802_15_4_notif_RFCore_to_M4->evtserial.evt);
}

MAC_802_15_4_Notification_t* MAC_802_15_4_GetNotificationPayloadBuffer(void)
{
  return (MAC_802_15_4_Notification_t*)(p_mac_802_15_4_notif_RFCore_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the MAC 802.15.4 commands from the
 *         M4 to the RFCore.
 *
 * @param   pCmdBuffer : pointer to the buffer to send
 * @return  None
 */
void Mac_802_15_4_CmdTransfer(void)
{
  TL_MAC_802_15_4_SendCmd();

  /* Wait completion of cmd */
  Wait_Getting_Ack_From_RFCore();
}

/* Initialisation of the transport layer */
void APP_ENTRY_TL_MAC_802_15_4_Init(void)
{
  Mac_802_15_4_ConfigBuffer.p_Mac_802_15_4_CmdRspBuffer = (uint8_t*)&Mac_802_15_4_CmdBuffer;
  Mac_802_15_4_ConfigBuffer.p_Mac_802_15_4_NotAckBuffer = (uint8_t*)Mac_802_15_4_NotifRspEvtBuffer;
  TL_MAC_802_15_4_Init( &Mac_802_15_4_ConfigBuffer );
}

/* For reception of MAC 802.15.4 Cmd return */
void TL_MAC_802_15_4_CmdEvtReceived( TL_EvtPacket_t * Otbuffer )
{
  Receive_Ack_From_RFCore();
}

/* For reception of MAC 802.15.4 Notification from RFCore */
void TL_MAC_802_15_4_NotReceived( TL_EvtPacket_t * Notbuffer )
{
  p_mac_802_15_4_notif_RFCore_to_M4 = Notbuffer;

  Receive_Notification_From_RFCore();
}

/**
  * @brief  This function is called before sending any ot command to the RFCore
  *         core. The purpose of this function is to be able to check if
  *         there are no notifications coming from the RFCore core which are
  *         pending before sending a new ot command.
  * @param  None
  * @retval None
  */
void Mac_802_15_4_PreCmdProcessing(void)
{
  UTIL_SEQ_WaitEvt( EVENT_SYNCHRO_BYPASS_IDLE );
}



/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Init_Debug( void )
{
#if (CFG_DEBUGGER_SUPPORTED == 1)
  /**
   * Keep debugger enabled while in any low power mode
   */
  HAL_DBGMCU_EnableDBGSleepMode();
  
  /* Enable debugger EXTI lines */
  LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_48);
  LL_C2_EXTI_EnableIT_32_63(LL_EXTI_LINE_48);

#else
  /* Disable debugger EXTI lines */
  LL_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);
  LL_C2_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);

  GPIO_InitTypeDef gpio_config = {0};

  gpio_config.Pull = GPIO_NOPULL;
  gpio_config.Mode = GPIO_MODE_ANALOG;

  gpio_config.Pin = GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_Init(GPIOA, &gpio_config);
  __HAL_RCC_GPIOA_CLK_DISABLE();

  gpio_config.Pin = GPIO_PIN_4 | GPIO_PIN_3;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_Init(GPIOB, &gpio_config);
  __HAL_RCC_GPIOB_CLK_DISABLE();

  /**
   * Do not keep debugger enabled while in any low power mode
   */
  HAL_DBGMCU_DisableDBGSleepMode();
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_DBGMCU_DisableDBGStandbyMode();
#endif /* (CFG_DEBUGGER_SUPPORTED == 1) */
  
#if(CFG_DEBUG_TRACE != 0)
  DbgTraceInit();
#endif
  
  return;
}

/**
 * @brief  Configure the system for power optimization
 *
 * @note  This API configures the system to be ready for low power mode
 *
 * @param  None
 * @retval None
 */
static void SystemPower_Config( void )
{
  // Before going to stop or standby modes, do the settings so that system clock and IP80215.4 clock
  // start on HSI automatically
  LL_RCC_HSI_EnableAutoFromStop();
  
  /**
   * Select HSI as system clock source after Wake Up from Stop mode
   */
  LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);

  /* Initialize low power manager */
  UTIL_LPM_Init( );
  
  /* Disable low power mode until INIT is complete */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
  UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
  
#if (CFG_USB_INTERFACE_ENABLE != 0)
  /**
   *  Enable USB power
   */
  HAL_PWREx_EnableVddUSB();
#endif

  /* Enable RAM1 (because OT instance.o is located here for Concurrent Mode */
  LL_C2_AHB1_GRP1_EnableClock(LL_C2_AHB1_GRP1_PERIPH_SRAM1);
  LL_C2_AHB1_GRP1_EnableClockSleep(LL_C2_AHB1_GRP1_PERIPH_SRAM1);

  return;
}

/**
  * @brief  This function waits for getting an acknowledgment from the RFCore.
  *
  * @param  None
  * @retval None
  */
static void Wait_Getting_Ack_From_RFCore(void)
{
  UTIL_SEQ_WaitEvt(EVENT_ACK_FROM_RFCore_EVT);
}

/**
  * @brief  Receive an acknowledgment from the RFCore core.
  *         Each command send by the M4 to the RFCore are acknowledged.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Ack_From_RFCore(void)
{
  UTIL_SEQ_SetEvt(EVENT_ACK_FROM_RFCore_EVT);
}

/**
  * @brief  Receive a notification from the RFCore+ through the IPCC.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Notification_From_RFCore(void)
{
  CptReceiveMsgFromRFCore = 1;
  UTIL_SEQ_SetTask(TASK_MSG_FROM_RF_CORE,CFG_SCH_PRIO_0);
}

#if (NBIOT_HARDWARE_CONNECTED == 1)
static void RxUART_Init(void)
{
  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1U, RxCpltCallback);
}
#endif

static void RxCpltCallback(void)
{
  /* Filling buffer and wait for '\r' char */
  if (indexReceiveChar < C_SIZE_CMD_STRING)
  {
    if (aRxBuffer[0] == '\r')
    {
      APP_DBG("received %s", CommandString);

      UartCmdExecute();

      /* Clear receive buffer and character counter*/
      indexReceiveChar = 0;
      memset(CommandString, 0, C_SIZE_CMD_STRING);
    }
    else if (aRxBuffer[0] == '\n')
    {
      /* Clear receive buffer and character counter*/
      indexReceiveChar = 0;
      memset(CommandString, 0, C_SIZE_CMD_STRING);
    }
    else
    {
      CommandString[indexReceiveChar++] = aRxBuffer[0];
    }
  }

  /* Once a character has been sent, put back the device in reception mode */
  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1U, RxCpltCallback);
}

static void UartCmdExecute(void)
{
  /* Parse received CommandString */
  if(strcmp((char const*)CommandString, "SW1") == 0)
  {
    APP_DBG("SW1 OK");
    exti_handle.Line = EXTI_LINE_4;
    HAL_EXTI_GenerateSWI(&exti_handle);
  }
  else if (strcmp((char const*)CommandString, "SW2") == 0)
  {
    APP_DBG("SW2 OK");
    exti_handle.Line = EXTI_LINE_0;
    HAL_EXTI_GenerateSWI(&exti_handle);
  }
  else if (strcmp((char const*)CommandString, "SW3") == 0)
  {
    APP_DBG("SW3 OK");
    exti_handle.Line = EXTI_LINE_1;
    HAL_EXTI_GenerateSWI(&exti_handle);
  }
  else
  {
    APP_DBG("NOT RECOGNIZED COMMAND : %s", CommandString);
  }
}

#if (CFG_USB_INTERFACE_ENABLE != 0)
// Ve tvé inicializační funkci (např. tam, kde spouštíš UART nebo BLE)
void USB_App_Init(void)
{
	// --- NOVÉ: Donutíme Windows zapomenout staré spojení ---
	Force_USB_Reenumeration();

  // 1. Spustíme hardwarové hodiny z Kroku 2
  HW_USB_Clock_Init();

  // 2. Inicializace ST USB Device knihovny
  USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS);

  // 3. Zaregistrování třídy CDC (Virtual COM Port)
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);

  // 4. Připojení tvého rozhraní pro čtení/zápis (soubor usbd_cdc_if.c)
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);

  // 5. Otevření brány (Zařízení se ukáže v PC)
  USBD_Start(&hUsbDeviceFS);

  APP_DBG(">>> USB CDC: Inicializovano a spusteno!");
}
#endif




