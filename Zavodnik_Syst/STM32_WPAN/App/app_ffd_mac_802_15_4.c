/**
  ******************************************************************************
 * @file    app_ffd_mac_802_15_4.c
 * @author  MCD Application Team
 * @brief   Application based on MAC 802.15.4
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "stm_queue.h"
#include "utilities_common.h"
#include "app_entry.h"
#include "app_ffd_mac_802_15_4.h"
#include "802_15_4_mac_sap.h"
#include "app_ffd_mac_802_15_4_process.h"
#include "dbg_trace.h"
#include "shci.h"
#include "stm_logging.h"

/* Defines -----------------------------------------------*/
#define DEMO_CHANNEL 26

/* Private function prototypes -----------------------------------------------*/
static void APP_FFD_MAC_802_15_4_Config(void);
extern void APP_MAC_BuzzerTask(void);
static void BuzzerTimer_Callback(void) {
	UTIL_SEQ_SetTask(1 << CFG_TASK_BUZZER, CFG_SCH_PRIO_0);
}

/* variables -----------------------------------------------*/
MAC_associateInd_t g_MAC_associateInd;
MAC_callbacks_t macCbConfig ;
uint8_t g_srvSerReq;
uint8_t g_srvDataReq;
__IO ITStatus CertifOutputPeripheralReady = SET;
uint8_t BuzzerTimerId;

/* Functions Definition ------------------------------------------------------*/
void APP_FFD_MAC_802_15_4_Init( APP_MAC_802_15_4_InitMode_t InitMode, TL_CmdPacket_t* pCmdBuffer)
{
  APP_ENTRY_RegisterCmdBuffer(pCmdBuffer);
  APP_ENTRY_TL_MAC_802_15_4_Init();
  SHCI_C2_MAC_802_15_4_Init();

  /* Register task */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_MSG_FROM_RF_CORE, UTIL_SEQ_RFU, APP_ENTRY_ProcessMsgFromRFCoreTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_FFD, UTIL_SEQ_RFU,APP_FFD_MAC_802_15_4_SetupTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SERVICE_COORD, UTIL_SEQ_RFU,APP_FFD_MAC_802_15_4_CoordSrvTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_COORD, UTIL_SEQ_RFU,APP_FFD_MAC_802_15_4_CoordDataTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_RECEIVE_DATA, UTIL_SEQ_RFU,APP_MAC_ReceiveData);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_BUZZER, UTIL_SEQ_RFU, APP_MAC_BuzzerTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_MAC_SNIFF, UTIL_SEQ_RFU, APP_MAC_SniffTask);
	UTIL_SEQ_RegTask( 1<<CFG_TASK_MAC_SLEEP, UTIL_SEQ_RFU, APP_MAC_SleepTask);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &BuzzerTimerId, 0, BuzzerTimer_Callback);

  APP_FFD_MAC_802_15_4_Config();
  //Race_StateMachine_Init(); // <-- ZDE SPUSTÍME NÁŠ AUTOMAT
  UTIL_SEQ_SetTask( 1<< CFG_TASK_FFD, CFG_SCH_PRIO_0 );
}

void APP_FFD_MAC_802_15_4_Stop()
{
  MAC_resetReq_t ResetReq;
  memset(&ResetReq,0x00,sizeof(MAC_resetReq_t));
  ResetReq.set_default_PIB = TRUE;
  MAC_MLMEResetReq( &ResetReq );
  UTIL_SEQ_WaitEvt(EVENT_DEVICE_RESET_CNF);

  UTIL_SEQ_PauseTask( 1<<CFG_TASK_MSG_FROM_RF_CORE);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_FFD);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_SERVICE_COORD);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_DATA_COORD);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_RECEIVE_DATA);
  BSP_LED_Off(LED_BLUE);
}

void APP_FFD_MAC_802_15_4_CoordSrvTask(void)
{
  g_srvSerReq = CFG_SRV_SER_REQ_NBR;
}

void APP_FFD_MAC_802_15_4_CoordDataTask(void)
{
  g_srvDataReq = CFG_SRV_DATA_REQ_NBR;
}

void APP_FFD_MAC_802_15_4_SetupTask(void)
{
  MAC_resetReq_t    ResetReq;
  MAC_setReq_t      SetReq;
  MAC_startReq_t    StartReq;

  long long extAddr = 0xACDE480000000002; // ZÁVODNÍK MÁ JINOU MAC ADRESU
  uint16_t shortAddr   = 0x2233;          // ZÁVODNÍK MÁ JINOU SHORT ADRESU
  uint16_t panId       = 0x1AAA;          // PAN ID (Sit) musí být stejné!
  uint8_t channel      = DEMO_CHANNEL;
  uint8_t PIB_Value = 0x00;

  APP_DBG("Run MAC 802.15.4 - ZAVODNIK (SKENER) STARTUP");

  memset(&ResetReq,0x00,sizeof(MAC_resetReq_t));
  ResetReq.set_default_PIB = TRUE;
  MAC_MLMEResetReq( &ResetReq );
  UTIL_SEQ_WaitEvt( 1U<< CFG_EVT_DEVICE_RESET_CNF );

  memset(&SetReq,0x00,sizeof(MAC_setReq_t));
  SetReq.PIB_attribute = g_MAC_EXTENDED_ADDRESS_c;
  SetReq.PIB_attribute_valuePtr = (uint8_t*) &extAddr;
  MAC_MLMESetReq( &SetReq );
  UTIL_SEQ_WaitEvt( 1U<< CFG_EVT_SET_CNF );

  memset(&SetReq,0x00,sizeof(MAC_setReq_t));
  SetReq.PIB_attribute = g_MAC_SHORT_ADDRESS_c;
  SetReq.PIB_attribute_valuePtr =(uint8_t*) &shortAddr;
  MAC_MLMESetReq( &SetReq );
  UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF );

  /* Závodník netvoří síť pro ostatní */
  memset(&SetReq,0x00,sizeof(MAC_setReq_t));
  SetReq.PIB_attribute = g_MAC_ASSOCIATION_PERMIT_c;
  PIB_Value = g_FALSE;
  SetReq.PIB_attribute_valuePtr = &PIB_Value;
  MAC_MLMESetReq( &SetReq );
  UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF );

  /* Závodník NENÍ Koordinátor (PAN_coordinator = g_FALSE) */
  memset(&StartReq,0x00,sizeof(MAC_startReq_t));
  memcpy(StartReq.a_PAN_id,(uint8_t*)&panId,0x02);
  StartReq.channel_number   = channel;
  StartReq.beacon_order     = 0x0F;
  StartReq.superframe_order = 0x0F;
  StartReq.PAN_coordinator  = g_FALSE;
  StartReq.battery_life_extension = g_FALSE;
  MAC_MLMEStartReq( &StartReq);
  UTIL_SEQ_WaitEvt( 1U << CFG_EVT_DEVICE_STARTED_CNF );

  /* ZAPNUTÍ MAC VRSTVY - ALE VE STAVU USPANÉ ANTÉNY! */
	memset(&SetReq,0x00,sizeof(MAC_setReq_t));
	SetReq.PIB_attribute = g_MAC_RX_ON_WHEN_IDLE_c;

	// TADY MUSÍ BÝT g_FALSE! Kdyby tu bylo g_TRUE, anténa se uzamkne a BLE nevysílá!
	PIB_Value = g_FALSE;

	SetReq.PIB_attribute_valuePtr = &PIB_Value;
	MAC_MLMESetReq( &SetReq );
	UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF );

	// Upravili jsme text, ať nás to na terminálu nemate :)
	APP_DBG("ZAVODNIK READY - MAC Inicializovan a USPAN!");
	BSP_LED_On(LED_BLUE);

	// =========================================================================
	// SPUŠTĚNÍ AUTOMATU AŽ ZDE! (Nyní je M0+ bezpečně připraven ho poslechnout)
	// =========================================================================
	extern void Race_StateMachine_Init(void);
	Race_StateMachine_Init();
}

static void APP_FFD_MAC_802_15_4_Config()
{
  memset(&macCbConfig,0x00,sizeof(MAC_callbacks_t));

  macCbConfig.mlmeResetCnfCb = APP_MAC_mlmeResetCnfCb;
  macCbConfig.mlmeSetCnfCb = APP_MAC_mlmeSetCnfCb;
  macCbConfig.mlmeStartCnfCb = APP_MAC_mlmeStartCnfCb;
  macCbConfig.mcpsDataIndCb = APP_MAC_mcpsDataIndCb;

  // TÍMTO ŘÁDKEM PROPOJÍME TEN NÁŠ NOVÝ CALLBACK Z KROKU 1!
  macCbConfig.mcpsDataCnfCb = APP_MAC_mcpsDataCnfCb;

  // --- AGRESIVNÍ VYSÍLÁNÍ I PRO ZÁVODNÍKA (Vypnutí CSMA/CA) ---
	MAC_setReq_t setReq;
	uint8_t max_backoffs = 0;
	setReq.PIB_attribute = 0x47; // macMaxCSMABackoffs
	setReq.PIB_attribute_valuePtr = &max_backoffs;
	MAC_MLMESetReq(&setReq);
}


