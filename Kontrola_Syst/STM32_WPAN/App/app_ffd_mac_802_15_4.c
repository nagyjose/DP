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
#include "hw_if.h"
#include <stdbool.h>
#include "flash_logger.h"
#include "app_conf.h"

/* Defines -----------------------------------------------*/
#define DEMO_CHANNEL 26

/* Private function prototypes -----------------------------------------------*/
static void APP_FFD_MAC_802_15_4_Config(void);
static void BeaconTimer_Callback(void);
void APP_MAC_SendBeaconTask(void);
extern void APP_MAC_RestartBeaconTimer(void);

// =========================================================================
// PŘIDÁNO: Dopředná deklarace a callback pro Bzučák
// =========================================================================
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

// NASE NOVE PROMENNE PRO MAJAK
uint8_t BeaconTimerId;
uint8_t BuzzerTimerId; // <--- PŘIDÁNO: Proměnná pro bzučák

// Globální semafor (zámek)
volatile bool can_send_beacon = true;

// =============================================================================
// TESTOVACÍ STAVOVÝ AUTOMAT (POUZE PRO VÝVOJ)
// =============================================================================

#if ENABLE_HARDWARE_TEST_MODE
volatile uint8_t test_machine_state = 0; // 0=Vypnuto (Config), 1=CLEAR, 2=CHECK, 3=START, 4=CIL, 5=STANDARD

void TestMode_CycleState(void) {
	test_machine_state++;
	if (test_machine_state > 9) test_machine_state = 0;

	switch(test_machine_state) {
		case 0: APP_DBG("TEST MOD: VYPNUTO (Ctu data z Flash)"); break;
		case 1: APP_DBG("TEST MOD: CLEAR (ID 0)"); break;
		case 2: APP_DBG("TEST MOD: CHECK (ID 1)"); break;
		case 3: APP_DBG("TEST MOD: START (ID 2)"); break;
		case 4: APP_DBG("TEST MOD: STANDARDNI (ID 31)"); break;
		case 5: APP_DBG("TEST MOD: STANDARDNI (ID 32)"); break;
		case 6: APP_DBG("TEST MOD: STANDARDNI (ID 33)"); break;
		case 7: APP_DBG("TEST MOD: STANDARDNI (ID 34)"); break;
		case 8: APP_DBG("TEST MOD: STANDARDNI (ID 35)"); break;
		case 9: APP_DBG("TEST MOD: CIL (ID 3)"); break;
	}
}

uint16_t TestMode_IdDefine(uint8_t test_machine_state) {
	// OPRAVA: Výchozí ID není 0, ale to reálné z Flash!
	uint16_t control_id = DEVICE_CONFIG->stat_device_type;

	if (test_machine_state == 1) control_id = 0;       // Vynutí CLEAR
	else if (test_machine_state == 2) control_id = 1;  // Vynutí CHECK
	else if (test_machine_state == 3) control_id = 2;  // Vynutí START
	else if (test_machine_state == 4) control_id = 31; // Vynutí STANDARDNÍ
	else if (test_machine_state == 5) control_id = 32; // Vynutí STANDARDNÍ
	else if (test_machine_state == 6) control_id = 33; // Vynutí STANDARDNÍ
	else if (test_machine_state == 7) control_id = 34; // Vynutí STANDARDNÍ
	else if (test_machine_state == 8) control_id = 35; // Vynutí STANDARDNÍ
	else if (test_machine_state == 9) control_id = 3;  // Vynutí CIL

	return control_id;
}
#endif
// =============================================================================


/* Functions Definition ------------------------------------------------------*/
void APP_FFD_MAC_802_15_4_Init( APP_MAC_802_15_4_InitMode_t InitMode, TL_CmdPacket_t* pCmdBuffer)
{
  APP_ENTRY_RegisterCmdBuffer(pCmdBuffer);
  APP_ENTRY_TL_MAC_802_15_4_Init();
  SHCI_C2_MAC_802_15_4_Init();

  // Inicializace tlačítka SW2 (Simulace magnetu) v režimu přerušení
	BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);

  /* Register task */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_MSG_FROM_RF_CORE, UTIL_SEQ_RFU, APP_ENTRY_ProcessMsgFromRFCoreTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_FFD, UTIL_SEQ_RFU,APP_FFD_MAC_802_15_4_SetupTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SERVICE_COORD, UTIL_SEQ_RFU,APP_FFD_MAC_802_15_4_CoordSrvTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_COORD, UTIL_SEQ_RFU,APP_FFD_MAC_802_15_4_CoordDataTask);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_RECEIVE_DATA, UTIL_SEQ_RFU,APP_MAC_ReceiveData);

  // NASE NOVA ULOHA PRO VYSILANI MAJAKU
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SEND_BEACON, UTIL_SEQ_RFU, APP_MAC_SendBeaconTask);

  // Vytvoreni hardwaroveho casovace
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &BeaconTimerId, hw_ts_SingleShot, BeaconTimer_Callback);

  // =========================================================================
	// PŘIDÁNO: Vytvoření časovače a Tasku pro bzučák, ať nám nekrade Maják!
	// =========================================================================
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &BuzzerTimerId, hw_ts_SingleShot, BuzzerTimer_Callback);
	UTIL_SEQ_RegTask( 1<<CFG_TASK_BUZZER, UTIL_SEQ_RFU, APP_MAC_BuzzerTask);

  /* Configuration MAC 802_15_4 */
  APP_FFD_MAC_802_15_4_Config();

  /*Start Main Coordinator - FFD Task*/
  UTIL_SEQ_SetTask( 1<< CFG_TASK_FFD, CFG_SCH_PRIO_0 );
}

void APP_FFD_MAC_802_15_4_Stop()
{
  APP_DBG(">>> MAC TEARDOWN: Zastavuji 50ms kulomet...");

  // 1. NEJPRVE ABSOLUTNĚ ZABRÁNIT DALŠÍMU VYSÍLÁNÍ
  can_send_beacon = false;

  // 2. FYZICKY ZABÍT VŠECHNY ČASOVAČE
  HW_TS_Stop(BeaconTimerId);
  HW_TS_Delete(BeaconTimerId);
  HW_TS_Stop(BuzzerTimerId);
  HW_TS_Delete(BuzzerTimerId);

  // 3. TEPRVE NYNÍ JE BEZPEČNÉ VYRESETOVAT KOPROCESOR
  MAC_resetReq_t ResetReq;
  memset(&ResetReq,0x00,sizeof(MAC_resetReq_t));
  ResetReq.set_default_PIB = TRUE;
  MAC_MLMEResetReq( &ResetReq );
  UTIL_SEQ_WaitEvt(EVENT_DEVICE_RESET_CNF);

  // 4. USPAT ZBYTKOVÉ TASKY
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_MSG_FROM_RF_CORE);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_FFD);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_SERVICE_COORD);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_DATA_COORD);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_RECEIVE_DATA);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_SEND_BEACON);
  UTIL_SEQ_PauseTask( 1<<CFG_TASK_BUZZER);

  BSP_LED_Off(LED_BLUE);
  APP_DBG(">>> MAC TEARDOWN: Hotovo, prepinam na BLE.");
}

void APP_FFD_MAC_802_15_4_CoordSrvTask(void)
{
  // PRO KONTROLU NEPOTREBUJEME ZADNOU ASOCIACI, ZAVODNIK JEN POSLOUCHA
  g_srvSerReq = CFG_SRV_SER_REQ_NBR;
}

void APP_FFD_MAC_802_15_4_CoordDataTask(void)
{
  g_srvDataReq = CFG_SRV_DATA_REQ_NBR;
}

void APP_FFD_MAC_802_15_4_SetupTask(void)
{
  //MAC_Status_t MacStatus = MAC_ERROR;
  MAC_resetReq_t    ResetReq;
  MAC_setReq_t      SetReq;
  MAC_startReq_t    StartReq;

  long long extAddr = 0xACDE480000000001; // Zde pak dame unikatni MAC adresu Kontroly
  uint16_t shortAddr   = 0x1122;
  uint16_t panId       = 0x1AAA;
  uint8_t channel      = DEMO_CHANNEL;
  uint8_t PIB_Value = 0x00;
  int8_t tx_power_pib_value = 0;

  APP_DBG("Run FFD MAC 802.15.4 - KONTROLA BEACON STARTUP");

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

  /* Vypiname association permit - jsme jen vysilac majaku */
  memset(&SetReq,0x00,sizeof(MAC_setReq_t));
  SetReq.PIB_attribute = g_MAC_ASSOCIATION_PERMIT_c;
  PIB_Value = g_FALSE;
  SetReq.PIB_attribute_valuePtr = &PIB_Value;
  MAC_MLMESetReq( &SetReq );
  UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF );

  /* Nastaveni sily signalu DYNAMICKY Z FLASH PAMĚTI */
	memset(&SetReq,0x00,sizeof(MAC_setReq_t));
	SetReq.PIB_attribute = g_PHY_TRANSMIT_POWER_c;

	tx_power_pib_value = DEVICE_CONFIG->tx_power;
	// Ochrana proti hodnotám mimo limit čipu ST
	if (tx_power_pib_value > 6) tx_power_pib_value = 6;
	if (tx_power_pib_value < -21) tx_power_pib_value = -21;

	SetReq.PIB_attribute_valuePtr = (uint8_t *)&tx_power_pib_value;
	MAC_MLMESetReq( &SetReq );
	UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF );
  
	// Tvorba pasivní sítě - my vysíláme broadcast data, ne HW majáky
	memset(&StartReq,0x00,sizeof(MAC_startReq_t));
	memcpy(StartReq.a_PAN_id,(uint8_t*)&panId,0x02);
	StartReq.channel_number   = channel;
	StartReq.beacon_order     = 0x0F;
	StartReq.superframe_order = 0x0F;
	StartReq.PAN_coordinator  = g_TRUE;
	StartReq.battery_life_extension = g_FALSE;
	MAC_MLMEStartReq( &StartReq);
	UTIL_SEQ_WaitEvt( 1U << CFG_EVT_DEVICE_STARTED_CNF );

	// !!! USPAT ANTÉNU - Necháme prostor pro nabootování BLE Tunelu !!!
	memset(&SetReq,0x00,sizeof(MAC_setReq_t));
	SetReq.PIB_attribute = g_MAC_RX_ON_WHEN_IDLE_c;
	PIB_Value = g_TRUE;
	SetReq.PIB_attribute_valuePtr = &PIB_Value;
	MAC_MLMESetReq( &SetReq );
	UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF );

  APP_DBG("KONTROLA READY - Zapinam pravidelne vysilani majaku!");
  BSP_LED_On(LED_BLUE);

  // Vzdy nabihat z BLE do usporneho rezimu (1x za sekundu)
	current_state = STATE_ACTIVE_MAC;

  // ZDE POPRVE ODPALIME CASOVAC (Zacne vysilat pakety)
  APP_MAC_RestartBeaconTimer();
}


extern RTC_HandleTypeDef hrtc; // Zpřístupníme si RTC strukturu z main.c

// -----------------------------------------------------------------------------
// VYLEPŠENÝ PŘEVOD: Vrací pouze sekundy od 1.1.2000 (Zabraňuje přetečení)
// -----------------------------------------------------------------------------
uint32_t Get_Calendar_Seconds_Since_2000(RTC_DateTypeDef *sDate, RTC_TimeTypeDef *sTime)
{
    uint8_t a = (14 - sDate->Month) / 12;
    uint16_t y = (sDate->Year + 2000) + 4800 - a; // RTC vrací rok 0-99
    uint8_t m = sDate->Month + (12 * a) - 3;

    uint32_t JDN = sDate->Date;
    JDN += (153 * m + 2) / 5;
    JDN += 365 * y;
    JDN += y / 4;
    JDN -= y / 100;
    JDN += y / 400;
    JDN -= 32045;

    // 2451545 je Juliánský den pro 1. 1. 2000
    uint32_t days_since_2000 = JDN - 2451545;

    uint32_t cal_seconds = days_since_2000 * 86400;
    cal_seconds += sTime->Hours * 3600;
    cal_seconds += sTime->Minutes * 60;
    cal_seconds += sTime->Seconds;

    return cal_seconds;
}

// -----------------------------------------------------------------------------
// ODESÍLACÍ FUNKCE KONTROLY (S OCHRANOU PROTI ZPOMALENÉMU RTC)
// -----------------------------------------------------------------------------
void APP_MAC_SendBeaconTask(void)
{
	// 1. OCHRANA PROTI ZADUŠENÍ (Semafor)
	if (!can_send_beacon) {
		// Koprocesor ještě bojuje s předchozím paketem!
		// Přeskočíme toto kolo, ať ho neudusíme.
		return;
	}

	static uint8_t msduHandle = 0;
	static uint8_t payload[6];

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	// 1. Odemknutí registrů
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	// 2. ČTENÍ PŘÍMO Z HARDWARU
	uint32_t ssr = RTC->SSR;
	uint32_t prer_s = RTC->PRER & RTC_PRER_PREDIV_S;
	uint32_t prer_a = (RTC->PRER & RTC_PRER_PREDIV_A) >> 16;

	if (prer_s == 0) prer_s = 255;
	if (prer_a == 0) prer_a = 127;

	// Zjistíme frekvenci (obvykle 256 Hz) a kolik reálných sekund trvá 1 tik kalendáře (obvykle 10s u ST)
	uint32_t ssr_freq = 32768 / (prer_a + 1);
	uint32_t seconds_per_tick = (prer_s + 1) / ssr_freq;
	if (seconds_per_tick == 0) seconds_per_tick = 1;

	// Přesný počet milisekund od posledního kalendářního tiku
	uint32_t ms_within_tick = ((prer_s - ssr) * 1000) / ssr_freq;

	// 3. ZÍSKÁNÍ PŘESNÝCH DESETIN A UNIX ČASU
	uint8_t tenths = (ms_within_tick % 1000) / 100;

	uint32_t cal_seconds = Get_Calendar_Seconds_Since_2000(&sDate, &sTime);
	uint32_t real_seconds_since_2000 = (cal_seconds * seconds_per_tick) + (ms_within_tick / 1000);

	// 946684800 je přesný Unix Timestamp pro 1. 1. 2000
	uint32_t unix_time = 946684800 + real_seconds_since_2000;

	// Čteme ID kontroly přímo z Flash paměti
	uint16_t control_id = DEVICE_CONFIG->stat_device_type;

	// =========================================================================
	// INJEKCE TESTOVACÍHO STAVOVÉHO AUTOMATU
	// =========================================================================
#if ENABLE_HARDWARE_TEST_MODE
	control_id = TestMode_IdDefine(test_machine_state);
#endif
	// =========================================================================

	// PŘIDÁNO: Výpis do terminálu, abychom nebyli slepí!
	//APP_DBG(">>> ODESILAM MAJAK (ID: %d) <<<", control_id);

	// =========================================================================
	// 4. BITOVÁ MAGIE: SKLÁDÁNÍ DO 6 BAJTŮ PODLE TYPU KONTROLY
	// =========================================================================
	if (control_id == 0)
	{
		// --- A) KONTROLA CLEAR ---
		// Nepotřebuje čas! Místo toho posílá parametry závodu.
		uint32_t race_id = DEVICE_CONFIG->event_id;
		uint8_t country = DEVICE_CONFIG->event_nation;
		uint8_t req_rssi = DEVICE_CONFIG->required_rssi;

		// Byte 0: ID (horních 8 bitů - vždy 0x00 pro CLEAR)
		payload[0] = 0x00;
		// Byte 1: ID (dolní 4 bity - 0x0) | Horní 4 bity z 20bitového race_id
		payload[1] = (uint8_t)((race_id >> 16) & 0x0F);
		// Byte 2: Prostředních 8 bitů z race_id
		payload[2] = (uint8_t)(race_id >> 8);
		// Byte 3: Dolních 8 bitů z race_id
		payload[3] = (uint8_t)(race_id & 0xFF);
		// Byte 4: Stát
		payload[4] = country;
		// Byte 5: RSSI threshold
		payload[5] = req_rssi;
	}
	else
	{
		// --- B) STANDARDNÍ KONTROLA (Start, Cíl, Check, Normální) ---
		// Posílá 12bit ID a k tomu přidává RTC čas a desetiny sekundy.
		payload[0] = (uint8_t)(control_id >> 4);
		payload[1] = (uint8_t)((control_id << 4) | (tenths & 0x0F));
		payload[2] = (uint8_t)(unix_time >> 24);
		payload[3] = (uint8_t)(unix_time >> 16);
		payload[4] = (uint8_t)(unix_time >> 8);
		payload[5] = (uint8_t)(unix_time & 0xFF);
	}

	// 5. ODESLÁNÍ DO VZDUCHU
	MAC_dataReq_t dataReq;
	memset(&dataReq, 0, sizeof(MAC_dataReq_t));

	dataReq.src_addr_mode = g_SHORT_ADDR_MODE_c;
	dataReq.dst_addr_mode = g_SHORT_ADDR_MODE_c;

	uint16_t panId = 0x1AAA;
	memcpy(dataReq.a_dst_PAN_id, (uint8_t*)&panId, 2);

	uint16_t destAddr = 0xFFFF;
	memcpy(&dataReq.dst_address, (uint8_t*)&destAddr, 2);

	dataReq.msdu_length = 6;
	dataReq.msduPtr = payload;
	dataReq.msdu_handle = msduHandle++;

	dataReq.ack_Tx = 0x00;
	dataReq.GTS_Tx = 0x00;
	dataReq.indirect_Tx = 0x00;

	// 2. ZAMKNUTÍ SEMAFORU A ODESLÁNÍ
	can_send_beacon = false;
	// Ochrana proti přerušení řetězce
	if (MAC_MCPSDataReq(&dataReq) != MAC_SUCCESS) {
	    can_send_beacon = true; // Fronta selhala ihned, odemkneme
	}

	BSP_LED_Toggle(LED_GREEN);
}

static void BeaconTimer_Callback(void)
{
	static uint8_t stuck_counter = 0;
	static uint8_t fatal_error_counter = 0; // NOVÁ PROMĚNNÁ

	// POJISTKA SEMAFORU (Timeout)
	if (!can_send_beacon) {
		stuck_counter++;
		if (stuck_counter > 5) {
			// Rádio nám už 5 cyklů (250 ms) nepotvrdilo odeslání, semafor se zasekl!
			// Násilím ho odemkneme, aby deska neztichla.
			can_send_beacon = true;
			stuck_counter = 0;

			// POČÍTÁME SELHÁNÍ RÁDIA
			fatal_error_counter++;
			if (fatal_error_counter > 10) {
				// Rádio nás ignorovalo 10x po sobě. Je definitivně mrtvé!
				// Zastavíme procesor v nekonečné smyčce. Pes do 1 vteřiny celou desku zresetuje.
				while(1);
			}
		}
	} else {
		stuck_counter = 0;
		fatal_error_counter = 0; // Rádio odpovědělo, vše je OK
	}

	// =========================================================================
	// STAVOVÝ AUTOMAT: VÝPOČET FREKVENCE MAJÁKU
	// =========================================================================
	uint32_t period_ms;

	if (current_state == STATE_IDLE_MAC) {
		// ÚSPORNÝ REŽIM: 1x za sekundu (Šetříme baterii před závodem)
		period_ms = 1000;
	} else {
		// ZÁVODNÍ REŽIM: Rychlá palba podle konfigurace (např. 50 ms)
		period_ms = DEVICE_CONFIG->beacon_period_ms;
		if (period_ms < 10) period_ms = 50; // Bezpečnostní pojistka
	}

	// Časovač nezávisle a neúprosně tiká
	HW_TS_Start(BeaconTimerId, (uint32_t)(period_ms * 1000 / CFG_TS_TICK_VAL));
	UTIL_SEQ_SetTask(1 << CFG_TASK_SEND_BEACON, CFG_SCH_PRIO_0);
}


// =========================================================================
// 1. CONFIG: REGISTRACE CALLBACKŮ A AGRESIVNÍ VYSÍLÁNÍ
// =========================================================================
static void APP_FFD_MAC_802_15_4_Config()
{
  memset(&macCbConfig,0x00,sizeof(MAC_callbacks_t));

  macCbConfig.mlmeResetCnfCb = APP_MAC_mlmeResetCnfCb;
  macCbConfig.mlmeSetCnfCb = APP_MAC_mlmeSetCnfCb;
  macCbConfig.mlmeStartCnfCb = APP_MAC_mlmeStartCnfCb;
  macCbConfig.mcpsDataIndCb = APP_MAC_mcpsDataIndCb; // Zpracování přijatého pípnutí
  macCbConfig.mcpsDataCnfCb = APP_MAC_mcpsDataCnfCb; // Uvolnění semaforu can_send_beacon!

  // --- AGRESIVNÍ VYSÍLÁNÍ (Vypnutí CSMA/CA) ---
  MAC_setReq_t setReq;
  uint8_t max_backoffs = 0;
  setReq.PIB_attribute = 0x47; // macMaxCSMABackoffs
  setReq.PIB_attribute_valuePtr = &max_backoffs;
  MAC_MLMESetReq(&setReq);
}

