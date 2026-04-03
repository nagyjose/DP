/**
  ******************************************************************************
* @file    app_mac_802-15-4_process.c
  ******************************************************************************
  */

#include "app_ffd_mac_802_15_4_process.h"
#include "app_conf.h"
#include "dbg_trace.h"
#include "shci.h"
#include "stm32_seq.h"
#include "app_ffd_mac_802_15_4.h"
#include "802_15_4_mac_sap.h"
#include "stm_logging.h"
#include "hw_if.h"
#include <stdlib.h> // Pro funkci rand()
#include <stdbool.h>
#include "flash_logger.h"

/* Global variables ----------------------------------------------------------*/
int volatile FrameOnGoing = FALSE;
extern MAC_associateInd_t g_MAC_associateInd;
MAC_dataInd_t      g_DataInd;

extern uint8_t BeaconTimerId; // Náš časovač z druhého souboru
extern volatile bool can_send_beacon;
extern RTC_HandleTypeDef hrtc;
uint32_t Get_Calendar_Seconds_Since_2000(RTC_DateTypeDef *sDate, RTC_TimeTypeDef *sTime);

// -----------------------------------------------------------------------------
// FUNKCE PRO NASTARTOVÁNI ČASOVAČE S NÁHODNÝM ROZPTYLEM (JITTER)
// -----------------------------------------------------------------------------
void APP_MAC_RestartBeaconTimer(void)
{
  // Vygenerujeme nahodny cas 18 az 22 milisekund pro zabraneni kolizim
  uint32_t jitter_ms = 18 + (rand() % 5);

  // Prepocet z milisekund na interni ticky (CFG_TS_TICK_VAL je vetsinou z app_conf.h)
  HW_TS_Start(BeaconTimerId, (uint32_t)(jitter_ms * 1000 / CFG_TS_TICK_VAL));
}

// =============================================================================
// MAC CALLBACKS
// =============================================================================

MAC_Status_t APP_MAC_mlmeResetCnfCb( const  MAC_resetCnf_t * pResetCnf )
{
  UTIL_SEQ_SetEvt(EVENT_DEVICE_RESET_CNF);
  return MAC_SUCCESS;
}

MAC_Status_t APP_MAC_mlmeSetCnfCb( const  MAC_setCnf_t * pSetCnf )
{
  UTIL_SEQ_SetEvt(EVENT_SET_CNF);
  return MAC_SUCCESS;
}

MAC_Status_t APP_MAC_mlmeStartCnfCb( const  MAC_startCnf_t * pStartCnf )
{
  UTIL_SEQ_SetEvt(EVENT_DEVICE_STARTED_CNF);
  return MAC_SUCCESS; // Puvodne tu bylo NOT IMPLEMENTED
}

// -----------------------------------------------------------------------------
// ZPRACOVÁNÍ PŘIJATÝCH DAT (Zde chytáme ty 3 byty od závodníka/konfigurátora)
// -----------------------------------------------------------------------------
void APP_MAC_ReceiveData(void)
{
	// Pokud Kontrola chytí 3bajtový paket (Odpověď od Závodníka)
	if (g_DataInd.msdu_length == 3)
	{
		uint8_t *payload = g_DataInd.msduPtr;

		// 1. Zpětná dešifrace prvního bajtu (Typ a ID)
		uint8_t device_type = (payload[0] >> 6) & 0x03;
		uint32_t received_id_or_hash = ((payload[0] & 0x3F) << 16) | (payload[1] << 8) | payload[2];

		if (device_type == 0x00)
		{
			// =================================================================
			// A) STANDARDNÍ ZÁVODNÍK -> ZÁPIS DO PAMĚTI
			// =================================================================
			APP_DBG(">>> ZAVODNIK ORAZIL! ID: %lu", received_id_or_hash);

		// Získání aktuálního času pro záznam
		RTC_TimeTypeDef sTime = {0};
		RTC_DateTypeDef sDate = {0};
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		uint32_t ssr = RTC->SSR;
		uint32_t prer_s = RTC->PRER & RTC_PRER_PREDIV_S;
		uint32_t prer_a = (RTC->PRER & RTC_PRER_PREDIV_A) >> 16;
		if (prer_s == 0) prer_s = 255;
		if (prer_a == 0) prer_a = 127;

		uint32_t ssr_freq = 32768 / (prer_a + 1);
		uint32_t seconds_per_tick = (prer_s + 1) / ssr_freq;
		if (seconds_per_tick == 0) seconds_per_tick = 1;

		uint32_t ms_within_tick = ((prer_s - ssr) * 1000) / ssr_freq;
		uint8_t sub_sec = (ms_within_tick % 1000) / 100; // Desetiny vteřiny (0-9)

		uint32_t cal_seconds = Get_Calendar_Seconds_Since_2000(&sDate, &sTime);
		uint32_t real_seconds_since_2000 = (cal_seconds * seconds_per_tick) + (ms_within_tick / 1000);
		uint32_t unix_time = 946684800 + real_seconds_since_2000;

		// Zápis do kruhového bufferu s Erase-Ahead logikou
		Logger_SavePunch_Kontrola(payload, sub_sec, unix_time);
		}
		else if (device_type == 0x01)
		{
			// =================================================================
			// B) KONFIGURAČNÍ JEDNOTKA -> PŘEPNUTÍ DO BLE
			// =================================================================
			APP_DBG(">>> DETEKOVAN KONFIGURATOR! Hash ze vzduchu: %lu", received_id_or_hash);

			// Přečteme si očekávaný hash z Flash paměti (maskujeme na 22 bitů pro jistotu)
			uint32_t expected_hash = DEVICE_CONFIG->hash_device & 0x3FFFFF;

			if (received_id_or_hash == expected_hash)
			{
				APP_DBG(">>> HASH SOUHLASI! INICIOVANO PREPNUTI DO BLE!");
				// Vyvoláme úkol pro ukončení MAC a spuštění BLE
				UTIL_SEQ_SetTask(1U << CFG_TASK_INIT_SWITCH_PROTOCOL, CFG_SCH_PRIO_0);
			}
			else
			{
				APP_DBG(">>> CHYBA BEZPECNOSTI: Neplatny konfigurační kód!");
			}
		}
	}
	BSP_LED_Toggle(LED_RED);
	FrameOnGoing = FALSE;
}

MAC_Status_t APP_MAC_mcpsDataIndCb( const  MAC_dataInd_t * pDataInd )
{
	// FIREWALL: Bezpečnostní kontrola délky (Bod 4)
	// Pokud je délka paketu nesmyslná (větší než náš standardní 128 bytový buffer
	// nebo úplně prázdná), paket okamžitě bez milosti zahodíme!
	if (pDataInd->msdu_length == 0 || pDataInd->msdu_length > 127)
	{
		return MAC_SUCCESS; // Tváříme se, že je vše OK, ale data ignorujeme
	}

	if (FrameOnGoing == FALSE)
	{
		FrameOnGoing = TRUE;
		memcpy(&g_DataInd, pDataInd, sizeof(MAC_dataInd_t));
		UTIL_SEQ_SetTask(1 << CFG_TASK_RECEIVE_DATA, CFG_SCH_PRIO_0);
	}
	return MAC_SUCCESS;
}

// -----------------------------------------------------------------------------
// CALLBACK: PAKET BYL ODESLÁN (ZDE RESTARTUJEME ČASOVAČ PRO DALŠÍ)
// -----------------------------------------------------------------------------
MAC_Status_t APP_MAC_mcpsDataCnfCb( const  MAC_dataCnf_t * pDataCnf )
{
	// Paket vyletěl z antény, otevíráme bránu pro další!
	can_send_beacon = true;
	UTIL_SEQ_SetEvt( EVENT_DATA_CNF );
	return MAC_SUCCESS;
}

// Zbytek callbacku, ktere Kontrola nepotrebuje vyrizovat...
MAC_Status_t APP_MAC_mlmeAssociateCnfCb(const MAC_associateCnf_t * pAssociateCnf) { return MAC_NOT_IMPLEMENTED_STATUS; }
MAC_Status_t APP_MAC_mlmeAssociateIndCb(const MAC_associateInd_t * pAssociateInd) { return MAC_NOT_IMPLEMENTED_STATUS; }
MAC_Status_t APP_MAC_mlmeBeaconNotifyIndCb(const MAC_beaconNotifyInd_t * pBeaconNotifyInd) { return MAC_NOT_IMPLEMENTED_STATUS; }
// ... (Zde muzes klidne nechat smazane nebo jako NOT_IMPLEMENTED vsechny ostatni callbacky z puvodniho souboru)
