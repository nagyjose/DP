#include "app_ffd_mac_802_15_4_process.h"
#include "app_conf.h"
#include "dbg_trace.h"
#include "shci.h"
#include "stm32_seq.h"
#include "app_ffd_mac_802_15_4.h"
#include "802_15_4_mac_sap.h"
#include "stm_logging.h"
#include "hw_if.h"
#include <stdbool.h>
#include "flash_logger.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_adc.h"

int volatile FrameOnGoing = FALSE;
extern MAC_associateInd_t g_MAC_associateInd;
MAC_dataInd_t      g_DataInd;
static uint8_t     rx_payload_safe[128]; // PŘIDÁNO: Naše vlastní bezpečné úložiště

extern uint8_t BuzzerTimerId; // Náš nový časovač pro blikání
volatile uint8_t blink_counter = 0; // Kolikrát má ještě bliknout

// --- NASTAVENÍ FILTRU A ZÁVODNÍKA ---
#define RSSI_THRESHOLD -40       // LUT: Prahová hodnota pro oražení (např. -60 dBm)
#define WINDOW_SIZE 5            // Velikost klouzavého okna (posledních 5 paketů)
#define HITS_REQUIRED 3          // Kolik z nich musí být nad prahem (např. 3 z 5)
#define COOLDOWN_MS 4000         // Cooldown (4 sekundy ochranná lhůta)

uint32_t my_runner_id = 123456;  // Unikátní ID tohoto čipu
uint8_t my_runner_type = 0x00;   // 00 = Normální závodník

// Paměť pro filtraci
static uint16_t scanning_control_id = 0xFFFF;
static uint8_t  history_mask = 0;

// Paměť pro cooldown
static uint16_t punched_control_id = 0xFFFF;
static uint32_t last_punch_unix_time = 0;

static uint16_t last_written_control_id = 0xFFFF; // Paměť posledního zápisu do Flash

// =============================================================================
// FUNKCE PRO MĚŘENÍ TEPLOTY (INTERNÍ ADC STM32)
// =============================================================================
extern ADC_HandleTypeDef hadc1; // Zpřístupníme ADC z main.c, pokud ho tam CubeMX generuje

// =============================================================================
// MANUÁLNÍ MĚŘENÍ TEPLOTY (BEZ CUBEMX)
// =============================================================================
static int8_t Get_MCU_Temperature(void)
{
	ADC_HandleTypeDef hadc1 = {0};
	ADC_ChannelConfTypeDef sConfig = {0};
	int8_t temp = 20; // Výchozí hodnota při selhání

	// Povolení hodin pro ADC
	__HAL_RCC_ADC_CLK_ENABLE();

	// Konfigurace ADC "na holém železe"
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.OversamplingMode = DISABLE;

	if (HAL_ADC_Init(&hadc1) == HAL_OK)
	{
		// Kalibrace pro přesné měření
		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

		// Nastavení kanálu pro vnitřní teplotní senzor
		sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; // Dostatečný čas pro probuzení senzoru
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
		{
			if (HAL_ADC_Start(&hadc1) == HAL_OK)
			{
				// Čekáme max 100 ms na změření
				if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
				{
					uint32_t raw_val = HAL_ADC_GetValue(&hadc1);
					// Standardní HAL makro, které použije tovární kalibrační hodnoty z paměti
					temp = (int8_t)__LL_ADC_CALC_TEMPERATURE(3300, raw_val, LL_ADC_RESOLUTION_12B);
				}
				HAL_ADC_Stop(&hadc1);
			}
		}
		HAL_ADC_DeInit(&hadc1);
	}

	// Vypneme hodiny pro úsporu baterie
	__HAL_RCC_ADC_CLK_DISABLE();

	return temp;
}


// =============================================================================
// TASK PRO BLIKÁNÍ LED (A ČASEM BZUČÁKU)
// =============================================================================
void APP_MAC_BuzzerTask(void)
{
	if (blink_counter > 0)
	{
		BSP_LED_Toggle(LED_GREEN); // Přepne stav LED (blik)
		blink_counter--;

		// Znovu se spustí za 100 ms (CFG_TS_TICK_VAL bývá definováno v app_conf)
		HW_TS_Start(BuzzerTimerId, (uint32_t)(100 * 1000 / CFG_TS_TICK_VAL));
	}
	else
	{
		BSP_LED_Off(LED_GREEN); // Dokončeno
	}
}

// =============================================================================
// ODESLÁNÍ 3BYTE ODPOVĚDI KONTROLE
// =============================================================================
static void Send_Punch_Response(void)
{
	static uint8_t msduHandle = 0;
	static uint8_t payload[3];

	// Získáme data aktuálního Závodníka PŘÍMO z Konfigurační nulté stránky
	uint8_t  my_type = DEVICE_CONFIG->comp_device_type;
	uint32_t my_id   = DEVICE_CONFIG->comp_device_id;

	// Byte 0: 2 bity TYP (posunuto o 6) | 6 nejvyšších bitů z ID
	payload[0] = (uint8_t)((my_type << 6) | ((my_id >> 16) & 0x3F));
	// Byte 1: Prostředních 8 bitů z ID
	payload[1] = (uint8_t)((my_id >> 8) & 0xFF);
	// Byte 2: Nejnižších 8 bitů z ID
	payload[2] = (uint8_t)(my_id & 0xFF);

	MAC_dataReq_t dataReq;
	memset(&dataReq, 0, sizeof(MAC_dataReq_t));
	dataReq.src_addr_mode = g_SHORT_ADDR_MODE_c;
	dataReq.dst_addr_mode = g_SHORT_ADDR_MODE_c;

	uint16_t panId = 0x1AAA;
	memcpy(dataReq.a_dst_PAN_id, (uint8_t*)&panId, 2);

	uint16_t destAddr = 0xFFFF; // Broadcast
	memcpy(&dataReq.dst_address, (uint8_t*)&destAddr, 2);

	dataReq.msdu_length = 3;
	dataReq.msduPtr = payload;
	dataReq.msdu_handle = msduHandle++;
	dataReq.ack_Tx = 0x00;
	dataReq.GTS_Tx = 0x00;
	dataReq.indirect_Tx = 0x00;

	MAC_MCPSDataReq(&dataReq);
}

// =============================================================================
// ZPRACOVÁNÍ PŘIJATÝCH DAT (MOZEK ZÁVODNÍKA)
// =============================================================================
// =============================================================================
// ZPRACOVÁNÍ PŘIJATÝCH DAT (MOZEK ZÁVODNÍKA)
// =============================================================================
void APP_MAC_ReceiveData(void)
{
	if (g_DataInd.msdu_length == 6)
	{
		uint8_t *payload = g_DataInd.msduPtr;
		int8_t rssi = g_DataInd.rssi;

		uint16_t control_id = (payload[0] << 4) | (payload[1] >> 4);
		uint8_t sub_seconds = payload[1] & 0x0F;
		uint32_t unix_time = ((uint32_t)payload[2] << 24) | ((uint32_t)payload[3] << 16) |
												 ((uint32_t)payload[4] << 8)  |  (uint32_t)payload[5];

		// Ochrana proti vícenásobnému oražení - tvůj 4s cooldown
		if (control_id == punched_control_id && (unix_time - last_punch_unix_time) < 4)
		{
			FrameOnGoing = FALSE;
			return;
		}

		// Filtrace - Klouzavé okno
		if (control_id != scanning_control_id) {
			scanning_control_id = control_id;
			history_mask = 0;
		}

		history_mask <<= 1;
		if (rssi >= RSSI_THRESHOLD) { history_mask |= 1; }

		uint8_t valid_hits = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			if (history_mask & (1 << i)) valid_hits++;
		}

		if (valid_hits >= HITS_REQUIRED)
		{
			char control_type_str[16];
			if (control_id == CLEAR_CONTROL_ID) {
				strcpy(control_type_str, "CLEAR");
			} else if (control_id <= 30) {
				strcpy(control_type_str, "SPECIFICKA");
			} else {
				strcpy(control_type_str, "STANDARDNI");
			}

			APP_DBG(">>> %s KONTROLA ORAZENA! | ID: %d | Cas: %lu.%d s | Sila: %d dBm",
							control_type_str, control_id, unix_time, sub_seconds, rssi);

			// Nastavení cooldownu
			punched_control_id = control_id;
			last_punch_unix_time = unix_time;
			history_mask = 0;

			// Odpovíme 3bajtovým paketem
			Send_Punch_Response();

			// =================================================================
			// ANTI-SPAM LOGIKA & ZÁPIS DO FLASH
			// =================================================================
			// Zjistíme, jestli to je typ kontroly podléhající Anti-Spamu (Start, Check, Standardní)
			bool is_standard_control = (control_id >= 31 && control_id <= 4095) ||
																 (control_id == CHECK_CONTROL_ID) ||
																 (control_id == START_CONTROL_ID);

			// Pokud to je standardní kontrola a její ID se shoduje s naposledy zapsaným, je to Spam
			bool is_spam = is_standard_control && (control_id == last_written_control_id);

			if (!is_spam)
			{
				if (control_id == CLEAR_CONTROL_ID) {
					Logger_NewRace(payload); // <--- TADY JE TA ZMĚNA (Předáváme přímo payload)
					last_written_control_id = control_id;
				} else {
					uint8_t abs_rssi = (uint8_t)(-rssi);
					int8_t teplota = Get_MCU_Temperature();
					Logger_SavePunch(payload, abs_rssi, teplota);
					last_written_control_id = control_id;
				}
			}
			else
			{
				APP_DBG("ANTI-SPAM: Kontrola %d jiz zapsana, pouze signalizuji!", control_id);
			}

			// Vizuální a zvuková signalizace proběhne VŽDY (i při Anti-Spamu)
			if (blink_counter == 0)
			{
				blink_counter = 40;
				HW_TS_Start(BuzzerTimerId, (uint32_t)(100 * 1000 / CFG_TS_TICK_VAL));
			}
		}
		else
		{
			APP_DBG("Skenuji Kontrolu %d (Hity: %d/%d, RSSI: %d dBm)", control_id, valid_hits, HITS_REQUIRED, rssi);
		}
	}

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

		// KRITICKÁ OPRAVA: Okopírujeme fyzické byty, než nám je rádio smaže pod rukama!
		if (pDataInd->msdu_length <= 128) {
			memcpy(rx_payload_safe, pDataInd->msduPtr, pDataInd->msdu_length);
			g_DataInd.msduPtr = rx_payload_safe; // Přesměrujeme ukazatel do bezpečí
		}

		UTIL_SEQ_SetTask(1 << CFG_TASK_RECEIVE_DATA, CFG_SCH_PRIO_0);
	}
	return MAC_SUCCESS;
}

// -----------------------------------------------------------------------------
// CALLBACK: PAKET ÚSPĚŠNĚ ODESLÁN (ZÁVODNÍK)
// -----------------------------------------------------------------------------
MAC_Status_t APP_MAC_mcpsDataCnfCb( const  MAC_dataCnf_t * pDataCnf )
{
	return MAC_SUCCESS;
}

// ... ostatní callbacky (Reset, Set, Start) zůstávají stejné jako dříve ...
MAC_Status_t APP_MAC_mlmeResetCnfCb( const  MAC_resetCnf_t * pResetCnf ) { UTIL_SEQ_SetEvt(EVENT_DEVICE_RESET_CNF); return MAC_SUCCESS; }
MAC_Status_t APP_MAC_mlmeSetCnfCb( const  MAC_setCnf_t * pSetCnf ) { UTIL_SEQ_SetEvt(EVENT_SET_CNF); return MAC_SUCCESS; }
MAC_Status_t APP_MAC_mlmeStartCnfCb( const  MAC_startCnf_t * pStartCnf ) { UTIL_SEQ_SetEvt(EVENT_DEVICE_STARTED_CNF); return MAC_SUCCESS; }
