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
#include "app_nbiot.h"

volatile BeaconState_t current_state = STATE_IDLE_MAC;

/* Global variables ----------------------------------------------------------*/
int volatile FrameOnGoing = FALSE;
extern MAC_associateInd_t g_MAC_associateInd;
MAC_dataInd_t      g_DataInd;

extern uint8_t BeaconTimerId; // Náš časovač z druhého souboru
extern volatile bool can_send_beacon;
extern RTC_HandleTypeDef hrtc;
uint32_t Get_Calendar_Seconds_Since_2000(RTC_DateTypeDef *sDate, RTC_TimeTypeDef *sTime);

extern uint8_t BuzzerTimerId; // Náš nový časovač pro blikání
volatile uint8_t blink_counter = 0; // Kolikrát má ještě bliknout

extern void APP_BLE_Stop(void);

// Musíme ten payload bezpečně vykopírovat ven dřív, než uvolníme koprocesor.
static uint8_t rx_payload_safe_kontrola[128];

// =============================================================================
// HARDWARE DRIVER: BZUČÁK (PWM) A LED
// =============================================================================
TIM_HandleTypeDef htim16; // Náš hardwarový časovač pro bzučák

// Makra pro spuštění a zastavení hardwarového PWM
#define BUZZER_ON()     HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1)
#define BUZZER_OFF()    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1)

// Perioda jednoho tiku signalizace v milisekundách
#define SIGNAL_PERIOD_MS 100

static bool is_signal_active = false; // Pomocná proměnná pro střídání stavu

// -----------------------------------------------------------------------------
// MANUÁLNÍ INICIALIZACE PWM (Bez CubeMX)
// -----------------------------------------------------------------------------
void Buzzer_PWM_Init(void)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// 1. Povolení hodin pro Timer 16 a Port B
	__HAL_RCC_TIM16_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// 2. Nastavení pinu PB8 pro PWM výstup
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternativní funkce (Push-Pull)
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	// KRITICKÉ PRO EMC: Low Speed zamezí ostrým hranám a vysokofrekvenčnímu rušení!
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	// Na STM32WB55 je TIM16_CH1 mapován na PB8 přes Alternate Function 14 (AF14)
	GPIO_InitStruct.Alternate = GPIO_AF14_TIM16;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// 3. Konfigurace samotného časovače na 4 kHz
	// Systémové hodiny jsou 64 MHz.
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 64 - 1;       // Zpomalí z 64 MHz na 1 MHz (1 tik = 1 mikrosekunda)
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 250 - 1;         // 1 MHz / 250 = 4000 Hz (Tedy naše vysněné 4 kHz)
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim16) != HAL_OK) {
		// Inicializace selhala (můžeme přidat Error Handler)
	}

	// 4. Konfigurace Kanálu 1 na 50% střídu (Duty Cycle)
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 125;                // Přesná polovina z 250 (Period) = 50 % střída
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		// Konfigurace kanálu selhala
	}
}

// -----------------------------------------------------------------------------
// UNIVERZÁLNÍ API PRO SPUŠTĚNÍ SIGNALIZACE
// -----------------------------------------------------------------------------
void System_Signalize_Start(uint8_t seconds)
{
	// Procesor si sám spočítá, kolikrát musí změnit stav (tiknout)
	uint8_t ticks_required = (seconds * 1000) / SIGNAL_PERIOD_MS;

	// Pokud signalizace zrovna neběží, nahodíme ji okamžitě
	if (blink_counter == 0) {
		blink_counter = ticks_required;
		is_signal_active = false; // Začneme vždy rozsvícením/pípnutím
		UTIL_SEQ_SetTask(1 << CFG_TASK_BUZZER, CFG_SCH_PRIO_0);
	} else {
		// Pokud už běží, jen jí vnutíme nový čas (nepřičítáme, pouze přepisujeme)
		blink_counter = ticks_required;
	}
}

// -----------------------------------------------------------------------------
// ASYNCHRONNÍ TASK PRO BLIKÁNÍ A PÍPÁNÍ
// -----------------------------------------------------------------------------
void APP_MAC_BuzzerTask(void)
{
	if (blink_counter > 0)
	{
		is_signal_active = !is_signal_active; // Přepnutí stavu (ON -> OFF -> ON)

		if (is_signal_active) {
			BSP_LED_On(LED_GREEN);
			if (DEVICE_CONFIG->buzzer_onoff == 1) BUZZER_ON();
		} else {
			BSP_LED_Off(LED_GREEN);
			BUZZER_OFF();
		}

		blink_counter--;

		// Znovu spustit task za 500 ms
		HW_TS_Start(BuzzerTimerId, (uint32_t)(SIGNAL_PERIOD_MS * 1000 / CFG_TS_TICK_VAL));
	}
	else
	{
		// Konec signalizace - vše bezpečně vypneme
		BSP_LED_Off(LED_GREEN);
		BUZZER_OFF();
		is_signal_active = false;
	}
}

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

// =============================================================================
// HARDWARE DRIVER: UNIFIKOVANÉ MĚŘENÍ ADC (TEPLOTA + BATERIE)
// =============================================================================
void Get_ADC_Measurements(int8_t *out_temp, uint16_t *out_batt_mv)
{
	ADC_HandleTypeDef hadc1 = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	// Výchozí chybové hodnoty
	*out_temp = -128;
	*out_batt_mv = 0;

	// 1. Povolení hodin
	__HAL_RCC_ADC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE(); // Port pro baterii (uprav dle tvé desky)

	// 2. Konfigurace pinu pro baterii (PA3 - Channel 4)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 3. Základní konfigurace ADC
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
		// Kalibrace se provede jen JEDNOU pro obě měření
		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

		// Společné nastavení kanálu
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; // Maximální čas na nabití
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;

		// --- MĚŘENÍ 1: INTERNÍ TEPLOTA ---
		sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
		{
			HAL_Delay(1); // Čas na ustálení interního senzoru
			if (HAL_ADC_Start(&hadc1) == HAL_OK)
			{
				if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
					uint32_t raw_val = HAL_ADC_GetValue(&hadc1);
					int8_t temp_raw = (int8_t)__LL_ADC_CALC_TEMPERATURE(3300, raw_val, LL_ADC_RESOLUTION_12B);
					*out_temp = temp_raw - 2; // Aplikace kalibračního offsetu
				}
				HAL_ADC_Stop(&hadc1);
			}
		}

		// --- MĚŘENÍ 2: NAPĚTÍ BATERIE ---
		sConfig.Channel = ADC_CHANNEL_4; // PA3 = CH4
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
		{
			HAL_Delay(1); // Čas na přebití kondenzátoru z nového pinu
			if (HAL_ADC_Start(&hadc1) == HAL_OK)
			{
				if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
					uint32_t raw_val = HAL_ADC_GetValue(&hadc1);
					*out_batt_mv = (raw_val * 3300) / 4095; // (x2 pokud máš dělič)
				}
				HAL_ADC_Stop(&hadc1);
			}
		}

		HAL_ADC_DeInit(&hadc1);
	}

	__HAL_RCC_ADC_CLK_DISABLE();
}

// -----------------------------------------------------------------------------
// ZPRACOVÁNÍ PŘIJATÝCH DAT (Zde chytáme ty 3 byty od závodníka/konfigurátora)
// -----------------------------------------------------------------------------
void APP_MAC_ReceiveData(void)
{
	// Pokud Kontrola chytí 3bajtový paket (Odpověď od Závodníka)
	if (g_DataInd.msdu_length == 3)
	{
		// =====================================================================
		// STAVOVÝ AUTOMAT: PROBUZENÍ Z VÍKENDOVÉHO REŽIMU
		// =====================================================================
		if (current_state == STATE_IDLE_MAC) {
			current_state = STATE_ACTIVE_MAC;
			APP_DBG(">>> AUTOMAT: Kontrola probuzena do ZAVODNIHO rezimu!");
		}

		uint8_t *payload = g_DataInd.msduPtr;

		// 1. Zpětná dešifrace prvního bajtu (Typ a ID)
		uint8_t device_type = (payload[0] >> 6) & 0x03;
		uint32_t received_id_or_hash = ((payload[0] & 0x3F) << 16) | (payload[1] << 8) | payload[2];

		if (device_type == 0x00)
		{
			// =================================================================
			// ANTI-SPAM FILTR (Ochrana proti Burst přenosu od Závodníka)
			// =================================================================
			static uint32_t last_saved_id = 0xFFFFFFFF;
			static uint32_t last_saved_tick = 0;

			// Pokud je to ten samý závodník a neuběhla ani 1 vteřina (1000 ms), zahoď to!
			if (received_id_or_hash == last_saved_id && (HAL_GetTick() - last_saved_tick < 1000)) {
				APP_DBG(">>> KONTROLA ANTI-SPAM: Ignoruji burst duplikat od ID: %lu", received_id_or_hash);
				FrameOnGoing = FALSE;
				return; // Ukončíme zpracování tohoto paketu
			}

			// Není to spam, uložíme si jeho ID a čas pro příště
			last_saved_id = received_id_or_hash;
			last_saved_tick = HAL_GetTick();

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

			// Vhodíme 3 bajty závodníka a čas do FIFO fronty!
			NBIOT_FIFO_Push(payload, unix_time);

			// =================================================================
			// ASYNCHRONNÍ PÍPNUTÍ A BLIKNUTÍ (Čisté využití hotového API)
			// =================================================================
			System_Signalize_Start(4); // Pípá a bliká po dobu 1 vteřiny

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

				// AUTOMAT: Přejít do módu Konfigurace
				current_state = STATE_BLE_CONFIG;

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

		// --- KRITICKÁ OPRAVA: BEZPEČNÁ ZÁLOHA PAYLOADU ---
		memcpy(rx_payload_safe_kontrola, pDataInd->msduPtr, pDataInd->msdu_length);
		g_DataInd.msduPtr = rx_payload_safe_kontrola; // Přesměrování ukazatele
		// -------------------------------------------------

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

// =============================================================================
// OBSLUHA MAGNETU (TLAČÍTKO SW2) - PŘECHOD DO ZÁVODNÍHO REŽIMU
// =============================================================================
void APP_MAC_Magnet_Action(void)
{
	static uint32_t last_trigger_time = 0;
	if (HAL_GetTick() - last_trigger_time < 500) return;
	last_trigger_time = HAL_GetTick();

	// Zvedáme stav POUZE pokud je Kontrola v IDLE (Standby)
	if (current_state == STATE_IDLE_MAC) {
		current_state = STATE_ACTIVE_MAC;
		APP_DBG(">>> MAGNET: Kontrola probuzena do ZAVODNIHO rezimu! <<<");

		// ---------------------------------------------------------
		// 1. ZCELA USMRTIT BLE STACK (Uvolní hardwarový arbitr antény)
		// ---------------------------------------------------------
		APP_BLE_Stop();

		// ---------------------------------------------------------
		// 2. PROBUZENÍ PŘIJÍMAČE MAC VRSTVY (Aby Kontrola slyšela Závodníka)
		// ---------------------------------------------------------
		MAC_setReq_t SetReq;
		memset(&SetReq, 0x00, sizeof(MAC_setReq_t));
		SetReq.PIB_attribute = g_MAC_RX_ON_WHEN_IDLE_c;
		uint8_t PIB_Value = g_TRUE; // ZAPNUTO!
		SetReq.PIB_attribute_valuePtr = &PIB_Value;
		MAC_MLMESetReq( &SetReq );
		UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF ); // Zde je WaitEvt bezpečný (standardní ST parametr)

		// ---------------------------------------------------------
		// 3. VYNUCENÍ SPRÁVNÉHO KANÁLU (Jistota po resetu)
		// ---------------------------------------------------------
		memset(&SetReq, 0x00, sizeof(MAC_setReq_t));
		SetReq.PIB_attribute = 0x21; // OPRAVA: 0x21 je standardní IEEE ID pro macLogicalChannel
		PIB_Value = 26; // DEMO_CHANNEL
		SetReq.PIB_attribute_valuePtr = &PIB_Value;
		MAC_MLMESetReq( &SetReq );
		UTIL_SEQ_WaitEvt( 1U << CFG_EVT_SET_CNF );

		APP_DBG(">>> SYSTEM: MAC radio pripraveno (RX ON, Kanál 26, BLE mrtve).");

		// Zablikáme a zapípáme jako potvrzení startu
		extern volatile uint8_t blink_counter;
		blink_counter = 2;
		UTIL_SEQ_SetTask(1 << CFG_TASK_BUZZER, CFG_SCH_PRIO_0);
	} else {
		APP_DBG(">>> MAGNET: Ignorovano. Kontrola uz je v ZAVODNIM rezimu! <<<");
	}
}


