/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    p2p_server_app.c
 * @author  MCD Application Team
 * @brief   peer to peer Server Application
  ******************************************************************************
  */
/* USER CODE END Header */

#include "p2p_server_app.h"
#include "dbg_trace.h"
#include "app_conf.h"
#include "ble.h"
#include "hci_tl.h"
#include "stm32_seq.h"
#include "flash_logger.h"
#include <stdbool.h>
#include "app_ffd_mac_802_15_4_process.h"

extern void System_Signalize_Start(uint8_t seconds);
extern void Get_ADC_Measurements(int8_t *out_temp, uint16_t *out_batt_mv);
extern void NBIOT_Force_Sleep(void);

// --- PŘIDÁNO: Dopředná deklarace naší nové funkce ---
void System_Send_ACK(uint8_t *payload, uint8_t length, uint8_t source);

// --- NAŠE UNIKÁTNÍ UUID (128-bit) ---
#define COPY_SERVICE_UUID(uuid_struct) { \
  (uuid_struct)[0]=0x19; (uuid_struct)[1]=0xed; (uuid_struct)[2]=0x82; (uuid_struct)[3]=0xae; \
  (uuid_struct)[4]=0xed; (uuid_struct)[5]=0x21; (uuid_struct)[6]=0x4c; (uuid_struct)[7]=0x9d; \
  (uuid_struct)[8]=0x41; (uuid_struct)[9]=0x45; (uuid_struct)[10]=0x22; (uuid_struct)[11]=0x8e; \
  (uuid_struct)[12]=0x40; (uuid_struct)[13]=0xFE; (uuid_struct)[14]=0x00; (uuid_struct)[15]=0x00; }

#define COPY_RX_UUID(uuid_struct) { (uuid_struct)[12]=0x41; (uuid_struct)[13]=0xFE; }
#define COPY_TX_UUID(uuid_struct) { (uuid_struct)[12]=0x42; (uuid_struct)[13]=0xFE; }

static uint16_t OrienteeringServiceHdle;
static uint16_t RxCharHdle;
static uint16_t TxCharHdle;
static uint16_t current_conn_handle = 0xFFFF; // Handle aktuálního spojení
static bool sleep_pending = false;            // Čekáme na slušné odpojení?

static uint8_t pending_disconnect_action = 0; // 0 = Nic, 1 = SLEEP, 2 = POWER_OFF
static uint8_t DisconnectTimerId;

// Callback časovače pouze probudí finální Task
static void DisconnectTimer_Callback(void) {
    UTIL_SEQ_SetTask(1 << CFG_TASK_EXECUTE_DISCONNECT, CFG_SCH_PRIO_0);
}
void Execute_Disconnect_Task(void); // <--- Přidej tento řádek nahoru k definicím

// =============================================================================
// SEZNAM HLAVNÍCH BLE PŘÍKAZŮ
// =============================================================================
typedef enum {
    // --- Bezpečnost a Relace ---
    CMD_UNLOCK              = 0x05,
    CMD_LOCK                = 0x06,
    CMD_VERIFY_HASH         = 0x07,

    // --- Čtení dat ---
    CMD_READ_CONFIG         = 0x10,
		CMD_GET_STATUS          = 0x11, // Rychlý status (Bez logů)
		CMD_GET_BATTERY         = 0x14, // <--- PŘIDÁNO: Samostatný stav baterie
		CMD_DOWNLOAD_ALL        = 0x21, // Vše (od nejstaršího po nejnovější)
		CMD_DOWNLOAD_FROM_TIME  = 0x23, // PŘIDÁNO: Od zadaného UNIX času
		CMD_DOWNLOAD_LAST_N     = 0x24, // PŘIDÁNO: Posledních N záznamů

    // --- Zápis do Konfigurace (Staging) ---
    CMD_STAGE_PARAM         = 0x12,
    CMD_COMMIT_CONFIG       = 0x13,

    // --- Nástroje ---
    CMD_IDENTIFY            = 0x40,
		CMD_SYNC_TIME						= 0x30,
		CMD_SLEEP               = 0x51,  // PŘIDÁNO: Příkaz pro návrat do MAC
		CMD_POWER_OFF           = 0x52,  // PŘIDÁNO: Tvrdé vypnutí do STORAGE

    // --- Mazání a Resety ---
    CMD_RESET_CONFIG        = 0x97,
    CMD_RESET_ALL           = 0x98,
    CMD_FORMAT_HISTORY      = 0x99
} BLE_Command_t;

// =============================================================================
// SEZNAM PARAMETRŮ PRO ÚPRAVU KONFIGURACE (Pro příkaz 0x12)
// =============================================================================
typedef enum {
    // --- Základní informace ---
    PARAM_BLE_DEVICE_NAME   = 0x01,
    PARAM_HASH_DEVICE       = 0x02,

    // --- Vysílané informace (Maják) ---
    PARAM_STAT_DEVICE_TYPE  = 0x03, // 12bit ID Kontroly (CLEAR, START, atd.)
    PARAM_REQ_RSSI          = 0x04,
    PARAM_EVENT_NATION      = 0x05,
    PARAM_EVENT_ID          = 0x06,

    // --- Nastavovací parametry ---
    PARAM_BEACON_PERIOD_MS  = 0x07,
    PARAM_BEACON_PERIOD_LP  = 0x08,
    PARAM_BUZZER_ONOFF      = 0x09,
    PARAM_TX_POWER          = 0x0A,
    PARAM_BAT_ALERT_THRESH  = 0x0B,

    // --- Informace o vlastníkovi (Oddíl) ---
    PARAM_TEAM_OWNER        = 0x0C,
    PARAM_TEAM_SHORTCUT     = 0x0D,
    PARAM_TEAM_NATION       = 0x0E,
    PARAM_TEAM_LEADER       = 0x0F,
    PARAM_TEAM_EMAIL        = 0x10,
    PARAM_TEAM_PHONE        = 0x11,
    PARAM_TEAM_ADDRESS      = 0x12,
    PARAM_TEAM_BANK         = 0x13,
    PARAM_TEAM_IBAN         = 0x14,
    PARAM_TEAM_BIC          = 0x15,
    PARAM_TEAM_ORISID       = 0x16,
    PARAM_TEAM_OTHER_INFO   = 0x17,

		// --- NB-IOT Parametry ---
		PARAM_NBIOT_APN         = 0x18,
		PARAM_NBIOT_IP          = 0x19,
		PARAM_NBIOT_PORT        = 0x1A
} Config_Param_t;

// Zpřístupnění hlavního RTC ovladače z main.c
extern RTC_HandleTypeDef hrtc;

// -----------------------------------------------------------------------------
// PŘEVODNÍK: UNIX TIMESTAMP -> STM32 RTC KALENDÁŘ
// -----------------------------------------------------------------------------
static void Convert_UnixToRTC(uint32_t unix_time, RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
    // Výpočet času v rámci dne
    uint32_t days = unix_time / 86400;
    uint32_t seconds_in_day = unix_time % 86400;

    time->Seconds = seconds_in_day % 60;
    time->Minutes = (seconds_in_day / 60) % 60;
    time->Hours = seconds_in_day / 3600;

    // Matematická magie pro převod dnů na kalendář (Shift z roku 1970)
    uint32_t z = days + 719468;
    uint32_t era = (z >= 0 ? z : z - 146096) / 146097;
    uint32_t doe = (z - era * 146097);
    uint32_t yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;
    uint32_t y = yoe + era * 400;
    uint32_t doy = doe - (365*yoe + yoe/4 - yoe/100);
    uint32_t mp = (5*doy + 2)/153;
    uint32_t d = doy - (153*mp+2)/5 + 1;
    uint32_t m = mp + (mp < 10 ? 3 : -9);
    uint32_t year = y + (m <= 2);

    // Uložení do STM32 struktur
    date->Date = d;
    date->Month = m;
    date->Year = year - 2000; // STM32 používá jen roky 0-99 (vztaženo k roku 2000)

    // Výpočet dne v týdnu (1.1.1970 byl čtvrtek = 4)
    date->WeekDay = (days + 4) % 7;
    if (date->WeekDay == 0) date->WeekDay = 7; // STM32 počítá týden 1-7 (Pondělí - Neděle)
}

// -----------------------------------------------------------------------------
// PŘEVODNÍK: STM32 RTC KALENDÁŘ -> UNIX TIMESTAMP
// -----------------------------------------------------------------------------
static uint32_t Convert_RTCToUnix(RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
{
    uint16_t y = date->Year + 2000;
    uint8_t m = date->Month;

    if (m <= 2) {
        y -= 1;
        m += 12;
    }

    // Klasický výpočet dnů od 1. 1. 1970
    uint32_t days = (365 * (uint32_t)y) + (y / 4) - (y / 100) + (y / 400) + ((367 * (uint32_t)m - 362) / 12) + date->Date - 1 - 719468;

    return (days * 86400) + (time->Hours * 3600) + (time->Minutes * 60) + time->Seconds;
}

// =============================================================================
// STAVOVÉ PROMĚNNÉ PRO KRÁJEČ (CHUNKER)
// =============================================================================
static uint8_t *chunk_ptr = NULL;        // Ukazatel na to, co zrovna odesíláme
static uint32_t chunk_rem_len = 0;       // Kolik bajtů nám ještě zbývá odeslat
static uint8_t  chunk_active_cmd = 0;    // Jaký příkaz se zrovna vykonává

#define CHUNK_MAX_SIZE 20 // Maximální velikost jedné rány (BLE MTU to bezpečně snese)

// =============================================================================
// STAVOVÉ PROMĚNNÉ PRO RELACI (SESSION & STAGING)
// =============================================================================
static bool is_unlocked = false;           // Stav zámku (Odemčeno / Zamčeno)
static BeaconConfig_t staged_config;       // RAM kopie konfigurace KONTROLY pro úpravy      // RAM kopie konfigurace pro úpravy

// =============================================================================
// KRYPTOGRAFIE (Challenge-Response)
// =============================================================================
static uint32_t current_challenge = 0; // Paměť pro aktuálně vygenerovanou výzvu

// Murmur3 Avalanche Hash (Jednoduchá, ale velmi silná míchací funkce)
static uint32_t Generate_Response(uint32_t challenge, uint32_t pin) {
    uint32_t mix = challenge ^ pin; // Smícháme výzvu s tajným heslem
    mix ^= mix >> 16;
    mix *= 0x85ebca6b;
    mix ^= mix >> 13;
    mix *= 0xc2b2ae35;
    mix ^= mix >> 16;
    return mix; // Výsledný Hash (Response)
}

// =============================================================================
// FUNKCE KRÁJEČE (Volá se asynchronně přes Sequencer)
// =============================================================================
void BLE_Chunker_Task(void)
{
	if (chunk_rem_len == 0) return; // Není co posílat

	uint16_t send_len = (chunk_rem_len > CHUNK_MAX_SIZE) ? CHUNK_MAX_SIZE : chunk_rem_len;

	// OCHRANA PROTI PŘETEČENÍ: Nesmíme číst za hranicí naší vyhrazené Flash paměti!
	uint32_t flash_end = LOGGER_START_ADDR + (LOGGER_MAX_PAGES * LOGGER_PAGE_SIZE);
	if ((uint32_t)chunk_ptr + send_len > flash_end) {
		send_len = flash_end - (uint32_t)chunk_ptr; // Zkrátíme paket přesně po hranu
	}

	// Fyzický pokus o odeslání do fronty Bluetooth rádia
	tBleStatus ret = aci_gatt_update_char_value(OrienteeringServiceHdle, TxCharHdle, 0, send_len, chunk_ptr);

	if (ret == BLE_STATUS_SUCCESS)
	{
		chunk_ptr += send_len;
		chunk_rem_len -= send_len;

		// Pokud jsme dojeli ukazatelem přesně na konec paměti, přetočíme ho na začátek
		if ((uint32_t)chunk_ptr >= flash_end) {
			chunk_ptr = (uint8_t*)LOGGER_START_ADDR;
		}

		if (chunk_rem_len > 0) {
			UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
		} else {
			APP_DBG(">>> BLE CHUNKER: Prenos dokoncen! (CMD: 0x%02X)", chunk_active_cmd);
		}
	}
	else if (ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
	{
		// Fronta rádia je dočasně plná. Nevadí, jdeme spát a počkáme na přerušení (TX_POOL_AVAILABLE)
		// APP_DBG(">>> BLE CHUNKER: Fronta plna, cekam na uvolneni...");
	}
	else
	{
		APP_DBG(">>> BLE CHUNKER: Kriticka chyba odeslani! (Err: 0x%02X)", ret);
	}
}

// =============================================================================
// HARDWAROVÉ VYPNUTÍ DO STORAGE MÓDU
// =============================================================================
static void System_Enter_Storage_Mode(void)
{
	APP_DBG(">>> SYSTEM: Prechod do hlubokeho spanku (STORAGE) <<<");

	NBIOT_Force_Sleep();

	// 1. Zastavíme všechny procesy a rádio
	// (V tuto chvíli běží BLE, takže ho korektně vypneme)
	extern void APP_BLE_Stop(void);
	APP_BLE_Stop();

	// 2. Vyčistíme všechny staré Wake-Up vlajky
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	// 3. POVOLENÍ WAKE-UP PINU PRO MAGNET
	// STM32WB55 má 5 možných WKUP pinů (PA0, PC13, PC12, PA2, PC5).
	// ZDE SI MUSÍŠ ZVOLIT TEN, NA KTERÉM JE TVÁ HALLOVA SONDA!
	// (Příklad pro PA0 = WKUP1, reaguje na vzestupnou hranu / HIGH):
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);

	// Pokud sonda vrací v klidu HIGH a při přiložení magnetu spadne na LOW,
	// použij parametr PWR_WAKEUP_PIN1_LOW (dostupné dle revize HAL knihoven).

	// 4. Skok do Standby módu (Deska se vypne, RAM se smaže)
	// Probudí se jedině magnetem, čímž proběhne klasický reset a start od main()
	HAL_PWR_EnterSTANDBYMode();
}

// -----------------------------------------------------------------------------
// CALLBACK: PARSER - Zde zpracováváme příkazy z Mobilu a zprávy z Rádia
// -----------------------------------------------------------------------------
static SVCCTL_EvtAckStatus_t Tunnel_Event_Handler(void *pckt)
{
	// LOCKDOWN: Pokud se zrovna vypínáme, ignorujeme všechny nové požadavky od mobilu!
	if (pending_disconnect_action != 0) {
		return SVCCTL_EvtNotAck;
	}

	hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
	hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

	if (hci_pckt->type != HCI_EVENT_PKT_TYPE) return SVCCTL_EvtNotAck;

	if (event_pckt->evt == HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE)
	{
		evt_blecore_aci *blecore_evt = (evt_blecore_aci*)event_pckt->data;

		switch (blecore_evt->ecode)
		{
			// --- A) ZPRÁVA Z RÁDIA: MÁM MÍSTO, MŮŽEŠ VYSÍLAT ---
			case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
				if (chunk_rem_len > 0) {
					// Rádio odeslalo předchozí pakety do éteru a vyprázdnilo frontu. Probudíme Kráječ.
					UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
				}
				break;

			// --- B) ZPRÁVA Z MOBILU: UŽIVATEL POSLAL PŘÍKAZ ---
			case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
			{
				aci_gatt_attribute_modified_event_rp0 *mod = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;

				// Zápis do RX Charakteristiky (Ověření Handle)
				if (mod->Attr_Handle == (RxCharHdle + 1))
				{
					// Odtud data jednoduše "přepošleme" do našeho agnostického jádra!
					// Parametr source = 0 (Znamená, že volajícím je BLE)
					System_Execute_Command(mod->Attr_Data, mod->Attr_Data_Length, 0);
				}
				break;
			}
		}
	}
	return SVCCTL_EvtNotAck;
}

// =============================================================================
// AGNOSTICKÉ JÁDRO PRO ZPRACOVÁNÍ PŘÍKAZŮ (Voláno z BLE, NB-IoT i USB)
// source: 0 = BLE, 1 = NB-IoT, 2 = USB
// =============================================================================
void System_Execute_Command(uint8_t *payload_data, uint8_t payload_len, uint8_t source)
{
	if (payload_len == 0) return;

	BLE_Command_t cmd = (BLE_Command_t)payload_data[0];

	// Bezpečnostní pojistka: Kdyby ještě běžel starý přenos, natvrdo ho zrušíme
	chunk_rem_len = 0;

	// =========================================================
	// ROZCESTNÍK PŘÍKAZŮ (COMMAND DICTIONARY)
	// =========================================================
	switch(cmd)
	{
		case CMD_READ_CONFIG: // CMD_READ_CONFIG
			APP_DBG(">>> BLE CMD: READ CONFIG (0x10) - Odesilam %d bajtu", sizeof(BeaconConfig_t));
			chunk_ptr = (uint8_t*)DEVICE_CONFIG;
			chunk_rem_len = sizeof(BeaconConfig_t);
			chunk_active_cmd = cmd;
			UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
			break;

		// =====================================================
		// RYCHLÝ STATUS KONTROLY (0x11)
		// =====================================================
		case CMD_GET_STATUS:
			if (is_unlocked) {
				uint8_t status_payload[12]; // Zvětšeno na 12 bajtů
				int8_t temp_c;
				uint16_t bat_mv;
				Get_ADC_Measurements(&temp_c, &bat_mv);

				// Vyčtení RTC hodin (POZOR: Musí se číst Time a hned po něm Date!)
				RTC_TimeTypeDef sTime = {0};
				RTC_DateTypeDef sDate = {0};
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				uint32_t unix_now = Convert_RTCToUnix(&sDate, &sTime);

				status_payload[0] = 0x11; // Odpověď na příkaz

				// 1. ID Kontroly (16 bitů)
				status_payload[1] = (DEVICE_CONFIG->stat_device_type >> 8) & 0xFF;
				status_payload[2] = DEVICE_CONFIG->stat_device_type & 0xFF;

				// 2. Vysílací parametry (Perioda, Výkon, Bzučák)
				status_payload[3] = DEVICE_CONFIG->beacon_period_ms;
				status_payload[4] = DEVICE_CONFIG->tx_power;
				status_payload[5] = DEVICE_CONFIG->buzzer_onoff;

				// 3. Napětí baterie (16 bitů v milivoltech)
				status_payload[6] = (bat_mv >> 8) & 0xFF;
				status_payload[7] = bat_mv & 0xFF;

				// 4. PŘIDÁNO: Aktuální UNIX čas v Kontrole (32 bitů)
				status_payload[8] = (unix_now >> 24) & 0xFF;
				status_payload[9] = (unix_now >> 16) & 0xFF;
				status_payload[10] = (unix_now >> 8) & 0xFF;
				status_payload[11] = unix_now & 0xFF;

				APP_DBG(">>> BLE CMD: GET STATUS (0x11) - Baterie: %d mV, Cas: %lu", bat_mv, unix_now);
				System_Send_ACK(status_payload, 12, source);
			} else {
				uint8_t err_lock[4] = {cmd, 0xAA, 0x00, 0x00}; System_Send_ACK(err_lock, 4, source);
			}
			break;

		// =====================================================
		// SAMOSTATNÉ MĚŘENÍ BATERIE (0x14)
		// =====================================================
		case CMD_GET_BATTERY:
			if (is_unlocked) {
				uint8_t bat_payload[3];
				int8_t temp_c;
				uint16_t bat_mv;
				Get_ADC_Measurements(&temp_c, &bat_mv);

				bat_payload[0] = 0x14; // Odpověď na příkaz
				bat_payload[1] = (bat_mv >> 8) & 0xFF;
				bat_payload[2] = bat_mv & 0xFF;

				APP_DBG(">>> BLE CMD: GET BATTERY (0x14) - Napeti: %d mV", bat_mv);
				System_Send_ACK(bat_payload, 3, source);
			} else {
				uint8_t err_lock[4] = {cmd, 0xAA, 0x00, 0x00}; System_Send_ACK(err_lock, 4, source);
			}
			break;

		case CMD_DOWNLOAD_ALL:
		case CMD_DOWNLOAD_FROM_TIME:
		case CMD_DOWNLOAD_LAST_N:
		{
			uint32_t param = 0;

			// Pokud mobil k příkazu přibalil 4 bajty dat (N, nebo UNIX čas)
			if (payload_len >= 5) {
				param = ((uint32_t)payload_data[1] << 24) |
								((uint32_t)payload_data[2] << 16) |
								((uint32_t)payload_data[3] << 8)  |
								 (uint32_t)payload_data[4];
			}

			uint8_t *data_ptr = NULL;
			uint32_t data_len = 0;

			// Předáme požadavek paměťovému modulu (všimni si, že param je nyní uint32_t!)
			extern void Logger_GetDownloadData(uint8_t cmd, uint32_t param, uint8_t **start_ptr, uint32_t *len);
			Logger_GetDownloadData(cmd, param, &data_ptr, &data_len);

			if (data_len > 0 && data_ptr != NULL) {
				APP_DBG(">>> BLE CMD: Odesilam LOGY (0x%02X) - Delka: %lu bajtu", cmd, data_len);
				chunk_ptr = data_ptr;
				chunk_rem_len = data_len;
				chunk_active_cmd = cmd;
				UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
			} else {
				APP_DBG(">>> BLE CMD: Zadne logy k odeslani (nebo zadne nevyhovuji)!");
				uint8_t ack_empty[4] = {cmd, 0x00, 0x00, 0x00};
				System_Send_ACK(ack_empty, 4, source);
			}
			break;
		}

		case CMD_IDENTIFY: // CMD_IDENTIFY (Najdi můj čip)
			APP_DBG(">>> BLE CMD: IDENTIFY (0x40) - Zacinam signalizovat!");

			// 10 vteřin blikání a pípání
			System_Signalize_Start(10);

			// Odpovíme mobilu, že úkol běží
			uint8_t ack_id[4] = {0x40, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_id, 4, source);
			break;

		case CMD_SYNC_TIME: // CMD_SYNC_TIME (Seřízení RTC hodin Kontroly)
			if (is_unlocked) {
				// OPRAVA: Musíme si z paketu správně vytáhnout délku a data
				uint8_t time_payload_len = payload_len - 1; // Mínus 1 bajt za samotný příkaz
				uint8_t *time_payload = &payload_data[1];      // Data začínají na indexu 1

				if (time_payload_len == 4) {
					// 1. Složení 4 bajtů (MSB First) do uint32_t
					uint32_t unix_timestamp = ((uint32_t)time_payload[0] << 24) |
												((uint32_t)time_payload[1] << 16) |
												((uint32_t)time_payload[2] << 8)  |
												((uint32_t)time_payload[3]);

					APP_DBG(">>> BLE CMD: SYNC TIME (0x30) - Prijaty UNIX cas: %lu", unix_timestamp);

					// 2. Převod a uložení do struktur
					RTC_TimeTypeDef sTime = {0};
					RTC_DateTypeDef sDate = {0};
					Convert_UnixToRTC(unix_timestamp, &sTime, &sDate);

					// 3. Fyzický zápis do hardwaru STM32
					// (Zásadní je pořadí: Vždy SetTime a hned po něm SetDate, jinak čip zamrzne datum)
					HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

					APP_DBG(">>> RTC SERIZENO NA: 20%02d-%02d-%02d %02d:%02d:%02d",
							sDate.Year, sDate.Month, sDate.Date,
							sTime.Hours, sTime.Minutes, sTime.Seconds);

					// 4. Odeslání potvrzení (ACK) do mobilu
					uint8_t ack_time[4] = {0x30, 0x01, 0x00, 0x00};
					System_Send_ACK(ack_time, 4, source);

				} else {
					// Špatná délka dat - posíláme chybu
					uint8_t err_time[4] = {0x30, 0xEE, 0x00, 0x00};
					System_Send_ACK(err_time, 4, source);
				}
			} else {
				APP_DBG(">>> BLE SECURITY: Prikaz zamitnut (ZAMCENO)");
				uint8_t err_lock[4] = {cmd, 0xAA, 0x00, 0x00}; System_Send_ACK(err_lock, 4, source);
			}
			break;

		case CMD_SLEEP:
			APP_DBG(">>> BLE CMD: SLEEP (0x51) - Zacinam slusne odpojovani...");
			uint8_t ack_sleep[4] = {0x51, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_sleep, 4, source);

			sleep_pending = true;
			pending_disconnect_action = 1; // 1 = Zámek pro SLEEP

			// Necháme rádio dýchat a za 100 ms spustíme finální odpojení
			HW_TS_Start(DisconnectTimerId, (uint32_t)(100 * 1000 / CFG_TS_TICK_VAL));
			break;

		case CMD_POWER_OFF:
			if (!is_unlocked) {
				uint8_t err_lock[4] = {cmd, 0xAA, 0x00, 0x00};
				System_Send_ACK(err_lock, 4, source);
				break;
			}

			APP_DBG(">>> BLE CMD: POWER OFF (0x52) - Vypinam kontrolu...");
			uint8_t ack_off[4] = {0x52, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_off, 4, source);

			pending_disconnect_action = 2; // 2 = Zámek pro POWER_OFF

			// Za 100 ms zabijeme procesor
			HW_TS_Start(DisconnectTimerId, (uint32_t)(100 * 1000 / CFG_TS_TICK_VAL));
			break;

		// =====================================================
		// 1A. KROK 1: MOBIL ŽÁDÁ O VÝZVU (0x05)
		// Paket z mobilu: [0x05]
		// =====================================================
		case CMD_UNLOCK:
			// Vygenerujeme náhodné číslo (Využijeme ticky procesoru a UDN čipu)
			current_challenge = HAL_GetTick() ^ DEVICE_CONFIG->magic_word ^ LL_FLASH_GetUDN();

			// Pokud by náhodou vyšla nula (neplatná výzva), změníme ji
			if (current_challenge == 0) current_challenge = 0xDEADBEEF;

			APP_DBG(">>> BLE SECURITY: Generuji vyzvu: %lu", current_challenge);

			// Odešleme výzvu do mobilu (0x05 + 4 bajty výzvy)
			uint8_t ack_chal[5] = {0x05,
														(current_challenge >> 24) & 0xFF,
														(current_challenge >> 16) & 0xFF,
														(current_challenge >> 8)  & 0xFF,
														 current_challenge        & 0xFF};
			System_Send_ACK(ack_chal, 5, source);
			break;

		// =====================================================
		// 1B. KROK 2: MOBIL POSÍLÁ RESPONSE K OVĚŘENÍ (0x07)
		// Paket z mobilu: [0x07] [Resp_1] [Resp_2] [Resp_3] [Resp_4]
		// =====================================================
		case CMD_VERIFY_HASH:
			if (payload_len >= 5) {
				// Přečteme Response od mobilu
				uint32_t received_response = ((uint32_t)payload_data[1] << 24) |
																		 ((uint32_t)payload_data[2] << 16) |
																		 ((uint32_t)payload_data[3] << 8)  |
																			(uint32_t)payload_data[4];

				// Vypočítáme si, co měl mobil reálně poslat (POUŽIJEME hash_device)
				uint32_t expected_response = Generate_Response(current_challenge, DEVICE_CONFIG->hash_device);

				// Ověření! (Zároveň kontrolujeme, že výzva byla vůbec vygenerována)
				if (received_response == expected_response && current_challenge != 0) {
					is_unlocked = true;
					current_challenge = 0; // Výzva se smí použít jen jednou! (Obrana proti replay)

					// Zkopírujeme aktuální stav z Flash do naší RAM (Stagingu)
					memcpy(&staged_config, (void*)DEVICE_CONFIG, sizeof(BeaconConfig_t));

					APP_DBG(">>> BLE SECURITY: ODEMCENO! Hash sedi. Relace spustena.");
					uint8_t ack[4] = {0x07, 0x01, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				} else {
					APP_DBG(">>> BLE SECURITY: SPATNA ODPOVED! Utocnik?");
					current_challenge = 0; // Při chybě výzvu okamžitě spálíme
					uint8_t ack[4] = {0x07, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				}
			}
			break;

		// =====================================================
		// 2. ZAMČENÍ RELACE (0x06)
		// Paket: [0x06]
		// =====================================================
		case CMD_LOCK:
			is_unlocked = false;
			memset(&staged_config, 0, sizeof(BeaconConfig_t)); // Bezpečný výmaz RAM
			APP_DBG(">>> BLE SECURITY: ZAMCENO uzivatelem.");
			uint8_t ack_lock[4] = {0x06, 0x01, 0x00, 0x00}; System_Send_ACK(ack_lock, 4, source);
			break;

		// =====================================================
		// 3. POSTUPNÉ SKLÁDÁNÍ DO RAM (0x12 - STAGE PARAMETER)
		// Paket: [0x12] [ID] [Offset_H] [Offset_L] [Data...]
		// =====================================================
		case CMD_STAGE_PARAM:
			if (!is_unlocked) {
					APP_DBG(">>> BLE SECURITY: Zamitnuto (ZAMCENO)");
					uint8_t ack[4] = {0x12, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
					break;
			}

			if (payload_len >= 5) {
				Config_Param_t param_id = (Config_Param_t)payload_data[1];
				uint16_t offset = (payload_data[2] << 8) | payload_data[3];
				uint8_t data_len = payload_len - 4;
				uint8_t *payload = &payload_data[4];

				APP_DBG(">>> BLE STAGE: Param 0x%02X, Offset: %d, Delka: %d", param_id, offset, data_len);

				// Úprava konkrétních proměnných KONTROLY v RAM
				switch (param_id) {
					// --- 1. ZÁKLADNÍ INFORMACE ---
					case PARAM_BLE_DEVICE_NAME: if (offset + data_len <= 32) memcpy(&staged_config.BLE_device_name[offset], payload, data_len); break;
					case PARAM_HASH_DEVICE: if (data_len >= 4) staged_config.hash_device = ((uint32_t)payload[0]<<24) | ((uint32_t)payload[1]<<16) | ((uint32_t)payload[2]<<8) | payload[3]; break;

					// --- 2. VYSÍLANÉ INFORMACE ---
					case PARAM_STAT_DEVICE_TYPE: if (data_len >= 2) staged_config.stat_device_type = ((uint16_t)payload[0]<<8) | payload[1]; break;
					case PARAM_REQ_RSSI: if (data_len >= 1) staged_config.required_rssi = payload[0]; break;
					case PARAM_EVENT_NATION: if (data_len >= 1) staged_config.event_nation = payload[0]; break;
					case PARAM_EVENT_ID: if (data_len >= 4) staged_config.event_id = ((uint32_t)payload[0]<<24) | ((uint32_t)payload[1]<<16) | ((uint32_t)payload[2]<<8) | payload[3]; break;

					// --- 3. NASTAVOVACÍ PARAMETRY ---
					case PARAM_BEACON_PERIOD_MS: if (data_len >= 1) staged_config.beacon_period_ms = payload[0]; break;
					case PARAM_BEACON_PERIOD_LP: if (data_len >= 1) staged_config.beacon_period_ms_lp = payload[0]; break;
					case PARAM_BUZZER_ONOFF: if (data_len >= 1) staged_config.buzzer_onoff = payload[0]; break;
					case PARAM_TX_POWER: if (data_len >= 1) staged_config.tx_power = (int8_t)payload[0]; break;
					case PARAM_BAT_ALERT_THRESH: if (data_len >= 1) staged_config.battery_alert_threshold = payload[0]; break;

					// --- 4. TEXTY A VLASTNÍK (Ukázka několika) ---
					case PARAM_TEAM_OWNER: if (offset + data_len <= 64) memcpy(&staged_config.team_owner[offset], payload, data_len); break;
					case PARAM_TEAM_EMAIL: if (offset + data_len <= 64) memcpy(&staged_config.team_email[offset], payload, data_len); break;
					case PARAM_TEAM_OTHER_INFO: if (offset + data_len <= 1024) memcpy(&staged_config.team_other_info[offset], payload, data_len); break;
					// (Zde si můžeš dopsat zbytek podle potřeby)

					// --- 5. NB-IOT ---
					case PARAM_NBIOT_APN: if (offset + data_len <= 32) memcpy(&staged_config.nbiot_apn[offset], payload, data_len); break;
					case PARAM_NBIOT_IP: if (offset + data_len <= 32) memcpy(&staged_config.nbiot_server_ip[offset], payload, data_len); break;
					case PARAM_NBIOT_PORT: if (data_len >= 2) staged_config.nbiot_server_port = ((uint16_t)payload[0]<<8) | payload[1]; break;

					default:
						APP_DBG(">>> BLE STAGE: Neznamy parametr (0x%02X)", param_id);
						uint8_t ack_err[4] = {0x12, 0xEE, param_id, 0x00}; System_Send_ACK(ack_err, 4, source);
						break;
				}

				uint8_t ack[4] = {0x12, 0x01, param_id, 0x00}; System_Send_ACK(ack, 4, source);
			}
			break;

		// =====================================================
		// 4. HROMADNÝ ZÁPIS DO FLASH (0x13 - COMMIT)
		// Paket: [0x13]
		// =====================================================
		case CMD_COMMIT_CONFIG:
			if (!is_unlocked) {
				uint8_t ack[4] = {0x13, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}

			APP_DBG(">>> BLE COMMIT: Zapisuji vsechny RAM upravy do Flash!");
			Config_Commit(&staged_config);

			uint8_t ack_com[4] = {0x13, 0x01, 0x00, 0x00}; System_Send_ACK(ack_com, 4, source);
			break;

		// =====================================================
		// 5. BEZPEČNÝ FORMÁT PAMĚTI (0x99)
		// Nyní nevyžaduje PIN v paketu, protože je chráněn zámkem!
		// =====================================================
		case CMD_FORMAT_HISTORY:
			if (!is_unlocked) {
				APP_DBG(">>> BLE SECURITY: Zamitnuto - Formát vyžaduje odemknuti!");
				uint8_t ack[4] = {0x99, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}

			APP_DBG(">>> BLE CMD: FORMATOVANI PAMETI ZAVODNIKA! (0x99)");
			chunk_rem_len = 0;
			Logger_FormatAll();
			uint8_t ack_fmt[4] = {0x99, 0x01, 0x00, 0x00}; System_Send_ACK(ack_fmt, 4, source);
			break;
		//}

		// =====================================================
		// 6. FACTORY RESET POUZE KONFIGURACE (0x97)
		// =====================================================
		case CMD_RESET_CONFIG:
			if (!is_unlocked) {
				uint8_t ack[4] = {0x97, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}

			// Odešleme mobilu zprávu o úspěchu ještě PŘED restartem
			uint8_t ack_97[4] = {0x97, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_97, 4, source);

			// Necháme paket 100 ms odletět a pak desku zabijeme
			HAL_Delay(100);
			Config_EraseAndReboot();
			break;

		// =====================================================
		// 7. KOMPLETNÍ FACTORY RESET - VŠE (0x98)
		// =====================================================
		case CMD_RESET_ALL:
			if (!is_unlocked) {
				uint8_t ack[4] = {0x98, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}

			// Odešleme mobilu zprávu o úspěchu ještě PŘED restartem
			uint8_t ack_98[4] = {0x98, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_98, 4, source);

			// Necháme paket 100 ms odletět a pak spustíme kompletní destrukci
			HAL_Delay(100);
			System_FactoryResetAll();
			break;

		default:
			APP_DBG(">>> BLE CMD: NEZNAMY PRIKAZ (0x%02X)", cmd);
			break;
	}
}


// -----------------------------------------------------------------------------
// INICIALIZACE: Registrace Služeb do BLE Stacku
// -----------------------------------------------------------------------------
void P2PS_APP_Init(void) {
	Char_UUID_t uuid;

	// Zaregistrujeme obsluhu BLE událostí
	SVCCTL_RegisterSvcHandler(Tunnel_Event_Handler);

	// Zaregistrujeme náš asynchronní úkol (Kráječ)
	UTIL_SEQ_RegTask(1 << CFG_TASK_BLE_CHUNKER, UTIL_SEQ_RFU, BLE_Chunker_Task);

	// 1. Přidání Služby
	COPY_SERVICE_UUID(uuid.Char_UUID_128);
	aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t *) &uuid, PRIMARY_SERVICE, 7, &OrienteeringServiceHdle);

	// 2. Přidání RX Charakteristiky (Mobil píše nám)
	uuid.Char_UUID_128[12] = 0x41; // RX UUID
	aci_gatt_add_char(OrienteeringServiceHdle, UUID_TYPE_128, &uuid, 244,
										CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
										ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 10, 1, &RxCharHdle);

	// 3. Přidání TX Charakteristiky (My posíláme mobilu přes Notifikace)
	uuid.Char_UUID_128[12] = 0x42; // TX UUID
	aci_gatt_add_char(OrienteeringServiceHdle, UUID_TYPE_128, &uuid, 244,
										CHAR_PROP_NOTIFY,
										ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS, 10, 1, &TxCharHdle);

	// 100 ms časovač pro správné odpojení BLE
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &DisconnectTimerId, hw_ts_SingleShot, DisconnectTimer_Callback);
	UTIL_SEQ_RegTask(1 << CFG_TASK_EXECUTE_DISCONNECT, UTIL_SEQ_RFU, Execute_Disconnect_Task);

	APP_DBG(">>> BLE TUNEL: Inicializovano a pripraveno!");
}

void BLE_Tunnel_Send(uint8_t *pPayload, uint16_t length) {
	aci_gatt_update_char_value(OrienteeringServiceHdle, TxCharHdle, 0, length, pPayload);
}

// =============================================================================
// UNIVERZÁLNÍ ODESÍLATEL ODPOVĚDÍ (Směřuje ACK tam, odkud přišel dotaz)
// =============================================================================
void System_Send_ACK(uint8_t *payload, uint8_t length, uint8_t source) {
	if (source == 0) {
		// Odpověď do Bluetooth
		BLE_Tunnel_Send(payload, length);
	}
	else if (source == 1) {
		// Odpověď do NB-IoT (Připravíme si to pro Downlink)
		// Zde později napojíme odeslání přes Quectel (např. přes FIFO)
		APP_DBG(">>> SYS ACK: Smerovano do NB-IoT (Zatim neimplementovano)");
	}
	else if (source == 2) {
		// Odpověď do USB (Pro budoucí použití)
	}
}

// -----------------------------------------------------------------------------
// CALLBACK: Stav připojení (Volá se z app_ble.c)
// -----------------------------------------------------------------------------
void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
	switch (pNotification->P2P_Evt_Opcode)
	{
		case PEER_CONN_HANDLE_EVT:
			APP_DBG(">>> BLE TUNEL: Mobil pripojen!");
			current_conn_handle = pNotification->ConnectionHandle; // Uložíme si Handle
			break;

		case PEER_DISCON_HANDLE_EVT:
			APP_DBG(">>> BLE TUNEL: Mobil odpojen!");
			current_conn_handle = 0xFFFF; // Vymažeme Handle
			chunk_rem_len = 0;

			// BEZPEČNOST: Automaticky zamknout a smazat RAM buffer
			is_unlocked = false;
			memset(&staged_config, 0, sizeof(BeaconConfig_t));

			if (sleep_pending) {
				// Pokud mobil inicioval spánek (nebo jsme ho vykopli my přes CMD_SLEEP)
				sleep_pending = false;
				APP_DBG(">>> BLE TUNEL: Provedeno slusne odpojeni. Prepinam na MAC!");
				UTIL_SEQ_SetTask(1U << CFG_TASK_INIT_SWITCH_PROTOCOL, CFG_SCH_PRIO_0);
			} else {
				APP_DBG(">>> BLE SECURITY: Spojeni ztraceno -> AUTO-LOCK aktivovan!");
			}
			break;

		default:
			break;
	}
}

// Odpojení BLE
void Execute_Disconnect_Task(void)
{
    if (pending_disconnect_action == 1) { // CMD_SLEEP
        if (current_conn_handle != 0xFFFF) {
            aci_gap_terminate(current_conn_handle, 0x13);
        } else {
            UTIL_SEQ_SetTask(1U << CFG_TASK_INIT_SWITCH_PROTOCOL, CFG_SCH_PRIO_0);
        }
    }
    else if (pending_disconnect_action == 2) { // CMD_POWER_OFF
        System_Enter_Storage_Mode();
    }
}

