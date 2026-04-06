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


// --- DOPŘEDNÁ DEKLARACE NOVÝCH FUNKCÍ ---
void System_Send_ACK(uint8_t *payload, uint8_t length, uint8_t source);
void System_Execute_Command(uint8_t *payload_data, uint8_t payload_len, uint8_t source);

// =============================================================================
// UNIVERZÁLNÍ ODESÍLATEL ODPOVĚDÍ (Směřuje ACK tam, odkud přišel dotaz)
// =============================================================================
void System_Send_ACK(uint8_t *payload, uint8_t length, uint8_t source) {
	if (source == 0) {
		// Odpověď do Bluetooth
		BLE_Tunnel_Send(payload, length);
	}
	else if (source == 2) {
		// Odpověď do USB (Pro budoucí implementaci)
		// CDC_Transmit_FS(payload, length);
	}
}

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
		CMD_GET_BATTERY         = 0x14, // <--- PŘIDÁNO: Samostatný stav baterie
    CMD_DOWNLOAD_CURRENT    = 0x20,
    CMD_DOWNLOAD_ALL        = 0x21,
    CMD_DOWNLOAD_SPECIFIC   = 0x22,

    // --- Zápis do Konfigurace (Staging) ---
    CMD_STAGE_PARAM         = 0x12,
    CMD_COMMIT_CONFIG       = 0x13,

    // --- Nástroje ---
    CMD_IDENTIFY            = 0x40,
		CMD_SLEEP               = 0x51,  // PŘIDÁNO: Příkaz pro návrat do MAC

    // --- Mazání a Resety ---
    CMD_RESET_CONFIG        = 0x97,
    CMD_RESET_ALL           = 0x98,
    CMD_FORMAT_HISTORY      = 0x99
} BLE_Command_t;

// =============================================================================
// SEZNAM PARAMETRŮ PRO ÚPRAVU KONFIGURACE (Pro příkaz 0x12)
// =============================================================================
typedef enum {
    // --- Vysílané a základní informace ---
    PARAM_BLE_DEVICE_NAME   = 0x01,
    PARAM_DEVICE_TYPE       = 0x02,
    PARAM_DEVICE_ID         = 0x03,
    PARAM_DEVICE_HASH       = 0x04,

    // --- Rozhodovací parametry ---
    PARAM_COOLDOWN_MS       = 0x05,
    PARAM_REQ_HITS          = 0x06,
    PARAM_NUM_HITS          = 0x07,
    PARAM_BUZZER_ONOFF      = 0x08,

    // --- Informace o závodníkovi ---
    PARAM_COMP_NAME         = 0x09,
    PARAM_COMP_NAT          = 0x0A,
    PARAM_COMP_BIRTH_DATE   = 0x0B,
    PARAM_COMP_EMAIL        = 0x0C,
    PARAM_COMP_PHONE        = 0x0D,
    PARAM_COMP_ADDRESS      = 0x0E,
    PARAM_COMP_REGISTRATION = 0x0F,
    PARAM_COMP_IOFID        = 0x10,
    PARAM_COMP_ORISID       = 0x11,
    PARAM_COMP_TEAM_1       = 0x12,
    PARAM_COMP_TEAM_2       = 0x13,
    PARAM_COMP_TEAM_3       = 0x14,

    // --- Dlouhé texty ---
    PARAM_COMP_MEDICAL_INFO = 0x15,
    PARAM_COMP_OTHER_INFO   = 0x16
} Config_Param_t;

extern void Get_ADC_Measurements(int8_t *out_temp, uint16_t *out_batt_mv);

static uint8_t pending_disconnect_action = 0; // 0 = Nic, 1 = SLEEP, 2 = POWER_OFF
static uint8_t DisconnectTimerId;

void Execute_Disconnect_Task(void); // <--- Přidej tento řádek nahoru k definicím

// Callback časovače pouze probudí finální Task
static void DisconnectTimer_Callback(void) {
    UTIL_SEQ_SetTask(1 << CFG_TASK_EXECUTE_DISCONNECT, CFG_SCH_PRIO_0);
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
static RunnerConfig_t staged_config;       // RAM kopie konfigurace pro úpravy

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

	// Zjistíme, jestli pošleme plných 200 bajtů, nebo už jen zbytek
	uint16_t send_len = (chunk_rem_len > CHUNK_MAX_SIZE) ? CHUNK_MAX_SIZE : chunk_rem_len;

	// Fyzický pokus o odeslání do fronty Bluetooth rádia
	tBleStatus ret = aci_gatt_update_char_value(OrienteeringServiceHdle, TxCharHdle, 0, send_len, chunk_ptr);

	if (ret == BLE_STATUS_SUCCESS)
	{
		// Paket vložen do vysílače! Posuneme ukazatele dopředu.
		chunk_ptr += send_len;
		chunk_rem_len -= send_len;

		// PŘIDÁNO: Nativní podpora pro Kruhový Buffer (Přetečení)
		if ((uint32_t)chunk_ptr >= (LOGGER_START_ADDR + (LOGGER_MAX_PAGES * LOGGER_PAGE_SIZE))) {
			chunk_ptr = (uint8_t*)LOGGER_START_ADDR;
		}

		if (chunk_rem_len > 0) {
			// Pořád máme data, takže se pokusíme rovnou vypálit další dávku
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
// AGNOSTICKÉ JÁDRO PRO ZPRACOVÁNÍ PŘÍKAZŮ (Voláno z BLE i USB)
// source: 0 = BLE, 2 = USB
// =============================================================================
void System_Execute_Command(uint8_t *payload_data, uint8_t payload_len, uint8_t source)
{
	if (payload_len == 0) return;

	BLE_Command_t cmd = (BLE_Command_t)payload_data[0];

	// Bezpečnostní pojistka: Kdyby ještě běžel starý přenos, natvrdo ho zrušíme
	chunk_rem_len = 0;

	switch(cmd)
	{
		case CMD_READ_CONFIG: // CMD_READ_CONFIG
			APP_DBG(">>> SYS CMD: READ CONFIG (0x10) - Zdroj: %d", source);

			chunk_ptr = (uint8_t*)DEVICE_CONFIG;
			chunk_rem_len = sizeof(RunnerConfig_t);
			chunk_active_cmd = cmd;

			// Spoušť (USB si bude řešit odesílání vlastním způsobem)
			if (source == 0) UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
			break;

		case CMD_DOWNLOAD_CURRENT:
		case CMD_DOWNLOAD_ALL:
		case CMD_DOWNLOAD_SPECIFIC:
		{
			uint8_t param = (payload_len >= 2) ? payload_data[1] : 0;
			uint8_t *data_ptr = NULL;
			uint32_t data_len = 0;

			Logger_GetDownloadData(cmd, param, &data_ptr, &data_len);

			if (data_len > 0 && data_ptr != NULL) {
				APP_DBG(">>> SYS CMD: Odesilam LOGY (0x%02X) - Delka: %lu", cmd, data_len);
				chunk_ptr = data_ptr;
				chunk_rem_len = data_len;
				chunk_active_cmd = cmd;
				if (source == 0) UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
			} else {
				APP_DBG(">>> SYS CMD: Zadne logy k odeslani!");
				uint8_t ack_empty[4] = {cmd, 0x00, 0x00, 0x00};
				System_Send_ACK(ack_empty, 4, source);
			}
			break;
		}

		case CMD_GET_BATTERY:
			if (is_unlocked) {
				uint8_t bat_payload[3];
				int8_t teplota_dummy;
				uint16_t bat_mv;
				Get_ADC_Measurements(&teplota_dummy, &bat_mv);

				bat_payload[0] = 0x14;
				bat_payload[1] = (bat_mv >> 8) & 0xFF;
				bat_payload[2] = bat_mv & 0xFF;

				APP_DBG(">>> SYS CMD: GET BATTERY (0x14) - Napeti: %d mV", bat_mv);
				System_Send_ACK(bat_payload, 3, source);
			} else {
				uint8_t err_lock[4] = {cmd, 0xAA, 0x00, 0x00}; System_Send_ACK(err_lock, 4, source);
			}
			break;

		case CMD_IDENTIFY:
			APP_DBG(">>> SYS CMD: IDENTIFY (0x40) - Zacinam signalizovat!");
			System_Signalize_Start(10);
			uint8_t ack_id[4] = {0x40, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_id, 4, source);
			break;

		case CMD_SLEEP:
			APP_DBG(">>> SYS CMD: SLEEP (0x51) - Zacinam slusne odpojovani...");
			uint8_t ack_sleep[4] = {0x51, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_sleep, 4, source);

			sleep_pending = true;
			pending_disconnect_action = 1;

			HW_TS_Start(DisconnectTimerId, (uint32_t)(100 * 1000 / CFG_TS_TICK_VAL));
			break;

		case CMD_UNLOCK:
			current_challenge = HAL_GetTick() ^ DEVICE_CONFIG->magic_word ^ LL_FLASH_GetUDN();
			if (current_challenge == 0) current_challenge = 0xDEADBEEF;

			APP_DBG(">>> SYS SECURITY: Generuji vyzvu: %lu", current_challenge);

			uint8_t ack_chal[5] = {0x05,
														(current_challenge >> 24) & 0xFF,
														(current_challenge >> 16) & 0xFF,
														(current_challenge >> 8)  & 0xFF,
														 current_challenge        & 0xFF};
			System_Send_ACK(ack_chal, 5, source);
			break;

		case CMD_VERIFY_HASH:
			if (payload_len >= 5) {
				uint32_t received_response = ((uint32_t)payload_data[1] << 24) |
																		 ((uint32_t)payload_data[2] << 16) |
																		 ((uint32_t)payload_data[3] << 8)  |
																			(uint32_t)payload_data[4];

				uint32_t expected_response = Generate_Response(current_challenge, DEVICE_CONFIG->comp_device_hash);

				if (received_response == expected_response && current_challenge != 0) {
					is_unlocked = true;
					current_challenge = 0;
					memcpy(&staged_config, (void*)DEVICE_CONFIG, sizeof(RunnerConfig_t));

					APP_DBG(">>> SYS SECURITY: ODEMCENO! Hash sedi. Relace spustena.");
					uint8_t ack[4] = {0x07, 0x01, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				} else {
					APP_DBG(">>> SYS SECURITY: SPATNA ODPOVED! Utocnik?");
					current_challenge = 0;
					uint8_t ack[4] = {0x07, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				}
			}
			break;

		case CMD_LOCK:
			is_unlocked = false;
			memset(&staged_config, 0, sizeof(RunnerConfig_t));
			APP_DBG(">>> SYS SECURITY: ZAMCENO uzivatelem.");
			uint8_t ack_lock[4] = {0x06, 0x01, 0x00, 0x00}; System_Send_ACK(ack_lock, 4, source);
			break;

		case CMD_STAGE_PARAM:
			if (!is_unlocked) {
					APP_DBG(">>> SYS SECURITY: Zamitnuto (ZAMCENO)");
					uint8_t ack[4] = {0x12, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
					break;
			}

			if (payload_len >= 5) {
				Config_Param_t param_id = (Config_Param_t)payload_data[1];
				uint16_t offset = (payload_data[2] << 8) | payload_data[3];
				uint8_t data_len = payload_len - 4;
				uint8_t *payload = &payload_data[4];

				APP_DBG(">>> SYS STAGE: Param 0x%02X, Offset: %d, Delka: %d", param_id, offset, data_len);

				switch (param_id) {
					case PARAM_BLE_DEVICE_NAME: if (offset + data_len <= 32) memcpy(&staged_config.BLE_device_name[offset], payload, data_len); break;
					case PARAM_DEVICE_TYPE: if (data_len >= 1) staged_config.comp_device_type = payload[0]; break;
					case PARAM_DEVICE_ID: if (data_len >= 4) staged_config.comp_device_id = ((uint32_t)payload[0]<<24) | ((uint32_t)payload[1]<<16) | ((uint32_t)payload[2]<<8) | payload[3]; break;
					case PARAM_DEVICE_HASH: if (data_len >= 4) staged_config.comp_device_hash = ((uint32_t)payload[0]<<24) | ((uint32_t)payload[1]<<16) | ((uint32_t)payload[2]<<8) | payload[3]; break;
					case PARAM_COOLDOWN_MS: if (data_len >= 2) staged_config.cooldown_ms = ((uint16_t)payload[0]<<8) | payload[1]; break;
					case PARAM_REQ_HITS: if (data_len >= 1) staged_config.required_hits = payload[0]; break;
					case PARAM_NUM_HITS: if (data_len >= 1) staged_config.num_hits = payload[0]; break;
					case PARAM_BUZZER_ONOFF: if (data_len >= 1) staged_config.buzzer_onoff = payload[0]; break;
					case PARAM_COMP_NAME: if (offset + data_len <= 64) memcpy(&staged_config.comp_name[offset], payload, data_len); break;
					case PARAM_COMP_NAT: if (offset + data_len <= 4)  memcpy(&staged_config.comp_nationality[offset], payload, data_len); break;
					case PARAM_COMP_BIRTH_DATE: if (data_len >= 4) staged_config.comp_birthday_date = ((uint32_t)payload[0]<<24) | ((uint32_t)payload[1]<<16) | ((uint32_t)payload[2]<<8) | payload[3]; break;
					case PARAM_COMP_EMAIL: if (offset + data_len <= 64) memcpy(&staged_config.comp_email[offset], payload, data_len); break;
					case PARAM_COMP_PHONE: if (offset + data_len <= 16) memcpy(&staged_config.comp_telephone[offset], payload, data_len); break;
					case PARAM_COMP_ADDRESS: if (offset + data_len <= 128) memcpy(&staged_config.comp_address[offset], payload, data_len); break;
					case PARAM_COMP_REGISTRATION: if (offset + data_len <= 8)  memcpy(&staged_config.comp_registration[offset], payload, data_len); break;
					case PARAM_COMP_IOFID: if (data_len >= 4) staged_config.comp_iofid = ((uint32_t)payload[0]<<24) | ((uint32_t)payload[1]<<16) | ((uint32_t)payload[2]<<8) | payload[3]; break;
					case PARAM_COMP_ORISID: if (data_len >= 4) staged_config.comp_orisid = ((uint32_t)payload[0]<<24) | ((uint32_t)payload[1]<<16) | ((uint32_t)payload[2]<<8) | payload[3]; break;
					case PARAM_COMP_TEAM_1: if (offset + data_len <= 64) memcpy(&staged_config.comp_team_1[offset], payload, data_len); break;
					case PARAM_COMP_TEAM_2: if (offset + data_len <= 64) memcpy(&staged_config.comp_team_2[offset], payload, data_len); break;
					case PARAM_COMP_TEAM_3: if (offset + data_len <= 64) memcpy(&staged_config.comp_team_3[offset], payload, data_len); break;
					case PARAM_COMP_MEDICAL_INFO: if (offset + data_len <= 512) memcpy(&staged_config.comp_medical_info[offset], payload, data_len); break;
					case PARAM_COMP_OTHER_INFO: if (offset + data_len <= 512) memcpy(&staged_config.comp_other_info[offset], payload, data_len); break;
					default:
						APP_DBG(">>> SYS STAGE: Neznamy parametr (0x%02X)", param_id);
						uint8_t ack_err[4] = {0x12, 0xEE, param_id, 0x00}; System_Send_ACK(ack_err, 4, source);
						break;
				}

				uint8_t ack[4] = {0x12, 0x01, param_id, 0x00}; System_Send_ACK(ack, 4, source);
			}
			break;

		case CMD_COMMIT_CONFIG:
			if (!is_unlocked) {
				uint8_t ack[4] = {0x13, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}
			APP_DBG(">>> SYS COMMIT: Zapisuji vsechny RAM upravy do Flash!");
			Config_Commit(&staged_config);
			uint8_t ack_com[4] = {0x13, 0x01, 0x00, 0x00}; System_Send_ACK(ack_com, 4, source);
			break;

		case CMD_FORMAT_HISTORY:
			if (!is_unlocked) {
				APP_DBG(">>> SYS SECURITY: Zamitnuto - Formát vyžaduje odemknuti!");
				uint8_t ack[4] = {0x99, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}
			APP_DBG(">>> SYS CMD: FORMATOVANI PAMETI ZAVODNIKA! (0x99)");
			chunk_rem_len = 0;
			Logger_FormatAll();
			uint8_t ack_fmt[4] = {0x99, 0x01, 0x00, 0x00}; System_Send_ACK(ack_fmt, 4, source);
			break;

		case CMD_RESET_CONFIG:
			if (!is_unlocked) {
				uint8_t ack[4] = {0x97, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}
			uint8_t ack_97[4] = {0x97, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_97, 4, source);
			HAL_Delay(100);
			Config_EraseAndReboot();
			break;

		case CMD_RESET_ALL:
			if (!is_unlocked) {
				uint8_t ack[4] = {0x98, 0xEE, 0x00, 0x00}; System_Send_ACK(ack, 4, source);
				break;
			}
			uint8_t ack_98[4] = {0x98, 0x01, 0x00, 0x00};
			System_Send_ACK(ack_98, 4, source);
			HAL_Delay(100);
			System_FactoryResetAll();
			break;

		default:
			APP_DBG(">>> SYS CMD: NEZNAMY PRIKAZ (0x%02X)", cmd);
			break;
	}
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
					// PŘEDÁNÍ DO AGNOSTICKÉHO JÁDRA (Zdroj 0 = BLE)
					System_Execute_Command(mod->Attr_Data, mod->Attr_Data_Length, 0);
				}
				break;
			}
		}
	}
	return SVCCTL_EvtNotAck;
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

	// Vložit do P2PS_APP_Init()
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &DisconnectTimerId, hw_ts_SingleShot, DisconnectTimer_Callback);
	UTIL_SEQ_RegTask(1 << CFG_TASK_EXECUTE_DISCONNECT, UTIL_SEQ_RFU, Execute_Disconnect_Task);

	APP_DBG(">>> BLE TUNEL: Inicializovano a pripraveno!");
}

void BLE_Tunnel_Send(uint8_t *pPayload, uint16_t length) {
	aci_gatt_update_char_value(OrienteeringServiceHdle, TxCharHdle, 0, length, pPayload);
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
			memset(&staged_config, 0, sizeof(RunnerConfig_t));

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
		//System_Enter_Storage_Mode(); // CMD_POWER_OFF nepoužíváme
	}
}
