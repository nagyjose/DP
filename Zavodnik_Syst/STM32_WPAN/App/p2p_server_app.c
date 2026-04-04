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

// =============================================================================
// STAVOVÉ PROMĚNNÉ PRO KRÁJEČ (CHUNKER)
// =============================================================================
static uint8_t *chunk_ptr = NULL;        // Ukazatel na to, co zrovna odesíláme
static uint32_t chunk_rem_len = 0;       // Kolik bajtů nám ještě zbývá odeslat
static uint8_t  chunk_active_cmd = 0;    // Jaký příkaz se zrovna vykonává

#define CHUNK_MAX_SIZE 20 // Maximální velikost jedné rány (BLE MTU to bezpečně snese)

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

// -----------------------------------------------------------------------------
// CALLBACK: PARSER - Zde zpracováváme příkazy z Mobilu a zprávy z Rádia
// -----------------------------------------------------------------------------
static SVCCTL_EvtAckStatus_t Tunnel_Event_Handler(void *pckt)
{
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
					uint8_t cmd = mod->Attr_Data[0];

					// Bezpečnostní pojistka: Kdyby ještě běžel starý přenos, natvrdo ho zrušíme
					chunk_rem_len = 0;

					// =========================================================
					// ROZCESTNÍK PŘÍKAZŮ (COMMAND DICTIONARY)
					// =========================================================
					switch(cmd)
					{
						case 0x10: // CMD_READ_CONFIG
							APP_DBG(">>> BLE CMD: READ CONFIG (0x10) - Odesilam %d bajtu", sizeof(RunnerConfig_t));

							// Nabijeme Kráječ celou Flash strukturou Závodníka
							chunk_ptr = (uint8_t*)DEVICE_CONFIG;
							chunk_rem_len = sizeof(RunnerConfig_t);
							chunk_active_cmd = cmd;

							// Zmáčkneme spoušť
							UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
							break;

						case 0x20: // CMD_DOWNLOAD_CURRENT (Poslední stránka)
						case 0x21: // CMD_DOWNLOAD_ALL (Celá historie)
						case 0x22: // CMD_DOWNLOAD_SPECIFIC (S parametrem)
						{
							// Přečteme případný druhý bajt (parametr), pokud ho mobil poslal
							uint8_t param = (mod->Attr_Data_Length >= 2) ? mod->Attr_Data[1] : 0;

							uint8_t *data_ptr = NULL;
							uint32_t data_len = 0;

							// Zeptáme se Loggeru, kde data leží a kolik jich je
							Logger_GetDownloadData(cmd, param, &data_ptr, &data_len);

							if (data_len > 0 && data_ptr != NULL) {
								APP_DBG(">>> BLE CMD: Odesilam LOGY (0x%02X) - Delka: %lu bajtu", cmd, data_len);

								// Nabijeme Kráječ a spustíme střelbu!
								chunk_ptr = data_ptr;
								chunk_rem_len = data_len;
								chunk_active_cmd = cmd;
								UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CHUNKER, CFG_SCH_PRIO_0);
							} else {
								APP_DBG(">>> BLE CMD: Zadne logy k odeslani!");
								// Odpovíme mobilu, že je paměť prázdná
								uint8_t ack_empty[4] = {cmd, 0x00, 0x00, 0x00};
								BLE_Tunnel_Send(ack_empty, 4);
							}
							break;
						}

						case 0x40: // CMD_IDENTIFY (Najdi můj čip)
							APP_DBG(">>> BLE CMD: IDENTIFY (0x40) - Zacinam blikat!");
							// TBD: Zapnout LED/Buzzer task
							break;

						case 0x99: // CMD_FORMAT_MEMORY (Nyní chráněno heslem!)
						{
								APP_DBG(">>> BLE CMD: Pozadavek na FORMAT (0x99)");

								// 1. Ochrana: Mobil poslal příliš krátkou zprávu (chybí heslo)
								if (mod->Attr_Data_Length < 5) {
										APP_DBG(">>> BLE SECURITY: Prikaz zamitnut - chybi heslo!");
										uint8_t ack_err[4] = {0x99, 0xEE, 0x00, 0x00}; // EE = Error
										BLE_Tunnel_Send(ack_err, 4);
										break;
								}

								// 2. Extrakce hesla z přijaté zprávy (Bajty 1 až 4)
								uint32_t received_pin = ((uint32_t)mod->Attr_Data[1] << 24) |
																				((uint32_t)mod->Attr_Data[2] << 16) |
																				((uint32_t)mod->Attr_Data[3] << 8)  |
																				 (uint32_t)mod->Attr_Data[4];

								// 3. Porovnání s uloženým heslem ve Flash paměti
								if (received_pin == DEVICE_CONFIG->comp_device_hash) {
										APP_DBG(">>> BLE SECURITY: Heslo OK. Spoustim mazani!");

										chunk_rem_len = 0; // Zastavení případného stahování
										Logger_FormatAll();

										uint8_t ack_ok[4] = {0x99, 0x01, 0x00, 0x00}; // 01 = Úspěch
										BLE_Tunnel_Send(ack_ok, 4);
								} else {
										APP_DBG(">>> BLE SECURITY: SPATNE HESLO! (Prijato: %lu)", received_pin);
										uint8_t ack_err[4] = {0x99, 0xEE, 0x00, 0x00};
										BLE_Tunnel_Send(ack_err, 4);
								}
								break;
						}

						default:
							APP_DBG(">>> BLE CMD: NEZNAMY PRIKAZ (0x%02X)", cmd);
							break;
					}
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
			break;
		case PEER_DISCON_HANDLE_EVT:
			APP_DBG(">>> BLE TUNEL: Mobil odpojen!");
			// Pokud se mobil utrhne uprostřed stahování, natvrdo ho zrušíme
			chunk_rem_len = 0;
			break;
		default:
			break;
	}
}
