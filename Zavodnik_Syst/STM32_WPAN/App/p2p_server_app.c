/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    p2p_server_app.c
 * @author  MCD Application Team
 * @brief   peer to peer Server Application
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

#include "p2p_server_app.h"
#include "dbg_trace.h"
#include "app_conf.h"
#include "ble.h"       // PŘIDÁNO: Definice BLE struktur
#include "hci_tl.h"    // PŘIDÁNO: Nízkoúrovňové HCI pakety pro náš Parser

// --- NAŠE UNIKÁTNÍ UUID (128-bit) ---
// Služba: ...FE40..., RX: ...FE41..., TX: ...FE42...
#define COPY_SERVICE_UUID(uuid_struct) { \
  (uuid_struct)[0]=0x19; (uuid_struct)[1]=0xed; (uuid_struct)[2]=0x82; (uuid_struct)[3]=0xae; \
  (uuid_struct)[4]=0xed; (uuid_struct)[5]=0x21; (uuid_struct)[6]=0x4c; (uuid_struct)[7]=0x9d; \
  (uuid_struct)[8]=0x41; (uuid_struct)[9]=0x45; (uuid_struct)[10]=0x22; (uuid_struct)[11]=0x8e; \
  (uuid_struct)[12]=0x40; (uuid_struct)[13]=0xFE; (uuid_struct)[14]=0x00; (uuid_struct)[15]=0x00; }

#define COPY_RX_UUID(uuid_struct) { (uuid_struct)[12]=0x41; (uuid_struct)[13]=0xFE; /* zbytek stejný */ }
#define COPY_TX_UUID(uuid_struct) { (uuid_struct)[12]=0x42; (uuid_struct)[13]=0xFE; /* zbytek stejný */ }

static uint16_t OrienteeringServiceHdle;
static uint16_t RxCharHdle;
static uint16_t TxCharHdle;

// -----------------------------------------------------------------------------
// CALLBACK: PARSER - Zde zpracováváme příkazy z Mobilu
// -----------------------------------------------------------------------------
static SVCCTL_EvtAckStatus_t Tunnel_Event_Handler(void *pckt) {
	hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
	hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

	if (hci_pckt->type != HCI_EVENT_PKT_TYPE) return SVCCTL_EvtNotAck;

	if (event_pckt->evt == HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE) {
		evt_blecore_aci *blecore_evt = (evt_blecore_aci*)event_pckt->data;

		if (blecore_evt->ecode == ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE) {
			aci_gatt_attribute_modified_event_rp0 *mod = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;

			// Pokud mobil zapsal do RX charakteristiky
			if (mod->Attr_Handle == (RxCharHdle + 1)) {
				uint8_t cmd = mod->Attr_Data[0];

				// =============================================================
				// ROZCESTNÍK PŘÍKAZŮ (COMMAND DICTIONARY)
				// =============================================================
				switch(cmd)
				{
					case 0x10: // CMD_READ_CONFIG
						APP_DBG(">>> BLE CMD: READ CONFIG (0x10)");

						// Zatím pošleme prvních 100 bajtů z DEVICE_CONFIG z Flash
						// Obsahuje: magic_word, kontrolni soucet, hw_verzi, jmeno atd.
						BLE_Tunnel_Send((uint8_t*)DEVICE_CONFIG, 100);
						break;

					case 0x20: // CMD_DOWNLOAD_CURRENT
						APP_DBG(">>> BLE CMD: DOWNLOAD CURRENT LOG (0x20)");
						// TBD v další fázi
						uint8_t ack_20[4] = {0x20, 0x01, 0x00, 0x00};
						BLE_Tunnel_Send(ack_20, 4);
						break;

					case 0x40: // CMD_IDENTIFY (Najdi můj čip)
						APP_DBG(">>> BLE CMD: IDENTIFY (0x40) - Zacinam blikat!");
						// TBD: Můžeme zde zapnout bzučák nebo LED
						break;

					default:
						APP_DBG(">>> BLE CMD: NEZNAMY PRIKAZ (0x%02X)", cmd);
						uint8_t error_msg[2] = {0xEE, cmd}; // Error Echo
						BLE_Tunnel_Send(error_msg, 2);
						break;
				}
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
    
    // Zaregistrujeme obsluhu událostí
    SVCCTL_RegisterSvcHandler(Tunnel_Event_Handler);

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
            break;
        default:
            break;
    }
}


