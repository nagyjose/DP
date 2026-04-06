/* USER CODE BEGIN */
/**
  ******************************************************************************
  * File Name          : App/p2p_server_app.h
  * Description        : Header for p2p_server_app.c module
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

#ifndef P2P_SERVER_APP_H
#define P2P_SERVER_APP_H

#include "app_common.h"
#include "ble.h"

// Definice událostí pro náš tunel
typedef enum {
    BLE_TUNNEL_RX_EVT,        // Mobil poslal data nám (příkaz)
    BLE_TUNNEL_NOTIFY_ENABLED, // Mobil zapnul poslech (ikonka šipek v nRF Connect)
    BLE_TUNNEL_NOTIFY_DISABLED
} BLE_Tunnel_Evt_t;

// --- Zpětná kompatibilita pro app_ble.c (události připojení/odpojení) ---
typedef enum
{
  PEER_CONN_HANDLE_EVT,
  PEER_DISCON_HANDLE_EVT,
} P2PS_APP_Opcode_Notification_evt_t;

typedef struct
{
  P2PS_APP_Opcode_Notification_evt_t   P2P_Evt_Opcode;
  uint16_t                             ConnectionHandle;
} P2PS_APP_ConnHandle_Not_evt_t;


// Funkce, které budeme volat zvenčí
void P2PS_APP_Init(void);
void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification);
void BLE_Tunnel_Send(uint8_t *pPayload, uint16_t length);
void System_Execute_Command(uint8_t *payload_data, uint8_t payload_len, uint8_t source);

#endif /* P2P_SERVER_APP_H */

