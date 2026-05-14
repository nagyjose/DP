/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    app_nbiot.h
 * @author  Josef Nagy
 * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Josef Nagy.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef APP_NBIOT_H
#define APP_NBIOT_H

#include <stdint.h>
#include <stdbool.h>

// Struktura jednoho záznamu (7 bajtů v RAM)
typedef struct {
	uint8_t runner_raw[3]; // Původní 3 bajty od Závodníka (Typ a ID)
	uint32_t timestamp;    // UNIX čas oražení
} NBIOT_PunchRecord_t;

// Veřejné funkce modulu
void NBIOT_FIFO_Init(void);
bool NBIOT_FIFO_Push(uint8_t *runner_raw, uint32_t timestamp);
bool NBIOT_FIFO_Pop(NBIOT_PunchRecord_t *out_record);
uint16_t NBIOT_FIFO_GetCount(void);
void NBIOT_Process_Task(void);

#endif // APP_NBIOT_H
