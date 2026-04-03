#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include "stm32wbxx_hal.h"

// Stejné adresy jako u Závodníka (zbylo nám tu 159 volných stránek)
#define LOGGER_START_ADDR       0x08021000
#define LOGGER_PAGE_SIZE        4096
#define LOGGER_MAX_PAGES        159 
#define LOGGER_MAX_RECORDS_PP   (LOGGER_PAGE_SIZE / 8) // 512 záznamů

// --- NAŠE NOVÁ 8BAJTOVÁ STRUKTURA PRO KONTROLU ---
typedef union {
    uint64_t double_word;
    struct {
        uint8_t  id_bytes[3]; // 3 byty: ID Závodníka
        uint8_t  sub_seconds; // 1 byte: Zlomky vteřiny
        uint32_t unix_time;   // 4 byty: Čas oražení z RTC
    } __attribute__((packed)) data;
} ControlRecord_t;

void Logger_Init(void);
void Config_Init(void);
void Logger_SavePunch_Kontrola(uint8_t* id_3bytes, uint8_t sub_sec, uint32_t unix_time);

#endif /* FLASH_LOGGER_H */
