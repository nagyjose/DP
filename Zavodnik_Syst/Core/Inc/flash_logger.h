#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include "stm32wbxx_hal.h"

// --- KONFIGURACE PAMĚTI ---
// Stránka 32 (0x08020000) je vyhrazená pro statická data uživatele (index 0)
// Stránka 33 (0x08021000) až 191 jsou vyhrazeny pro náš kruhový Závodní Logger
#define LOGGER_START_ADDR       0x08021000
#define LOGGER_PAGE_SIZE        4096
#define LOGGER_MAX_PAGES        159 
#define LOGGER_MAX_RECORDS_PP   (LOGGER_PAGE_SIZE / 8) // 512 záznamů na stránku

// Speciální ID pro CLEAR kontrolu (Nulování závodu)
#define CLEAR_CONTROL_ID        999 

// --- STRUKTURA ZÁZNAMU (Přesně 8 bajtů / 64 bitů) ---
typedef union {
    uint64_t double_word;
    struct {
        uint8_t  raw_payload[6]; // Původních 6 bajtů přímo ze vzduchu
        uint8_t  rssi;           // 1 byte: Síla signálu (absolutní hodnota)
        uint8_t  index;          // 1 byte: Pořadové číslo (0-255)
    } __attribute__((packed)) data;
} PunchRecord_t;

// --- FUNKCE ---
void Logger_Init(void);
// Do funkce teď předáme jen ukazatel na surová data a RSSI
void Logger_SavePunch(uint8_t* raw_payload, uint8_t rssi_abs);
void Logger_NewRace(uint32_t start_time);

#endif /* FLASH_LOGGER_H */
