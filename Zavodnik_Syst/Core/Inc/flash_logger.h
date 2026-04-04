#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include "stm32wbxx_hal.h"
#include "app_conf.h"

// --- KONFIGURACE PAMĚTI ---
// Stránka 32 (0x08020000) je vyhrazená pro statická data uživatele (index 0)
// Stránka 33 (0x08021000) až 191 jsou vyhrazeny pro náš kruhový Závodní Logger
#define LOGGER_START_ADDR       0x08021000
#define LOGGER_PAGE_SIZE        4096
#define LOGGER_MAX_PAGES        159 
#define LOGGER_MAX_RECORDS_PP   (LOGGER_PAGE_SIZE / 8) // 512 záznamů na stránku

// --- DEFINICE KÓDŮ KONTROL ---
#define CLEAR_CONTROL_ID   0
#define CHECK_CONTROL_ID   1
#define START_CONTROL_ID   2
#define FINISH_CONTROL_ID  3


// --- STRUKTURA ZÁZNAMU (Přesně 8 bajtů / 64 bitů) ---
typedef union {
    uint64_t double_word;
    struct {
        uint8_t  raw_payload[6]; // Původních 6 bajtů přímo ze vzduchu
        uint8_t  rssi;           // 1 byte: Síla signálu (absolutní hodnota)
        int8_t   temperature;    // 1 byte: Teplota procesoru (-128 až +127 °C)
    } __attribute__((packed)) data;
} PunchRecord_t;


// --- FUNKCE ---
void Logger_Init(void);
// Do funkce teď předáme jen ukazatel na surová data a RSSI
void Logger_SavePunch(uint8_t* raw_payload, uint8_t rssi_abs, int8_t temperature);
// Upravená hlavička funkce, aby přijala přímo data majáku CLEAR
void Logger_NewRace(uint8_t* raw_clear_payload);
// PŘIDÁNO: Funkce pro vyčítání dat do BLE Tunelu
void Logger_GetDownloadData(uint8_t cmd, uint8_t param, uint8_t **start_ptr, uint32_t *len);
void Logger_FormatAll(void);
void Config_Commit(RunnerConfig_t *new_cfg);


#endif /* FLASH_LOGGER_H */
