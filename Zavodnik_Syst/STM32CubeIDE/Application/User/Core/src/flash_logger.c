

#include "flash_logger.h"
#include "dbg_trace.h"
#include "app_conf.h"
#include "app_common.h"

// Globální proměnné pro rychlý zápis z RAM
static uint32_t current_flash_ptr = LOGGER_START_ADDR;
static uint16_t current_punch_index = 0;

// Interní pomocná funkce pro smazání 4KB stránky
static void ErasePage(uint32_t page_address)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    // Výpočet čísla stránky z adresy
		uint32_t page_index = (page_address - 0x08000000) / 4096;

		// VÝRAZNÝ LOG PŘI MAZÁNÍ
		APP_DBG("--- FLASH: MAZANI STRANKY 0x%08X (Cislo: %lu) ---", page_address, page_index);


    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    // Výpočet čísla stránky z adresy (0x08000000 je start, stránka má 4096 B)
    EraseInitStruct.Page = (page_address - 0x08000000) / 4096;
    EraseInitStruct.NbPages = 1;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Lock();
}

// 1. INICIALIZACE PŘI BOOTU (Hledání ztraceného pointeru)
void Logger_Init(void)
{
    uint32_t active_page_addr = LOGGER_START_ADDR;
    uint8_t found = 0;

    // Skenujeme první záznam na každé z 159 stránek
    for (int i = 0; i < LOGGER_MAX_PAGES; i++)
    {
        uint32_t page_addr = LOGGER_START_ADDR + (i * LOGGER_PAGE_SIZE);
        uint32_t next_page_addr = LOGGER_START_ADDR + (((i + 1) % LOGGER_MAX_PAGES) * LOGGER_PAGE_SIZE);

        uint64_t current_page_head = *(volatile uint64_t*)page_addr;
        uint64_t next_page_head = *(volatile uint64_t*)next_page_addr;

        // Našli jsme stránku, která má data, ale stránka za ní je smazaná (samé 1). 
        // To je náš aktivní závod!
        if (current_page_head != 0xFFFFFFFFFFFFFFFF && next_page_head == 0xFFFFFFFFFFFFFFFF)
        {
            active_page_addr = page_addr;
            found = 1;
            break;
        }
    }

    // Pokud je paměť úplně prázdná (první zapnutí)
    if (!found) {
        ErasePage(active_page_addr);
    }

    // Nyní najdeme první prázdné místo uvnitř aktivní stránky
    current_flash_ptr = active_page_addr;
    current_punch_index = 0;
    
    while (*(volatile uint64_t*)current_flash_ptr != 0xFFFFFFFFFFFFFFFF)
    {
        current_punch_index++;
        current_flash_ptr += 8;
        
        // Bezpečnostní pojistka proti přetečení stránky
        if (current_punch_index >= LOGGER_MAX_RECORDS_PP) break;
    }
    
    APP_DBG("LOGGER INIT: Aktivni stranka: 0x%08X, Zaznamu: %d", active_page_addr, current_punch_index);
}

// 2. ORAŽENÍ KONTROLY (Bleskový zápis)
// Najdi funkci Logger_SavePunch a uprav její tělo takto:

void Logger_SavePunch(uint8_t* raw_payload, uint8_t rssi_abs, int8_t temperature)
{
    if (current_punch_index >= LOGGER_MAX_RECORDS_PP) {
        APP_DBG("LOGGER ERROR: Stranka je plna!");
        return; 
    }

    uint32_t saved_addr = current_flash_ptr;
    PunchRecord_t record;

    // Rychlé zkopírování 6 bajtů surových dat
    for(int i = 0; i < 6; i++) {
        record.data.raw_payload[i] = raw_payload[i];
    }

    record.data.rssi = rssi_abs;
    record.data.temperature = temperature; // Nová teplota!

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_flash_ptr, record.double_word);
    HAL_FLASH_Lock();

    current_flash_ptr += 8;
    current_punch_index++;
    
    // --- Rozbalení POUZE pro lidský výpis do logu ---
    uint16_t control_id = (raw_payload[0] << 4) | (raw_payload[1] >> 4);
    uint8_t sub_seconds = raw_payload[1] & 0x0F;
    uint32_t unix_time = ((uint32_t)raw_payload[2] << 24) | ((uint32_t)raw_payload[3] << 16) |
                         ((uint32_t)raw_payload[4] << 8) | (uint32_t)raw_payload[5];

    APP_DBG("LOG ULOZEN -> Adr: 0x%08X | Ctrl_ID: %d | Cas: %lu.%d s | RSSI: -%d dBm | Teplota: %d C",
             saved_addr, control_id, unix_time, sub_seconds, rssi_abs, temperature);
}

// 3. CLEAR KONTROLA (Nulování a přesun na další stránku)
void Logger_NewRace(uint8_t* raw_clear_payload)
{
    uint32_t current_page_offset = current_flash_ptr - LOGGER_START_ADDR;
    uint32_t current_page_index = current_page_offset / LOGGER_PAGE_SIZE;
    
    uint32_t next_page_index = (current_page_index + 1) % LOGGER_MAX_PAGES;
    uint32_t next_page_addr = LOGGER_START_ADDR + (next_page_index * LOGGER_PAGE_SIZE);

    uint32_t barrier_page_index = (next_page_index + 1) % LOGGER_MAX_PAGES;
    uint32_t barrier_page_addr = LOGGER_START_ADDR + (barrier_page_index * LOGGER_PAGE_SIZE);
    
    ErasePage(barrier_page_addr);

    current_flash_ptr = next_page_addr;
    current_punch_index = 0;
    
    // Zápis hlavičky závodu: 6 bajtů z CLEAR majáku, RSSI=0, Teplota=20 (zástupné hodnoty pro hlavičku)
    Logger_SavePunch(raw_clear_payload, 0, 20);
    
    APP_DBG(">>> NOVY ZAVOD START! Presun na stranku: %lu (Adresa: 0x%08X)", next_page_index, next_page_addr);
}
