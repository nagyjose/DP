#include "flash_logger.h"
#include "dbg_trace.h"

// Globální proměnné pro rychlý zápis z RAM
static uint32_t current_flash_ptr = LOGGER_START_ADDR;
static uint16_t current_punch_index = 0;

// Interní pomocná funkce pro smazání 4KB stránky
static void ErasePage(uint32_t page_address)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

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
void Logger_SavePunch(uint16_t control_id, uint32_t unix_time)
{
    // Ochrana před přetečením stránky (>512 kontrol v jednom závodu)
    if (current_punch_index >= LOGGER_MAX_RECORDS_PP) {
        APP_DBG("LOGGER ERROR: Stranka je plna!");
        return; 
    }

    PunchRecord_t record;
    record.data.index = current_punch_index + 1; // +1 aby se to lépe četlo lidem (1, 2, 3...)
    record.data.control_id = control_id;
    record.data.unix_time = unix_time;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_flash_ptr, record.double_word);
    HAL_FLASH_Lock();

    current_flash_ptr += 8;
    current_punch_index++;
    
    APP_DBG("LOG ULOZEN -> IDX: %d, ID: %d", record.data.index, control_id);
}

// 3. CLEAR KONTROLA (Nulování a přesun na další stránku)
void Logger_NewRace(uint32_t start_time)
{
    // Zjistíme na jaké stránce jsme
    uint32_t current_page_offset = current_flash_ptr - LOGGER_START_ADDR;
    uint32_t current_page_index = current_page_offset / LOGGER_PAGE_SIZE;
    
    // Posun na další stránku (Kruhový buffer)
    uint32_t next_page_index = (current_page_index + 1) % LOGGER_MAX_PAGES;
    uint32_t next_page_addr = LOGGER_START_ADDR + (next_page_index * LOGGER_PAGE_SIZE);

    // Aby to fungovalo po havárii, musíme SMAZAT ještě stránku za tou naší novou,
    // abychom vytvořili onu bariéru (N+2) ze samých jedniček.
    uint32_t barrier_page_index = (next_page_index + 1) % LOGGER_MAX_PAGES;
    uint32_t barrier_page_addr = LOGGER_START_ADDR + (barrier_page_index * LOGGER_PAGE_SIZE);
    
    ErasePage(barrier_page_addr); // Vytvoříme zeď

    // Nastavíme se na novou stránku
    current_flash_ptr = next_page_addr;
    current_punch_index = 0;
    
    // Zapíšeme hlavičku závodu (nulté oražení)
    Logger_SavePunch(CLEAR_CONTROL_ID, start_time);
    
    APP_DBG("NOVY ZAVOD START! Stranka: %d", next_page_index);
}
