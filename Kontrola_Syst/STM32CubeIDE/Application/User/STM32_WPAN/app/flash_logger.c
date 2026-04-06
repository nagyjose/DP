
#include "flash_logger.h"
#include "dbg_trace.h"
#include "app_conf.h"
#include "app_common.h"
#include <stdbool.h>
#include "stm32wbxx_hal.h"


extern IWDG_HandleTypeDef hiwdg; // Potřebujeme sáhnout na psa v main.c

static uint32_t current_flash_ptr = LOGGER_START_ADDR;
static uint16_t current_punch_index = 0;

// =============================================================================
// VÝPOČET CRC-32 (Standardní IEEE 802.3 polynom)
// =============================================================================
static uint32_t Calculate_CRC32(uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

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
	EraseInitStruct.Page = (page_address - 0x08000000) / 4096;
	EraseInitStruct.NbPages = 1;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock();
}

// 1. INICIALIZACE PŘI BOOTU
void Logger_Init(void)
{
	uint32_t active_page_addr = LOGGER_START_ADDR;
	uint8_t found = 0;

	// Skenujeme první záznam na všech 159 stránkách
	for (int i = 0; i < LOGGER_MAX_PAGES; i++)
	{
		uint32_t page_addr = LOGGER_START_ADDR + (i * LOGGER_PAGE_SIZE);
		uint32_t next_page_addr = LOGGER_START_ADDR + (((i + 1) % LOGGER_MAX_PAGES) * LOGGER_PAGE_SIZE);

		uint64_t current_page_head = *(volatile uint64_t*)page_addr;
		uint64_t next_page_head = *(volatile uint64_t*)next_page_addr;

		// Pokud má aktuální stránka data a ta další je smazaná, jsme na konci logu!
		if (current_page_head != 0xFFFFFFFFFFFFFFFF && next_page_head == 0xFFFFFFFFFFFFFFFF)
		{
			active_page_addr = page_addr;
			found = 1;
			break;
		}
	}

	// Pokud je paměť úplně prázdná nebo zmatená (např. po nahrání nového kódu)
	if (!found) {
		APP_DBG("LOGGER INIT: Pamet neznama, formatuji startovni blok!");
		ErasePage(active_page_addr);
		ErasePage(active_page_addr + LOGGER_PAGE_SIZE);
	}

	// ERASE-AHEAD POJISTKA: Ujistíme se, že stránka hned za naší aktivní je 100% smazaná
	uint32_t active_page_index = (active_page_addr - LOGGER_START_ADDR) / LOGGER_PAGE_SIZE;
	uint32_t next_page_index = (active_page_index + 1) % LOGGER_MAX_PAGES;
	uint32_t next_page_addr = LOGGER_START_ADDR + (next_page_index * LOGGER_PAGE_SIZE);

	if (*(volatile uint64_t*)next_page_addr != 0xFFFFFFFFFFFFFFFF) {
		APP_DBG("LOGGER INIT: Obnovuji ochrannou barieru na strance %lu...", next_page_index);
		ErasePage(next_page_addr); // Tohle se stane jen při startu
	}

	current_flash_ptr = active_page_addr;
	current_punch_index = 0;

	// Najdeme první prázdné místo na aktivní stránce
	while (*(volatile uint64_t*)current_flash_ptr != 0xFFFFFFFFFFFFFFFF)
	{
		current_punch_index++;
		current_flash_ptr += 8;
		if (current_punch_index >= LOGGER_MAX_RECORDS_PP) break;
	}

	APP_DBG("LOGGER INIT: Aktivni stranka: 0x%08X, Zaznamu: %d", active_page_addr, current_punch_index);
}

// 2. DASHCAM ZÁPIS ORAŽENÍ (S automatickým mazáním starých dat)
void Logger_SavePunch_Kontrola(uint8_t* id_3bytes, uint8_t sub_sec, uint32_t unix_time)
{
	// Pokud jsme naplnili aktuální stránku (512 záznamů)
	if (current_punch_index >= LOGGER_MAX_RECORDS_PP)
	{
		// Spočítáme si index stránky, kterou jsme PRÁVĚ DOPSALI
		uint32_t current_page_offset = (current_flash_ptr - 8) - LOGGER_START_ADDR;
		uint32_t current_page_index = current_page_offset / LOGGER_PAGE_SIZE;

		// Stránka N+1 (Na kterou jdeme psát). TA UŽ JE BEZPEČNĚ SMAZANÁ!
		uint32_t next_page_index = (current_page_index + 1) % LOGGER_MAX_PAGES;
		uint32_t next_page_addr = LOGGER_START_ADDR + (next_page_index * LOGGER_PAGE_SIZE);

		// Stránka N+2 (Naše budoucí bariéra)
		uint32_t barrier_page_index = (next_page_index + 1) % LOGGER_MAX_PAGES;
		uint32_t barrier_page_addr = LOGGER_START_ADDR + (barrier_page_index * LOGGER_PAGE_SIZE);

		// JEDNO JEDINÉ MAZÁNÍ (25 ms) -> Mažeme až tu bariéru před námi!
		ErasePage(barrier_page_addr);

		current_flash_ptr = next_page_addr;
		current_punch_index = 0;
	}

	ControlRecord_t record;
	record.data.id_bytes[0] = id_3bytes[0];
	record.data.id_bytes[1] = id_3bytes[1];
	record.data.id_bytes[2] = id_3bytes[2];
	record.data.sub_seconds = sub_sec;
	record.data.unix_time = unix_time;

	uint32_t saved_addr = current_flash_ptr;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_flash_ptr, record.double_word);
	HAL_FLASH_Lock();

	current_flash_ptr += 8;
	current_punch_index++;

	uint32_t runner_id = ((id_3bytes[0] & 0x3F) << 16) | (id_3bytes[1] << 8) | id_3bytes[2];

	APP_DBG("LOG ULOZEN -> Adr: 0x%08X | ID: %lu | Cas: %lu.%d s", saved_addr, runner_id, unix_time, sub_sec);
}

// =============================================================================
// TOVÁRNÍ NASTAVENÍ KONTROLY A ZÁPIS DO FLASH
// =============================================================================
static void EraseConfigPage(void)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = 32; // 0x08020000 odpovídá stránce 32
	EraseInitStruct.NbPages = 1;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock();
}

static void Config_FactoryReset(void)
{
	APP_DBG("CONFIG: Vytvarim tovarni nastaveni Kontroly...");

	// Vytvoříme strukturu v RAM a vynulujeme ji
	BeaconConfig_t def_cfg;
	memset(&def_cfg, 0, sizeof(BeaconConfig_t));

	// --- NAPLNĚNÍ VÝCHOZÍMI HODNOTAMI ---
	def_cfg.magic_word = 0xCAFECAFE;
	strcpy(def_cfg.hw_revision, "Rev 2.1 Kontrola");
	strcpy(def_cfg.BLE_device_name, "KONTROLA_31");
	def_cfg.hash_device = 123456; // Výchozí PIN pro konfiguraci

	// Zásadní parametry pro maják
	def_cfg.stat_device_type = 31; // Výchozí ID kontroly
	def_cfg.tx_power = 2;          // Výkon +2 dBm
	def_cfg.beacon_period_ms = 50; // Maják každých 50 ms
	def_cfg.buzzer_onoff = 1;

	// Výchozí hodnoty pro případnou CLEAR kontrolu
	def_cfg.event_id = 9000;
	def_cfg.event_nation = 1;      // 1 = CZE
	def_cfg.required_rssi = 75;    // Vyžadovaný signál -75 dBm

	strcpy(def_cfg.team_owner, "Vychozi Oddil");

	// =================================================================
	// OPRAVA: Výpočet a uložení CRC-32 před zápisem!
	// =================================================================
	def_cfg.control_sum = 0; // Pro jistotu vynulujeme před výpočtem
	def_cfg.control_sum = Calculate_CRC32((uint8_t*)&def_cfg, sizeof(BeaconConfig_t));

	// --- ZÁPIS DO FLASH PAMĚTI ---
	EraseConfigPage();

	uint8_t *data_ptr = (uint8_t*)&def_cfg;
	uint32_t flash_ptr = CONFIG_FLASH_ADDR;
	uint32_t double_words_count = sizeof(BeaconConfig_t) / 8;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);

	// Zápis se speciální ochranou proti Unaligned Access
	for (uint32_t i = 0; i < double_words_count; i++)
	{
		uint64_t double_word_to_write = 0;
		memcpy(&double_word_to_write, data_ptr + (i * 8), 8);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_ptr, double_word_to_write);
		flash_ptr += 8;
	}

	HAL_FLASH_Lock();
	APP_DBG("CONFIG: Tovarni nastaveni uspesne zapsano!");
}

// =============================================================================
// HLAVNÍ INICIALIZACE A ZÁCHRANNÝ RESET (Voláno z main.c)
// =============================================================================
void Config_Init(void)
{
	// 1. ZÁCHRANNÁ BRZDA: Kontrola tlačítka SW3 při startu desky
	// Tlačítko je Active-Low (Při stisku vrací 0, v klidu 1)
	if (BSP_PB_GetState(BUTTON_SW3) == 0)
	{
		// Krátká prodleva pro odfiltrování zákmitů
		HAL_Delay(50);

		// Pokud je i po 50 ms stále stisknuto, myslí to uživatel vážně
		if (BSP_PB_GetState(BUTTON_SW3) == 0)
		{
			uint8_t hold_time = 0;

			// Cyklus běží POUZE dokud uživatel fyzicky drží tlačítko
			while(BSP_PB_GetState(BUTTON_SW3) == 0) {
				HAL_Delay(100);
				hold_time++;

				// !!! KRITICKÉ: Nakrmíme psa, aby nás během držení nesežral !!!
				HAL_IWDG_Refresh(&hiwdg);

				if(hold_time > 30) { // 30 * 100ms = 3 vteřiny
					APP_DBG(">>> SECURITY: DETEKOVAN ZACHRANNY RESET TLACITKEM! <<<");
					Config_FactoryReset(); // Smaže konfiguraci a vrátí PIN na 123456

					// Zablikáme pro potvrzení úspěchu
					for(int i=0; i<10; i++) { BSP_LED_Toggle(LED_RED); HAL_Delay(50); }
					BSP_LED_Off(LED_RED);
					break; // Vyskočíme z cyklu
				}
			}
		}
	}

	// =========================================================================
	// 2. KONTROLA INTEGRITY PAMĚTI (Magic Word + CRC-32)
	// =========================================================================

	// Zkopírujeme si obsah Flash do RAM, abychom s ním mohli pracovat
	BeaconConfig_t temp_cfg;
	memcpy(&temp_cfg, (void*)DEVICE_CONFIG, sizeof(BeaconConfig_t));

	// Uložíme si přečtené CRC a pole pro výpočet vynulujeme
	uint32_t saved_crc = temp_cfg.control_sum;
	temp_cfg.control_sum = 0;

	// Spočítáme kontrolní součet z dat
	uint32_t calculated_crc = Calculate_CRC32((uint8_t*)&temp_cfg, sizeof(BeaconConfig_t));

	// Pokud nesedí magické slovo NEBO nesedí CRC, paměť je zničená/prázdná
	if (DEVICE_CONFIG->magic_word != 0xCAFECAFE || saved_crc != calculated_crc)
	{
			APP_DBG("CONFIG: Pamet neznama nebo poskozena! (CRC: Ocekavano %08X, Precteno %08X)", calculated_crc, saved_crc);
			Config_FactoryReset();
	}
	else
	{
			APP_DBG("CONFIG: Nacteno OK. Kontrola: %d (Vlastník: %s)",
											 DEVICE_CONFIG->stat_device_type, DEVICE_CONFIG->team_owner);
	}
}

// =============================================================================
// ROZHRANÍ PRO BLE TUNEL (Ukládání, Mazání, Vyčítání)
// =============================================================================

void Config_Commit(BeaconConfig_t *new_cfg)
{
	// =================================================================
	// OPRAVA: Výpočet a uložení CRC-32 před uložením do paměti!
	// =================================================================
	new_cfg->control_sum = 0;
	new_cfg->control_sum = Calculate_CRC32((uint8_t*)new_cfg, sizeof(BeaconConfig_t));

	EraseConfigPage();
	uint8_t *data_ptr = (uint8_t*)new_cfg;
	uint32_t flash_ptr = CONFIG_FLASH_ADDR;
	uint32_t double_words_count = sizeof(BeaconConfig_t) / 8;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
	for (uint32_t i = 0; i < double_words_count; i++) {
		uint64_t double_word_to_write = 0;
		memcpy(&double_word_to_write, data_ptr + (i * 8), 8);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_ptr, double_word_to_write);
		flash_ptr += 8;
	}
	HAL_FLASH_Lock();
	APP_DBG("CONFIG: Nova konfigurace ulozena!");
}

void Config_EraseAndReboot(void)
{
	EraseConfigPage();
	NVIC_SystemReset();
}

void Logger_FormatAll(void)
{
	APP_DBG("LOGGER: Formatuji celou pamet...");
	for (int i = 0; i < LOGGER_MAX_PAGES; i++) {
		ErasePage(LOGGER_START_ADDR + (i * LOGGER_PAGE_SIZE));
	}
	current_flash_ptr = LOGGER_START_ADDR;
	current_punch_index = 0;
	APP_DBG("LOGGER: Formatovani dokonceno!");
}

void System_FactoryResetAll(void)
{
	Logger_FormatAll();
	EraseConfigPage();
	NVIC_SystemReset();
}

// ZMĚNĚNO: parametr 'param' je nyní uint32_t! (Nezapomeň upravit i v hlavičce flash_logger.h)
void Logger_GetDownloadData(uint8_t cmd, uint32_t param, uint8_t **start_ptr, uint32_t *len)
{
	uint32_t ptr = current_flash_ptr;
	uint32_t count = 0;
	uint32_t max_records = (LOGGER_MAX_PAGES * LOGGER_PAGE_SIZE) / 8; // Absolutní maximum
	uint32_t target_count = max_records; // Výchozí stav pro 0x21 (Stáhni VŠE)

	if (cmd == 0x24) { // CMD_DOWNLOAD_LAST_N
		target_count = param;
		if (target_count > max_records) target_count = max_records; // Ochrana proti nesmyslnému N
	}

	// Couváme záznam po záznamu
	while (count < target_count) {
		uint32_t prev_ptr = ptr;

		// Krok zpět s ošetřením přetečení kruhového bufferu
		if (prev_ptr == LOGGER_START_ADDR) {
			prev_ptr = LOGGER_START_ADDR + (LOGGER_MAX_PAGES * LOGGER_PAGE_SIZE);
		}
		prev_ptr -= 8;

		// Kontrola, zda jsme nenarazili na smazanou paměť (Konec reálných historických dat)
		uint64_t val = *(volatile uint64_t*)prev_ptr;
		if (val == 0xFFFFFFFFFFFFFFFF) {
			break;
		}

		// Pro příkaz 0x23 (Od UNIX času) čteme samotný časový údaj
		if (cmd == 0x23) {
			uint32_t record_time;
			// Zkopírujeme natvrdo poslední 4 bajty z 8bajtového záznamu (Time razítko)
			memcpy(&record_time, (void*)(prev_ptr + 4), 4);

			// Upozornění: Pokud do logu ten unix time ukládáš v Big Endian,
			// musíš tady to číslo z 'record_time' otočit zpět, jinak porovnání selže.
			if (record_time < param) {
				break; // Našli jsme záznam starší než požadovaný čas, dál už nejdeme!
			}
		}

		// Záznam vyhovuje, posuneme náš pomyslný startovní ukazatel
		ptr = prev_ptr;
		count++;
	}

	*start_ptr = (uint8_t*)ptr;
	*len = count * 8; // Počet nalezených záznamů x 8 bajtů

	APP_DBG(">>> LOGGER: Pripraveno. Start: 0x%08X, Delka: %lu bajtu (%lu zaznamu)", *start_ptr, *len, count);
}

