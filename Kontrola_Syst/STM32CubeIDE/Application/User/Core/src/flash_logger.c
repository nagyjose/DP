#include "flash_logger.h"
#include "app_common.h"
#include "app_conf.h"
#include "dbg_trace.h"
#include <string.h>

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

void Config_Init(void)
{
	if (DEVICE_CONFIG->magic_word != 0xCAFECAFE)
	{
		APP_DBG("CONFIG: Pamet prazdna nebo poskozena!");
		Config_FactoryReset();
	}
	else
	{
		APP_DBG("CONFIG: Nacteno OK. ID Kontroly: %u, TX Power: %d dBm",
						 DEVICE_CONFIG->stat_device_type, DEVICE_CONFIG->tx_power);
	}
}

// =============================================================================
// ROZHRANÍ PRO BLE TUNEL (Ukládání, Mazání, Vyčítání)
// =============================================================================

void Config_Commit(BeaconConfig_t *new_cfg)
{
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

void Logger_GetDownloadData(uint8_t cmd, uint8_t param, uint8_t **start_ptr, uint32_t *len)
{
	// Kontrola je jen jednoduchý kruhový buffer od začátku
	*start_ptr = (uint8_t*)LOGGER_START_ADDR;
	*len = (current_flash_ptr - LOGGER_START_ADDR);
	APP_DBG("LOGGER: Pripraveno ke stazeni %lu bajtu", *len);
}

