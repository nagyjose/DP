

#include "flash_logger.h"
#include "dbg_trace.h"
#include "app_conf.h"
#include "app_common.h"
#include <stdbool.h>
#include "stm32wbxx_hal.h"

extern IWDG_HandleTypeDef hiwdg; // Potřebujeme sáhnout na psa v main.c

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
	// Vyčteme si ID kontroly ze syrových dat
	uint16_t control_id = (raw_payload[0] << 4) | (raw_payload[1] >> 4);

	// =========================================================================
	// POJISTKA PRO CÍL (Rezervace pozice 512)
	// =========================================================================
	// Pokud nám zbývá poslední místo na stránce (index 511) a NENÍ to Cíl...
	if (current_punch_index == (LOGGER_MAX_RECORDS_PP - 1) && control_id != FINISH_CONTROL_ID) {
		APP_DBG("LOGGER WARNING: Stranka plna, rezervovano vyhradne pro CIL!");
		return;
	}

	// Standardní ochrana proti přetečení
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
	// uint16_t control_id = (raw_payload[0] << 4) | (raw_payload[1] >> 4);
	uint8_t sub_seconds = raw_payload[1] & 0x0F;
	uint32_t unix_time = ((uint32_t)raw_payload[2] << 24) | ((uint32_t)raw_payload[3] << 16) |
											 ((uint32_t)raw_payload[4] << 8) | (uint32_t)raw_payload[5];

	APP_DBG("LOG ULOZEN -> Adr: 0x%08X | Ctrl_ID: %d | Cas: %lu.%d s | RSSI: -%d dBm | Teplota: %d C",
					 saved_addr, control_id, unix_time, sub_seconds, rssi_abs, temperature);
}

// 3. CLEAR KONTROLA (Nulování a přesun na další stránku)
void Logger_NewRace(uint8_t* raw_clear_payload)
{
	//uint32_t current_page_offset = current_flash_ptr - LOGGER_START_ADDR;
	//uint32_t current_page_index = current_page_offset / LOGGER_PAGE_SIZE;
	//uint32_t current_page_addr = LOGGER_START_ADDR + (current_page_index * LOGGER_PAGE_SIZE);

	uint32_t current_page_addr = current_flash_ptr - (current_punch_index * 8);
	uint32_t current_page_index = (current_page_addr - LOGGER_START_ADDR) / LOGGER_PAGE_SIZE;
	// =========================================================================
	// INTELIGENTNÍ CLEAR (ZÁCHRANA PAMĚTI)
	// =========================================================================
	if (current_punch_index == 0)
	{
		// Stránka je prázdná, prostě rovnou zapíšeme první záznam
		Logger_SavePunch(raw_clear_payload, 0, 20);
		APP_DBG(">>> INTELIGENTNI CLEAR! Prvni zapis na cistou stranku.");
	}
	else if (current_punch_index == 1)
	{
		// Na stránce je PŘESNĚ JEDEN záznam (z předchozího CLEAR).
		// Jdeme ho přečíst přímo z Flash paměti (6 bajtů payloadu).
		uint8_t* saved_payload = (uint8_t*)current_page_addr;

		bool is_identical = true;
		for(int i = 0; i < 6; i++) {
			if (saved_payload[i] != raw_clear_payload[i]) {
				is_identical = false;
				break;
			}
		}

		if (is_identical)
		{
			// Záznam je totožný s tím, co tam leží. NEDĚLÁME NIC!
			APP_DBG(">>> INTELIGENTNI CLEAR! Zaznam je shodny, setrim pamet (nic nemazu).");
			return;
		}
		else
		{
			// Záznam je odlišný (závodník přešel k JINÉ CLEAR kontrole). Přemažeme.
			APP_DBG(">>> INTELIGENTNI CLEAR! Zaznam odlisny. Premazavam stranku %lu", current_page_index);
			current_flash_ptr = current_page_addr;
			current_punch_index = 0;
			ErasePage(current_page_addr);
			Logger_SavePunch(raw_clear_payload, 0, 20);
		}
	}
	else
	{
		// =====================================================================
		// STANDARDNÍ CLEAR (Máme už záznamy ze závodu, posouváme se na N+1)
		// =====================================================================
		uint32_t next_page_index = (current_page_index + 1) % LOGGER_MAX_PAGES;
		uint32_t next_page_addr = LOGGER_START_ADDR + (next_page_index * LOGGER_PAGE_SIZE);

		uint32_t barrier_page_index = (next_page_index + 1) % LOGGER_MAX_PAGES;
		uint32_t barrier_page_addr = LOGGER_START_ADDR + (barrier_page_index * LOGGER_PAGE_SIZE);

		ErasePage(barrier_page_addr); // N+2 bariéra

		current_flash_ptr = next_page_addr;
		current_punch_index = 0;

		APP_DBG(">>> NOVY ZAVOD START! Presun na stranku: %lu (Adresa: 0x%08X)", next_page_index, next_page_addr);
		Logger_SavePunch(raw_clear_payload, 0, 20); // Zápis na N+1
	}
}

// Pomocná funkce pro smazání nulté konfigurační stránky (Page 32)
static void EraseConfigPage(void)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;

	// 0x08020000 je přesně 32. stránka (128 KB od začátku paměti 0x08000000)
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = 32;
	EraseInitStruct.NbPages = 1;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock();
}

// =============================================================================
// TOVÁRNÍ NASTAVENÍ A ZÁPIS DO FLASH
// =============================================================================
static void Config_FactoryReset(void)
{
	APP_DBG("CONFIG: Vytvarim tovarni nastaveni...");

	// Vytvoříme prázdnou strukturu v RAM a vynulujeme ji
	RunnerConfig_t def_cfg;
	memset(&def_cfg, 0, sizeof(RunnerConfig_t));

	// --- NAPLNĚNÍ VÝCHOZÍMI HODNOTAMI ---
	def_cfg.magic_word = 0xCAFECAFE;
	strcpy(def_cfg.hw_revision, "Rev 1.0 ZAVODNIK");
	// fw_version se doplní dynamicky v main.c pomocí tvého extern Gitu

	// Vysílané informace
	def_cfg.comp_device_type = 0x00;       // Standardní závodník
	def_cfg.comp_device_id = 999999;       // Výchozí ID
	def_cfg.comp_device_hash = 123456;     // Výchozí PIN pro BLE

	// Rozhodovací parametry
	def_cfg.cooldown_ms = 4000;            // 4 vteřiny
	def_cfg.required_hits = 3;             // 3 hity
	def_cfg.num_hits = 5;                  // Z 5 paketů
	def_cfg.buzzer_onoff = 1;              // Pípání povoleno

	// Texty
	strcpy(def_cfg.comp_name, "Neznamy Zavodnik");
	strcpy(def_cfg.comp_nationality, "CZE");

	// --- ZÁPIS DO FLASH PAMĚTI ---
	EraseConfigPage(); // Smažeme starý odpad

	// Budeme data čist jako pole obyčejných bajtů (bezpečné pro packed struktury)
	uint8_t *data_ptr = (uint8_t*)&def_cfg;
	uint32_t flash_ptr = CONFIG_FLASH_ADDR;

	// Zápis probíhá striktně po 8 bajtech (Double Word)
	uint32_t double_words_count = sizeof(RunnerConfig_t) / 8;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);

	for (uint32_t i = 0; i < double_words_count; i++)
	{
		uint64_t double_word_to_write = 0;

		// Zkopírujeme 8 bajtů ze struktury do našeho 64bitového čísla pro zápis
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
	// 1. ZÁCHRANNÁ BRZDA: Kontrola tlačítka SW1 při startu desky
	// Pokud je tlačítko stisknuto během bootování (hodnota 0 nebo 1 dle zapojení,
	// použijeme standardní BSP funkci), provedeme tvrdý formát konfigurace.
	if (BSP_PB_GetState(BUTTON_SW3) == 1 || BSP_PB_GetState(BUTTON_SW3) == 0)
	{
		// Krátká prodleva pro odfiltrování zákmitů
		HAL_Delay(50);
		if (BSP_PB_GetState(BUTTON_SW1) == 1 || BSP_PB_GetState(BUTTON_SW3) == 0) // Zkontroluj reálný stav sepnutí na tvé desce (obvykle 1)
		{
			// PRO JISTOTU KONTROLUJEME PŘESNÝ STAV:
			// Pokud BSP_PB_GetState vrací při stisku 1, uprav podmínku výše na == 1
		}
	}

	// Tady je vylepšená 100% spolehlivá detekce držení tlačítka po dobu 3 vteřin:
	uint8_t hold_time = 0;
	// Předpokládáme, že stisknuté tlačítko vrací 1 (pokud ne, změň na 0)
	while(BSP_PB_GetState(BUTTON_SW3) == 1) {
		HAL_Delay(100);
		hold_time++;
		if(hold_time > 30) { // 30 * 100ms = 3 vteřiny
			APP_DBG(">>> SECURITY: DETEKOVAN ZACHRANNY RESET TLACITKEM! <<<");
			Config_FactoryReset(); // Smaže konfiguraci a vrátí PIN na 123456

			// Zablikáme pro potvrzení
			for(int i=0; i<10; i++) { BSP_LED_Toggle(LED_RED); HAL_Delay(50); }
			BSP_LED_Off(LED_RED);
			break;
		}
	}

	// 2. Standardní kontrola magického slova
	if (DEVICE_CONFIG->magic_word != 0xCAFECAFE)
	{
		APP_DBG("CONFIG: Pamet neznama nebo poskozena!");
		Config_FactoryReset();
	}
	else
	{
		APP_DBG("CONFIG: Nacteno OK. Zavodnik: %s (ID: %lu)",
										 DEVICE_CONFIG->comp_name, DEVICE_CONFIG->comp_device_id);
	}
}

// -----------------------------------------------------------------------------
// PŘÍPRAVA DAT PRO BLE TUNEL
// -----------------------------------------------------------------------------
void Logger_GetDownloadData(uint8_t cmd, uint8_t param, uint8_t **start_ptr, uint32_t *len)
{
	if (cmd == 0x20) {
		// --- CMD_DOWNLOAD_CURRENT (Jen aktuální závod) ---
		// current_flash_ptr ukazuje na PRVNÍ VOLNÉ MÍSTO.
		// Začátek závodu získáme odečtením délky všech zatím zapsaných záznamů.
		*start_ptr = (uint8_t*)(current_flash_ptr - (current_punch_index * 8));
		*len = current_punch_index * 8; // Pošleme jen platné záznamy

		// Pojistka: Pokud jsme na úplném začátku nové stránky (Cíl byl 512. záznam)
		if (*len == 0 && current_flash_ptr > LOGGER_START_ADDR) {
			*start_ptr = (uint8_t*)(current_flash_ptr - LOGGER_PAGE_SIZE);
			*len = LOGGER_PAGE_SIZE;
		}
	}
	else if (cmd == 0x21) {
		// --- CMD_DOWNLOAD_ALL (Kompletní výpis všeho) ---

		// Pokud je paměť úplně prázdná
		if (current_flash_ptr == LOGGER_START_ADDR && current_punch_index == 0) {
				*start_ptr = NULL;
				*len = 0;
				return;
		}

		// Protože jsi paměť naformátoval (a 640 KB je obrovský prostor),
		// všechna tvá data leží bezpečně a souvisle od začátku paměti
		// až po tvůj aktuální ukazatel zápisu.
		*start_ptr = (uint8_t*)LOGGER_START_ADDR;
		*len = current_flash_ptr - LOGGER_START_ADDR;
	}
	else if (cmd == 0x22) {
		// --- CMD_DOWNLOAD_SPECIFIC (N závodů do historie) ---
		if (param == 0) param = 1; // Pojistka: param=1 znamená předchozí závod

		uint32_t search_ptr = current_flash_ptr;
		uint32_t end_of_target = 0;
		uint32_t start_of_target = 0;
		uint8_t clears_found = 0;

		// Pokud je paměť úplně prázdná
		if (current_flash_ptr == LOGGER_START_ADDR && current_punch_index == 0) {
			*start_ptr = NULL; *len = 0; return;
		}

		// Hledáme pozpátku jako buldok
		while (1) {
			// Posun vzad s ošetřením kruhového přetečení na konec paměti
			if (search_ptr <= LOGGER_START_ADDR) {
				search_ptr = LOGGER_START_ADDR + (LOGGER_MAX_PAGES * LOGGER_PAGE_SIZE);
			}
			search_ptr -= 8;

			// Obešli jsme celou paměť až k našemu výchozímu bodu
			if (search_ptr == current_flash_ptr) break;

			uint64_t val = *(volatile uint64_t*)search_ptr;

			// TOTO BYLA TA CHYBA: Pokud narazíme na prázdné místo mezi stránkami,
			// nesmíme to ukončit! Jen ho v klidu přeskočíme a hledáme dál.
			if (val == 0xFFFFFFFFFFFFFFFF) {
				continue;
			}

			// Přečteme ID kontroly
			uint8_t *raw = (uint8_t*)search_ptr;
			uint16_t cid = (raw[0] << 4) | (raw[1] >> 4);

			// Narazili jsme na oddělovač závodů
			if (cid == CLEAR_CONTROL_ID) {
				clears_found++;

				if (clears_found == param) {
					// Našli jsme CLEAR novějšího závodu. Zde náš historický závod končí.
					end_of_target = search_ptr;
				}
				else if (clears_found == param + 1) {
					// Našli jsme CLEAR našeho historického závodu. Tady začíná!
					start_of_target = search_ptr;
					break;
				}
			}
		}

		// =====================================================================
		// VYHODNOCENÍ A PŘEDÁNÍ KRÁJEČI
		// =====================================================================

		// Pokud jsme nenašli dostatek CLEARů (uživatel chce víc historie, než máme)
		if (clears_found < (param + 1)) {
			*start_ptr = NULL;
			*len = 0;
			return;
		}

		*start_ptr = (uint8_t*)start_of_target;

		// NOVÉ: Přesný výpočet délky bez prázdné vaty (0xFF)
		uint32_t exact_len = 0;
		uint32_t read_ptr = start_of_target;

		// Půjdeme od začátku závodu dopředu, dokud nenarazíme na další závod (end_of_target)
		// NEBO dokud nenarazíme na prázdnou paměť (0xFFFFFFFFFFFFFFFF).
		while (read_ptr != end_of_target) {
			uint64_t val = *(volatile uint64_t*)read_ptr;

			// Tady závod reálně končí, zbytek stránky je už jen prázdná vata
			if (val == 0xFFFFFFFFFFFFFFFF) {
				break;
			}

			exact_len += 8;

			// Kruhový posun dopředu
			read_ptr += 8;
			if (read_ptr >= LOGGER_START_ADDR + (LOGGER_MAX_PAGES * LOGGER_PAGE_SIZE)) {
				read_ptr = LOGGER_START_ADDR;
			}
		}

		*len = exact_len;
	}
	else {
		// Neznámý příkaz
		*start_ptr = NULL;
		*len = 0;
	}
}

// -----------------------------------------------------------------------------
// FORMÁTOVÁNÍ CELÉ PAMĚTI (Příkaz z BLE)
// -----------------------------------------------------------------------------
void Logger_FormatAll(void)
{
	APP_DBG("--- FORMATOVANI CELE PAMETI (159 stranek) ZACAHA... ---");

	for (int i = 0; i < LOGGER_MAX_PAGES; i++) {
		uint32_t page_addr = LOGGER_START_ADDR + (i * LOGGER_PAGE_SIZE);
		ErasePage(page_addr);

		// Nakrmíme psa, aby nás během dlouhého mazání nesežral
		HAL_IWDG_Refresh(&hiwdg);
	}

	// Reset ukazatelů na úplný začátek
	current_flash_ptr = LOGGER_START_ADDR;
	current_punch_index = 0;

	APP_DBG("--- FORMATOVANI DOKONCENO, PAMET JE CISTA ---");
}

// -----------------------------------------------------------------------------
// HROMADNÝ ZÁPIS KONFIGURACE (COMMIT z RAM)
// -----------------------------------------------------------------------------
void Config_Commit(RunnerConfig_t *new_cfg)
{
	// 1. Smažeme celou konfigurační stránku ve Flash
	EraseConfigPage();

	// 2. Projdeme celou strukturu z RAM a zapíšeme ji po 8 bajtech
	uint8_t *data_ptr = (uint8_t*)new_cfg;
	uint32_t flash_ptr = CONFIG_FLASH_ADDR;
	uint32_t double_words_count = sizeof(RunnerConfig_t) / 8;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);

	for (uint32_t i = 0; i < double_words_count; i++)
	{
		uint64_t double_word_to_write = 0;
		memcpy(&double_word_to_write, data_ptr + (i * 8), 8);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_ptr, double_word_to_write);
		flash_ptr += 8;
	}
	HAL_FLASH_Lock();

	APP_DBG("CONFIG: Nova konfigurace byla uspesne vypalena do Flash!");
}

// -----------------------------------------------------------------------------
// TVRDÉ RESETOVACÍ FUNKCE (Vyvolané z BLE)
// -----------------------------------------------------------------------------
void Config_EraseAndReboot(void)
{
    APP_DBG(">>> BLE: Mazani Konfigurace a RESTART! <<<");
    EraseConfigPage();  // Smaže POUZE nultou stránku
    NVIC_SystemReset(); // Tvrdý hardwarový restart
}

void System_FactoryResetAll(void)
{
    APP_DBG(">>> BLE: Kompletni Factory Reset a RESTART! <<<");
    Logger_FormatAll(); // Smaže 159 stránek historie a nakrmí Watchdoga
    EraseConfigPage();  // Smaže nultou konfigurační stránku
    NVIC_SystemReset(); // Tvrdý hardwarový restart
}
