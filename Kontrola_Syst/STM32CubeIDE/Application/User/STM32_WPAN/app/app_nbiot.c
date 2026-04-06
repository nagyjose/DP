#include "app_nbiot.h"
#include "stm32_seq.h"
#include "app_conf.h"
#include "dbg_trace.h"
#include "stm_logging.h"
#include "utilities_common.h"
#include <string.h>
#include "stm32wbxx_hal.h"
#include <stdio.h>
#include <stdlib.h> // Pro funkci atoi()
#include "p2p_server_app.h" // Pro System_Execute_Command

// SMAZÁNO: extern uint16_t Get_Battery_Voltage(void);
// PŘIDÁNO: Přístup k unifikovanému měření z MAC vrstvy
extern void Get_ADC_Measurements(int8_t *out_temp, uint16_t *out_batt_mv);

// --- NOVÉ PROMĚNNÉ PRO STATUS A BATERII ---
static uint8_t last_csq = 99;         // 99 znamená "Neznámý signál"
static bool is_nbiot_dead = false;    // Vlajka pro trvalé vypnutí modulu
static uint8_t NbiotHeartbeatTimerId; // Časovač pro pravidelný Status


// =============================================================================
// HARDWARE DEFINICE (Uprav podle fyzického zapojení desky!)
// =============================================================================
#define NBIOT_TX_PORT           GPIOA
#define NBIOT_TX_PIN            GPIO_PIN_2   // LPUART1 TX
#define NBIOT_RX_PORT           GPIOA
#define NBIOT_RX_PIN            GPIO_PIN_3   // LPUART1 RX

#define NBIOT_PWRKEY_PORT       GPIOB
#define NBIOT_PWRKEY_PIN        GPIO_PIN_5   // Pin pro zapnutí modulu

#define NBIOT_RX_BUFFER_SIZE    256          // Velikost kruhového bufferu pro odpovědi modulu

// Globální handlery pro HAL
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

// Buffer, kam nám bude DMA sypat data bez účasti procesoru
uint8_t nbiot_rx_buffer[NBIOT_RX_BUFFER_SIZE];

// Dopředná deklarace funkcí
void NBIOT_PowerOn_Pulse(void);

// =============================================================================
// STAVY NBIOT AUTOMATU
// =============================================================================
typedef enum {
	NB_STATE_SLEEP = 0,     // Modul spí, fronta je prázdná
	NB_STATE_WAKING_UP,     // Čekáme na nabootování modulu (hláška RDY)
	NB_STATE_INIT_NETWORK,  // Připojování do sítě (AT+CGATT?)
	NB_STATE_OPEN_SOCKET,   // Otevírání UDP spojení
	NB_STATE_SENDING,       // Fyzické odesílání 10 bajtů z fronty
	NB_STATE_REQ_SEND,      // Čekáme na '>' z modulu
	NB_STATE_WAITING_ACK,    // Čekáme, až nám náš server odpoví bajtem 0x01
	NB_STATE_READ_DOWNLINK,  // <--- PŘIDÁNO (Čtení dat ze socketu)
	NB_STATE_GET_CSQ
} NBIOT_State_t;

static uint8_t udp_payload[128]; // Buffer pro slepení závodníků
static uint16_t udp_payload_len = 0;

static NBIOT_State_t current_nb_state = NB_STATE_SLEEP;
static NBIOT_PunchRecord_t active_record; // Záznam, který se zrovna pokoušíme odeslat

// Pomocná funkce pro rychlé odeslání AT příkazu (přidá \r\n automaticky)
static void NBIOT_Send_AT(const char *cmd) {
	char tx_buf[64];
	snprintf(tx_buf, sizeof(tx_buf), "%s\r\n", cmd);
	APP_DBG(">>> NBIOT TX: %s", cmd); // Vypíše do terminálu, ať vidíme, co se děje

	// Odesíláme blokujícím způsobem, protože TX při 9600 baud je velmi rychlé (jednotky ms)
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)tx_buf, strlen(tx_buf), 100);
}

// Kapacita pro 64 ražení (bohatě stačí pro případ, že modul chvíli hledá síť)
#define NBIOT_FIFO_SIZE 64

typedef struct {
	NBIOT_PunchRecord_t buffer[NBIOT_FIFO_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint16_t count;
} NBIOT_FIFO_t;

static NBIOT_FIFO_t nbiot_fifo;

// =============================================================================
// ČASOVAČE A STAVOVÉ PROMĚNNÉ PRO BATCHING A BACKOFF
// =============================================================================
static uint8_t NbiotBackoffTimerId;
static uint8_t NbiotBatchTimerId;
static uint8_t NbiotPollTimerId; // Časovač pro dotazování (AT+CGATT?)

static bool is_backoff_active = false; // Běží 5minutová penalizace?
static bool is_batch_active = false;   // Běží 10vteřinové okno pro nasbírání dat?
static uint8_t cgatt_retries = 0;      // Kolikrát jsme se marně ptali na síť

static void Timer_Heartbeat_Cb(void) {
	// Pokud už jsme se kvůli baterii trvale vypnuli, Heartbeat neposíláme
	if (is_nbiot_dead) return;

	// Vytvoříme "fiktivního" závodníka (3 bajty 0xFF znamenají Status)
	uint8_t dummy_runner[3] = {0xFF, 0xFF, 0xFF};

	// Vhodíme ho do fronty (čas dáme 0, server si doplní vlastní čas přijetí)
	NBIOT_FIFO_Push(dummy_runner, 0);

	APP_DBG("NBIOT: Pravidelny Heartbeat vlozen do fronty.");

	// Znovu spustit za 1 hodinu (3 600 000 ms)
	HW_TS_Start(NbiotHeartbeatTimerId, (uint32_t)(3600000ULL * 1000ULL / CFG_TS_TICK_VAL));
}

static void Timer_Backoff_Cb(void) {
	is_backoff_active = false;
	APP_DBG("NBIOT: Penalizace (5 min) vyprsela. Zkusime to znovu!");
	UTIL_SEQ_SetTask(1 << CFG_TASK_NBIOT_PROCESS, CFG_SCH_PRIO_0);
}

static void Timer_Batch_Cb(void) {
	is_batch_active = false;
	APP_DBG("NBIOT: Davkovaci okno (10 s) uzavreno. Jdeme vysilat!");
	UTIL_SEQ_SetTask(1 << CFG_TASK_NBIOT_PROCESS, CFG_SCH_PRIO_0);
}

static void Timer_Poll_Cb(void) {
	// Pokud jsme ve fázi čekání na ACK a vypršel timeout (5 vteřin), server neodpověděl!
	if (current_nb_state == NB_STATE_WAITING_ACK ||
			current_nb_state == NB_STATE_OPEN_SOCKET ||
			current_nb_state == NB_STATE_READ_DOWNLINK) {
		APP_DBG("NBIOT TIMEOUT: Modul neodpovedel! Zacinam BACKOFF (5 min).");
		NBIOT_Send_AT("AT+QPOWD=1"); // Zavřít krám

		is_backoff_active = true;
		HW_TS_Start(NbiotBackoffTimerId, (uint32_t)(300000 * 1000 / CFG_TS_TICK_VAL));
		current_nb_state = NB_STATE_SLEEP;
	} else {
		// Běžné probuzení k dalšímu dotazu (např. AT+CGATT?)
		UTIL_SEQ_SetTask(1 << CFG_TASK_NBIOT_PROCESS, CFG_SCH_PRIO_0);
	}
}

// =============================================================================
// BEZPEČNOSTNÍ HASH (Anti-Spoofing pro UDP)
// =============================================================================
static uint32_t Calculate_Payload_Hash(uint8_t *data, uint16_t len, uint32_t secret_key) {
	// Inicializujeme hash naším tajným PINem z Flash paměti
	uint32_t hash = secret_key;

	// Postupně "zamícháme" každý bajt odesílané zprávy
	for (uint16_t i = 0; i < len; i++) {
		hash ^= data[i];
		hash *= 0x5bd1e995; // Magická konstanta z Murmur3
		hash ^= hash >> 15;
	}
	return hash;
}


void NBIOT_FIFO_Init(void) {
	nbiot_fifo.head = 0;
	nbiot_fifo.tail = 0;
	nbiot_fifo.count = 0;
}

// VLOŽENÍ DO FRONTY (Voláno z rychlého MAC přerušení)
bool NBIOT_FIFO_Push(uint8_t *runner_raw, uint32_t timestamp) {
	// Zakážeme na zlomek mikrosekundy přerušení pro bezpečný zápis
	uint32_t primask_bit = __get_PRIMASK();
	__disable_irq();

	if (nbiot_fifo.count >= NBIOT_FIFO_SIZE) {
		// PŘETEČENÍ: Fronta je plná! Zahodíme nejstarší záznam posunutím tail
		nbiot_fifo.tail = (nbiot_fifo.tail + 1) % NBIOT_FIFO_SIZE;
		// count se nezvyšuje, zůstává na maximu
		APP_DBG(">>> NBIOT FIFO: Plno! Prepisuji nejstarsi zaznam.");
	} else {
		nbiot_fifo.count++;
	}

	// Zkopírujeme data
	memcpy(nbiot_fifo.buffer[nbiot_fifo.head].runner_raw, runner_raw, 3);
	nbiot_fifo.buffer[nbiot_fifo.head].timestamp = timestamp;

	// Posuneme ukazatel hlavy
	nbiot_fifo.head = (nbiot_fifo.head + 1) % NBIOT_FIFO_SIZE;

	__set_PRIMASK(primask_bit);

	// Probudíme hlavní NBIOT Task
	UTIL_SEQ_SetTask(1 << CFG_TASK_NBIOT_PROCESS, CFG_SCH_PRIO_0);

	return true;
}

// VYJMUTÍ Z FRONTY (Voláno z pomalého Quectel automatu)
bool NBIOT_FIFO_Pop(NBIOT_PunchRecord_t *out_record) {
	uint32_t primask_bit = __get_PRIMASK();
	__disable_irq();

	if (nbiot_fifo.count == 0) {
		__set_PRIMASK(primask_bit);
		return false;
	}

	*out_record = nbiot_fifo.buffer[nbiot_fifo.tail];
	nbiot_fifo.tail = (nbiot_fifo.tail + 1) % NBIOT_FIFO_SIZE;
	nbiot_fifo.count--;

	__set_PRIMASK(primask_bit);
	return true;
}

uint16_t NBIOT_FIFO_GetCount(void) {
	return nbiot_fifo.count;
}

// =============================================================================
// HLAVNÍ NBIOT TASK (Asynchronní stavový automat)
// =============================================================================
void NBIOT_Process_Task(void)
{
	switch(current_nb_state)
	{
		case NB_STATE_SLEEP:
			// Pokud jsme mrtví a fronta je prázdná, absolutně nic neděláme
			if (is_nbiot_dead && NBIOT_FIFO_GetCount() == 0) return;

			// 1. BEZPEČNOSTNÍ KONTROLA BATERIE PŘED JAKOUKOLIV AKCÍ
			if (!is_nbiot_dead) {
				uint16_t threshold_mv = DEVICE_CONFIG->battery_alert_threshold * 100;

				// NOVÉ: Vyčtení obou hodnot
				int8_t current_temp;
				uint16_t current_batt_mv;
				Get_ADC_Measurements(&current_temp, &current_batt_mv);

				if (threshold_mv > 0 && current_batt_mv < threshold_mv) {
					APP_DBG("!!! BATERIE KRITICKA (%d mV)! NBIOT odesila umiracek a TRVALE SE VYPINA !!!", current_batt_mv);
					is_nbiot_dead = true; // Uzamčení modulu

					// Vhodíme do fronty umělý Heartbeat, který se ihned odešle jako poslední zpráva
					uint8_t dummy_runner[3] = {0xFF, 0xFF, 0xFF};
					NBIOT_FIFO_Push(dummy_runner, 0);
				}
			}

			// 2. BĚŽNÁ KONTROLA FRONTY
			if (is_backoff_active || is_batch_active) return;

			if (NBIOT_FIFO_GetCount() > 0) {
				APP_DBG("NBIOT AUTOMAT: Mam data k odeslani, zapinam modul...");
				NBIOT_PowerOn_Pulse();
				current_nb_state = NB_STATE_WAKING_UP;
			}
			break;

		case NB_STATE_WAKING_UP:
			if (strstr((char*)nbiot_rx_buffer, "RDY") != NULL || strstr((char*)nbiot_rx_buffer, "OK") != NULL) {
				APP_DBG("NBIOT: Boot OK. Nastavuji APN a zapinam radio...");
				nbiot_rx_buffer[0] = '\0';

				// 1. Nastavíme kontext sítě s APN z naší Flash paměti
				char cmd[64];
				snprintf(cmd, sizeof(cmd), "AT+QICSGP=1,1,\"%s\"", DEVICE_CONFIG->nbiot_apn);
				NBIOT_Send_AT(cmd);

				// 2. Zapneme rádiovou část modulu
				NBIOT_Send_AT("AT+CFUN=1");

				// Příprava na hledání sítě
				cgatt_retries = 0;
				current_nb_state = NB_STATE_INIT_NETWORK;

				// Zeptáme se na síť poprvé až za 3 sekundy
				HW_TS_Start(NbiotPollTimerId, (uint32_t)(3000 * 1000 / CFG_TS_TICK_VAL));
			}
			break;

		case NB_STATE_INIT_NETWORK:
			if (strstr((char*)nbiot_rx_buffer, "+CGATT: 1") != NULL) {
				APP_DBG("NBIOT: >>> SIT NALEZENA! Zjistuji silu signalu... <<<");
				nbiot_rx_buffer[0] = '\0';

				// Zeptáme se na sílu signálu POUZE JEDNOU při připojení
				NBIOT_Send_AT("AT+CSQ");
				current_nb_state = NB_STATE_GET_CSQ;

			} else if (strstr((char*)nbiot_rx_buffer, "+CGATT: 0") != NULL || nbiot_rx_buffer[0] == '\0') {
				cgatt_retries++;
				if (cgatt_retries > 15) { // 15 pokusů = 30 vteřin hledání sítě
					APP_DBG("NBIOT ERROR: Sit v lese nenalezena! Zacinam BACKOFF (5 min).");
					NBIOT_Send_AT("AT+QPOWD=1"); // Tvrdě ho vypneme

					is_backoff_active = true;
					HW_TS_Start(NbiotBackoffTimerId, (uint32_t)(300000 * 1000 / CFG_TS_TICK_VAL));
					current_nb_state = NB_STATE_SLEEP;
				} else {
					nbiot_rx_buffer[0] = '\0';
					NBIOT_Send_AT("AT+CGATT?");
					HW_TS_Start(NbiotPollTimerId, (uint32_t)(2000 * 1000 / CFG_TS_TICK_VAL));
				}
			}
			break;

		case NB_STATE_GET_CSQ:
			if (strstr((char*)nbiot_rx_buffer, "+CSQ:") != NULL) {
				// Modul odpoví např. "+CSQ: 18,99"
				char *ptr = strstr((char*)nbiot_rx_buffer, ": ");
				if (ptr) {
					last_csq = atoi(ptr + 2); // Vysekne první číslo (sílu signálu)
				}
				nbiot_rx_buffer[0] = '\0';
				APP_DBG("NBIOT: Sila signalu (CSQ) = %d", last_csq);

				// Nyní otevřeme UDP Socket
				char cmd[128];
				snprintf(cmd, sizeof(cmd), "AT+QIOPEN=1,0,\"UDP\",\"%s\",%d,0,1",
								 DEVICE_CONFIG->nbiot_server_ip, DEVICE_CONFIG->nbiot_server_port);
				NBIOT_Send_AT(cmd);

				current_nb_state = NB_STATE_OPEN_SOCKET;
				HW_TS_Start(NbiotPollTimerId, (uint32_t)(10000 * 1000 / CFG_TS_TICK_VAL));
			}
			break;

		case NB_STATE_OPEN_SOCKET:
			if (strstr((char*)nbiot_rx_buffer, "+QIOPEN: 0,0") != NULL) {
				APP_DBG("NBIOT: Socket Otevren! Pripravuji data k odeslani...");
				nbiot_rx_buffer[0] = '\0';

				udp_payload_len = 0;
				uint16_t my_control_id = DEVICE_CONFIG->stat_device_type;

				while(NBIOT_FIFO_Pop(&active_record)) {
					if (udp_payload_len + 10 > sizeof(udp_payload)) break;

					// --- ROZLIŠENÍ: ZÁVODNÍK vs. STATUS (HEARTBEAT) ---
					if (active_record.runner_raw[0] == 0xFF && active_record.runner_raw[1] == 0xFF) {

						// --- B) STATUS PAKET (0x02) ---
						udp_payload[udp_payload_len++] = 0x02; // MSG_TYPE = STATUS
						udp_payload[udp_payload_len++] = (my_control_id >> 8) & 0xFF;
						udp_payload[udp_payload_len++] = my_control_id & 0xFF;

						// NOVÉ: Čteme aktuální data z ADC
						int8_t temp_c;
						uint16_t bat_mv;
						Get_ADC_Measurements(&temp_c, &bat_mv);

						// Zápis baterie
						udp_payload[udp_payload_len++] = (bat_mv >> 8) & 0xFF;
						udp_payload[udp_payload_len++] = bat_mv & 0xFF;

						// Zápis signálu
						udp_payload[udp_payload_len++] = last_csq;

						// Zápis TEPLOTY (Bajt č. 6)
						udp_payload[udp_payload_len++] = (uint8_t)temp_c;

						// Zbylé 3 bajty dáme na 0x00 (Server si čas doplní sám)
						udp_payload[udp_payload_len++] = 0x00;
						udp_payload[udp_payload_len++] = 0x00;
						udp_payload[udp_payload_len++] = 0x00;
					} else {
						// --- A) BĚŽNÝ ZÁVODNÍK (0x01) ---
						udp_payload[udp_payload_len++] = 0x01; // MSG_TYPE = ZAVODNIK
						udp_payload[udp_payload_len++] = (my_control_id >> 8) & 0xFF;
						udp_payload[udp_payload_len++] = my_control_id & 0xFF;

						udp_payload[udp_payload_len++] = active_record.runner_raw[0];
						udp_payload[udp_payload_len++] = active_record.runner_raw[1];
						udp_payload[udp_payload_len++] = active_record.runner_raw[2];

						udp_payload[udp_payload_len++] = (active_record.timestamp >> 24) & 0xFF;
						udp_payload[udp_payload_len++] = (active_record.timestamp >> 16) & 0xFF;
						udp_payload[udp_payload_len++] = (active_record.timestamp >> 8) & 0xFF;
						udp_payload[udp_payload_len++] = active_record.timestamp & 0xFF;
					}
				}

				// =================================================================
				// VIP: PŘIDÁNÍ BEZPEČNOSTNÍHO PODPISU (HASH)
				// =================================================================
				// Vypočítáme podpis z aktuálních dat, klíčem je náš tajný PIN
				uint32_t packet_hash = Calculate_Payload_Hash(udp_payload, udp_payload_len, DEVICE_CONFIG->hash_device);

				// Přilepíme 4 bajty podpisu na úplný konec zprávy
				udp_payload[udp_payload_len++] = (packet_hash >> 24) & 0xFF;
				udp_payload[udp_payload_len++] = (packet_hash >> 16) & 0xFF;
				udp_payload[udp_payload_len++] = (packet_hash >> 8)  & 0xFF;
				udp_payload[udp_payload_len++] = packet_hash & 0xFF;

								// Odeslání do modulu Quectel
				char cmd[32];
				snprintf(cmd, sizeof(cmd), "AT+QISEND=0,%d", udp_payload_len);
				NBIOT_Send_AT(cmd);
				current_nb_state = NB_STATE_REQ_SEND;

			} else if (strstr((char*)nbiot_rx_buffer, "+QIOPEN: 0,") != NULL) {
				// Chyba otevírání (např. +QIOPEN: 0,561)
				APP_DBG("NBIOT ERROR: Nelze otevrit socket! Uspavam modul.");
				NBIOT_Send_AT("AT+QPOWD=1");
				current_nb_state = NB_STATE_SLEEP; // Zkusí to při dalším závodníkovi znovu
			}
			break;

		case NB_STATE_REQ_SEND:
			// Modul pošle znak '>', čímž říká: "Nasyp do mě ty surové bajty"
			if (strchr((char*)nbiot_rx_buffer, '>') != NULL) {
				APP_DBG("NBIOT: Odesilam %d bajtu do eteru...", udp_payload_len);
				nbiot_rx_buffer[0] = '\0';

				// Odeslání samotných binárních dat (bez \r\n na konci!)
				HAL_UART_Transmit(&hlpuart1, udp_payload, udp_payload_len, 1000);

				current_nb_state = NB_STATE_WAITING_ACK;
				// Server má 5 vteřin na to, aby poslal potvrzení
				HW_TS_Start(NbiotPollTimerId, (uint32_t)(5000 * 1000 / CFG_TS_TICK_VAL));
			}
			break;

		case NB_STATE_WAITING_ACK:
			// Server potvrdil příjem
			if (strstr((char*)nbiot_rx_buffer, "+QIURC: \"recv\"") != NULL) {
				APP_DBG("NBIOT: >>> ACK OD SERVERU! Zjistuji, zda poslal i nejaka data...");
				nbiot_rx_buffer[0] = '\0';

				// Požádáme modul, ať nám ze svého vnitřního bufferu "vyklopí" co přišlo
				NBIOT_Send_AT("AT+QIRD=0");

				current_nb_state = NB_STATE_READ_DOWNLINK;
				// Máme 5 vteřin na to, abychom z modulu ta data vydolovali
				HW_TS_Start(NbiotPollTimerId, (uint32_t)(5000 * 1000 / CFG_TS_TICK_VAL));
			}
			break;

		case NB_STATE_READ_DOWNLINK:
			// Modul pošle text "+QIRD: <delka>" a hned na dalším řádku čistá data
			if (strstr((char*)nbiot_rx_buffer, "+QIRD:") != NULL) {

				char *qird_ptr = strstr((char*)nbiot_rx_buffer, "+QIRD:");
				int downlink_len = atoi(qird_ptr + 6); // Přečte číslo délky za textem

				if (downlink_len > 0) {
					APP_DBG("NBIOT: Ze serveru prislo %d bajtu! Predavam jadru...", downlink_len);

					// Najdeme začátek samotných binárních dat (hledáme první Enter \n)
					char *data_start = strchr(qird_ptr, '\n');

					if (data_start != NULL) {
						data_start++; // Posuneme ukazatel o 1 znak za ten Enter

						// =========================================================
						// MAGIE ZDE: Surová data hodíme do stejného procesoru jako BLE!
						// Parametr '1' znamená, že zdroj je NB-IoT.
						// =========================================================
						System_Execute_Command((uint8_t*)data_start, downlink_len, 1);
					}
				} else {
					APP_DBG("NBIOT: Server sice odpovedel, ale neposlal zadny prikaz.");
				}

				// Vše je hotovo, teď teprve můžeme jít klidně spát!
				nbiot_rx_buffer[0] = '\0';
				APP_DBG("NBIOT: Uspavam modul a spoustim BATCH (10s)");
				NBIOT_Send_AT("AT+QPOWD=1");

				is_batch_active = true;
				HW_TS_Start(NbiotBatchTimerId, (uint32_t)(10000 * 1000 / CFG_TS_TICK_VAL));
				current_nb_state = NB_STATE_SLEEP;
			}
			break;

		default:
			break;
	}
}

// =============================================================================
// INICIALIZACE LPUART1 A DMA PRO NB-IOT
// =============================================================================
void NBIOT_Hardware_Init(void)
{
	// 1. POVOLENÍ HODIN (Clocks)
	__HAL_RCC_LPUART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	// 2. KONFIGURACE PINU PWRKEY (Výstupní pin pro zapínání modulu)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = NBIOT_PWRKEY_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NBIOT_PWRKEY_PORT, &GPIO_InitStruct);

	// Výchozí stav: PWRKEY uvolněný (obvykle HIGH, modul má vnitřní pull-up)
	HAL_GPIO_WritePin(NBIOT_PWRKEY_PORT, NBIOT_PWRKEY_PIN, GPIO_PIN_SET);

	// 3. KONFIGURACE PINŮ LPUART1 (TX a RX)
	GPIO_InitStruct.Pin = NBIOT_TX_PIN | NBIOT_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1; // AF8 je pro LPUART na STM32WB
	HAL_GPIO_Init(NBIOT_TX_PORT, &GPIO_InitStruct);

	// 4. KONFIGURACE LPUART1 (9600 Baudrate je pro NB-IoT nejspolehlivější a nejúspornější)
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 9600;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
			APP_DBG("CHYBA: NBIOT LPUART1 Init selhal!");
	}

	// 5. KONFIGURACE DMA PRO PŘÍJEM (RX)
	hdma_lpuart1_rx.Instance = DMA1_Channel1; // Použijeme volný kanál 1
	hdma_lpuart1_rx.Init.Request = DMA_REQUEST_LPUART1_RX; // Propojení DMAMUX
	hdma_lpuart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_lpuart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_lpuart1_rx.Init.MemInc = DMA_MINC_ENABLE; // Zapisujeme do pole
	hdma_lpuart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_lpuart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_lpuart1_rx.Init.Mode = DMA_CIRCULAR; // MAGIE: Kruhový buffer (nikdy se nezastaví)
	hdma_lpuart1_rx.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_lpuart1_rx) != HAL_OK) {
			APP_DBG("CHYBA: NBIOT DMA Init selhal!");
	}
	__HAL_LINKDMA(&hlpuart1, hdmarx, hdma_lpuart1_rx);

	// 6. POVOLENÍ PŘERUŠENÍ
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(LPUART1_IRQn);

	// 7. SPUŠTĚNÍ ASYNCHRONNÍHO PŘÍJMU (IDLE LINE DETECTION)
	HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, nbiot_rx_buffer, NBIOT_RX_BUFFER_SIZE);

	UTIL_SEQ_RegTask(1 << CFG_TASK_NBIOT_PROCESS, UTIL_SEQ_RFU, NBIOT_Process_Task);

	// Založení časovačů pro logiku sítě
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &NbiotBackoffTimerId, hw_ts_SingleShot, Timer_Backoff_Cb);
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &NbiotBatchTimerId, hw_ts_SingleShot, Timer_Batch_Cb);
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &NbiotPollTimerId, hw_ts_SingleShot, Timer_Poll_Cb);

	// --- NOVÝ ČASOVAČ PRO HEARTBEAT (STATUS) ---
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &NbiotHeartbeatTimerId, hw_ts_SingleShot, Timer_Heartbeat_Cb);
	// Rovnou ho poprvé nastartujeme, ať pošle první stavový paket za 1 hodinu (3 600 000 ms)
	HW_TS_Start(NbiotHeartbeatTimerId, (uint32_t)(3600000ULL * 1000ULL / CFG_TS_TICK_VAL));

	APP_DBG(">>> NBIOT HARDWARE: Inicializovan (LPUART1 + DMA). Cekam na data.");
}

// =============================================================================
// OVLÁDÁNÍ NAPÁJENÍ (PWRKEY)
// =============================================================================
void NBIOT_PowerOn_Pulse(void)
{
	APP_DBG(">>> NBIOT: Posilam zapinaci pulz na PWRKEY...");

	// Stáhneme pin na LOW (Zapínáme)
	HAL_GPIO_WritePin(NBIOT_PWRKEY_PORT, NBIOT_PWRKEY_PIN, GPIO_PIN_RESET);

	// Potřebujeme držet LOW cca 500-1000 ms (Quectel specifikace)
	// Protože toto voláme z našeho Sequencer Tasku (a ne z přerušení), můžeme bezpečně
	// použít HAL_Delay. Během tohoto Delay MAC vrstva vESELE běží dál a ukládá do Flash!
	HAL_Delay(800);

	// Uvolníme pin zpět na HIGH
	HAL_GPIO_WritePin(NBIOT_PWRKEY_PORT, NBIOT_PWRKEY_PIN, GPIO_PIN_SET);
}

// =============================================================================
// ZACHYCENÍ ODPOVĚDI OD MODULU (Voláno z hlubin HAL knihovny přes DMA)
// =============================================================================
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == LPUART1)
	{
		// Ochrana proti přetečení bufferu
		if (Size < NBIOT_RX_BUFFER_SIZE) {
			nbiot_rx_buffer[Size] = '\0'; // Ukončovací znak pro práci s textem (strstr)
		} else {
			nbiot_rx_buffer[NBIOT_RX_BUFFER_SIZE - 1] = '\0';
		}

		// =========================================================
		// OPRAVA: Čištění textu děláme, jen když nečekáme binární data!
		// =========================================================
		if (current_nb_state != NB_STATE_READ_DOWNLINK) {
			for(int i = 0; i < Size; i++) {
				if (nbiot_rx_buffer[i] == '\r' || nbiot_rx_buffer[i] == '\n') nbiot_rx_buffer[i] = ' ';
			}
			APP_DBG("<<< NBIOT RX: %s", nbiot_rx_buffer);
		} else {
			APP_DBG("<<< NBIOT RX: [Prijat raw blok o delce %d bajtu]", Size);
		}

		// ZPRÁVA KOMPLETNÍ: Probudíme hlavní NBIOT Task v Sequenceru!
		UTIL_SEQ_SetTask(1 << CFG_TASK_NBIOT_PROCESS, CFG_SCH_PRIO_0);

		// Znovu spustíme DMA naslouchání pro další zprávu
		HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, nbiot_rx_buffer, NBIOT_RX_BUFFER_SIZE);
	}
}

// Systémové hardwarové handlery (Musí tu být, aby přerušení nezahučelo do prázdna)
void DMA1_Channel1_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
}
void LPUART1_IRQHandler(void) {
	HAL_UART_IRQHandler(&hlpuart1);
}

// =============================================================================
// NÁSILNÉ VYPNUTÍ MODULU (Voláno při přechodu do BLE nebo Storage)
// =============================================================================
void NBIOT_Force_Sleep(void) {
	APP_DBG(">>> NBIOT: Vynucene vypnuti modulu (System jde spat) <<<");

	// Pro jistotu pošleme slušný povel k vypnutí (pokud UART zrovna žije)
	NBIOT_Send_AT("AT+QPOWD=1");

	// Zastavíme všechny NB-IoT časovače
	HW_TS_Stop(NbiotBatchTimerId);
	HW_TS_Stop(NbiotBackoffTimerId);
	HW_TS_Stop(NbiotPollTimerId);
	HW_TS_Stop(NbiotHeartbeatTimerId);

	// Natvrdo přejdeme do stavu SLEEP
	current_nb_state = NB_STATE_SLEEP;
	is_nbiot_dead = true; // Zabráníme dalšímu probuzení, dokud se systém neresetuje
}

