#include "app_nbiot.h"
#include "stm32_seq.h"
#include "app_conf.h"
#include "dbg_trace.h"
#include "stm_logging.h"
#include <string.h>
#include "stm32wbxx_hal.h"
#include <stdio.h>


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
	NB_STATE_WAITING_ACK    // Čekáme, až nám náš server odpoví bajtem 0x01
} NBIOT_State_t;

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
		__set_PRIMASK(primask_bit);
		APP_DBG(">>> NBIOT FIFO ERROR: Fronta je plna! Zaznam ztracen pro cloud.");
		return false; // Ztráta dat pro cloud (v lese ale ve Flash zůstanou bezpečně)
	}

	// Zkopírujeme data
	memcpy(nbiot_fifo.buffer[nbiot_fifo.head].runner_raw, runner_raw, 3);
	nbiot_fifo.buffer[nbiot_fifo.head].timestamp = timestamp;

	// Posuneme ukazatel
	nbiot_fifo.head = (nbiot_fifo.head + 1) % NBIOT_FIFO_SIZE;
	nbiot_fifo.count++;

	__set_PRIMASK(primask_bit);

	// Tímto probudíme hlavní NBIOT Task v Sequenceru (automat pro Quectel)
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
// HLAVNÍ NBIOT TASK (Mozek operace, běží asynchronně v Sequenceru)
// =============================================================================
void NBIOT_Process_Task(void)
{
	switch(current_nb_state)
	{
		case NB_STATE_SLEEP:
			// Jsme uspaní. Podíváme se, jestli něco nepřistálo ve frontě
			if (NBIOT_FIFO_GetCount() > 0) {
				APP_DBG("NBIOT AUTOMAT: Nova data ve fronte! Zapinam modul...");

				// Vyndáme nejstarší záznam z fronty a uložíme si ho do pracovní proměnné
				NBIOT_FIFO_Pop(&active_record);

				NBIOT_PowerOn_Pulse(); // Fyzicky nakopneme Quectel pinu
				current_nb_state = NB_STATE_WAKING_UP;

				// Nyní task končí. Procesor jde spát a my čekáme, až nás RX Callback
				// probudí zprávou "RDY" (Ready) od modulu.
			}
			break;

		case NB_STATE_WAKING_UP:
			// Sem procesor vstoupí ve chvíli, kdy modul něco napsal na sériovou linku
			if (strstr((char*)nbiot_rx_buffer, "RDY") != NULL || strstr((char*)nbiot_rx_buffer, "OK") != NULL) {
				APP_DBG("NBIOT AUTOMAT: Modul nabootoval. Zapinam radio (CFUN=1).");
				NBIOT_Send_AT("AT+CFUN=1");
				current_nb_state = NB_STATE_INIT_NETWORK;
			}
			break;

		case NB_STATE_INIT_NETWORK:
			// Kontrolujeme, zda se připojil k operátorovi (AT+CGATT)
			// Zde později dopíšeme logiku navazování sítě
			APP_DBG("NBIOT AUTOMAT: Jsme ve stavu Init Network...");
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

		// Pro debug vymažeme zbytečné konce řádků pro hezčí výpis
		for(int i = 0; i < Size; i++) {
			if (nbiot_rx_buffer[i] == '\r' || nbiot_rx_buffer[i] == '\n') nbiot_rx_buffer[i] = ' ';
		}
		APP_DBG("<<< NBIOT RX: %s", nbiot_rx_buffer);

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

