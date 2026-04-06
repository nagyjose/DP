#include "app_nbiot.h"
#include "stm32_seq.h"
#include "app_conf.h"
#include "dbg_trace.h"
#include <string.h>
#include "stm32wbxx_hal.h"

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
