#include "getTime.h"

volatile uint32_t startTime = 0;
volatile uint32_t nowTime = 0;

static volatile uint32_t s_systickOverflow = 0;

void SysTick_Handler(void) {
    s_systickOverflow++;
}

void TimeBase_Init(void) {
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

static uint64_t getExtendedTicks(void) {
    uint32_t overflow1, overflow2;
    uint32_t current;
    uint32_t period = SysTick->LOAD + 1U;

    do {
        overflow1 = s_systickOverflow;
        current = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
        overflow2 = s_systickOverflow;
    } while (overflow1 != overflow2);

    return ((uint64_t) overflow1 * period) + ((period - 1U) - current);
}

uint32_t getNowUs(void) {
    uint64_t ticks = getExtendedTicks();
    return (uint32_t) (ticks / (CPUCLK_FREQ / 1000000U));
}

uint32_t getNowMs(void) {
    uint64_t ticks = getExtendedTicks();
    return (uint32_t) (ticks / (CPUCLK_FREQ / 1000U));
}

uint32_t getTimeUs(uint32_t nowUs, uint32_t lastUs) {
    return nowUs - lastUs;
}

uint32_t getTimeMs(uint32_t nowMs, uint32_t lastMs) {
    return nowMs - lastMs;
}