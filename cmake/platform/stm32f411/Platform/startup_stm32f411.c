/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * Minimal STM32F411 Startup & Vector Table
 * ============================================================================
 */

#include <stdint.h>
#include <stddef.h>
#include <errno.h>

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

/* ----------------------------------------------------------------------------
 * Dynamic Destructor Stubs
 * Prevent __register_exitproc dynamic allocation hangs before/during main
 * ---------------------------------------------------------------------------- */
__attribute__((used))
int __cxa_atexit(void (*destructor)(void *), void *arg, void *dso) {
    (void)destructor;
    (void)arg;
    (void)dso;
    return 0;
}

__attribute__((used))
int __aeabi_atexit(void *object, void (*destructor)(void *), void *dso_handle) {
    (void)object;
    (void)destructor;
    (void)dso_handle;
    return 0;
}

__attribute__((used))
int atexit(void (*fn)(void)) {
    (void)fn;
    return 0;
}

/* ----------------------------------------------------------------------------
 * Heap Management (_sbrk)
 * Ensures heap bounds checking against _ebss and _estack/stack pointer
 * ---------------------------------------------------------------------------- */
void* _sbrk(ptrdiff_t incr) {
    static uint8_t *heap_end = NULL;
    uint8_t *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = (uint8_t *)&_ebss;
    }

    prev_heap_end = heap_end;

    register uint8_t *sp __asm__("sp");

    // Heap grows upwards towards stack pointer.
    // Ensure heap does not collide with stack or exceed top of RAM (_estack).
    if (((incr > 0) && ((heap_end + incr > sp) || (heap_end + incr > (uint8_t *)&_estack))) ||
        ((incr < 0) && (heap_end + incr < (uint8_t *)&_ebss))) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}

void Default_Handler(void) {
    while (1);
}

void Reset_Handler(void);
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector"), used))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))(&_estack),
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};

void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    // Call static constructors
    extern void __libc_init_array(void);
    __libc_init_array();

    // Call main
    extern int main(int argc, char** argv);
    main(0, (void*)0);

    while (1);
}
