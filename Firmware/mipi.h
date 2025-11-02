// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#define FIRST_HSTX_PIN 12
#define SOTEOTWAIT() asm volatile("nop; nop; nop; nop; nop; nop; nop; nop;")
#define WAITFIFOEMPTY() asm volatile(" nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop;\
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop;\
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop;\
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop;\
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop; \
    nop; nop; nop; nop; nop; nop; nop; nop ; nop; nop; nop; nop; nop; nop; nop; nop")

//clock lane ls pins , can be any pin
#define PIN_LS_CLK_P   19
#define PIN_LS_CLK_N   18
//clock lane hs pins
#define PIN_HS_CLK_P   17
#define PIN_HS_CLK_N   16

//data ls shared with all data lanes
#define PIN_LS_D_P    15
#define PIN_LS_D_N    14
//data lanes hs pins
#define PIN_HS_D0_P   13
#define PIN_HS_D0_N   12
//#define PIN_HS_D1_P   18
//#define PIN_HS_D1_N   19

#if   PIN_HS_CLK_P < FIRST_HSTX_PIN || PIN_HS_CLK_P >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: PIN_HS_CLK_P"
#elif PIN_HS_CLK_N < FIRST_HSTX_PIN || PIN_HS_CLK_N >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: PIN_HS_CLK_N"
#elif PIN_LS_D_P  < FIRST_HSTX_PIN || PIN_LS_D_P  >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: PIN_LS_D_P"
#elif PIN_LS_D_N  < FIRST_HSTX_PIN || PIN_LS_D_N  >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: PIN_LS_D_N"
#elif PIN_HS_D0_P < FIRST_HSTX_PIN || PIN_HS_D0_P >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: PIN_HS_D0_P"
#elif PIN_HS_D0_N < FIRST_HSTX_PIN || PIN_HS_D0_N >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: PIN_HS_D0_N"
//#elif PIN_HS_D1_P  < FIRST_HSTX_PIN || PIN_HS_D1_P  >= FIRST_HSTX_PIN + 8
//#error "Must be an HSTX-capable pin: PIN_HS_D1_P"
//#elif PIN_HS_D1_N  < FIRST_HSTX_PIN || PIN_HS_D1_N  >= FIRST_HSTX_PIN + 8
//#error "Must be an HSTX-capable pin: PIN_HS_D1_N"
#endif

void mipi_init(void);
void mipiCsiSendLong(int type, uint8_t *data, int len);
void mipiCsiFrameStart(void);
void mipiCsiFrameEnd(void);
void DMASetup(uint8_t *data);


/**
 * @brief Die Taktfrequenz des Systems (Core Clock) in Kilohertz (kHz).
 */
#define FCLK_KHZ 330000

/**
 * @brief Anzahl der Taktzyklen, die eine Iteration der Assembler-Warteschleife benötigt.
 * @note Für die Funktion delay_cycles(), typisch 3 für ARM Cortex-M.
 */
#define CYCLES_PER_ASM_LOOP 3

/**
 * @brief Eine simple Funktion, die eine reine Assembler-Schleife ausführt.
 * @param loop_count Anzahl der Schleifendurchläufe.
 *
 * Diese Funktion ist bewusst einfach gehalten, um Toolchain-Probleme zu vermeiden.
 * Der Compiler fügt den notwendigen Funktions-Prolog/Epilog selbst hinzu.
 */
static inline void delay_cycles_loop(uint32_t loop_count) {
    __asm__ volatile (
        "1: \n"
        "   subs %0, #1 \n"      // Zähler dekrementieren
        "   bne 1b \n"           // Springe zurück, wenn nicht null
        : "+r" (loop_count)      // Input/Output: 'loop_count' wird in einem Register übergeben und verändert
        :                        // Keine reinen Inputs
        : "cc"                   // Info für Compiler: Condition-Codes (Flags) werden verändert
    );
}

/**
 * @brief Erzeugt eine Warteschleife mit einer Dauer von 'ns' Nanosekunden.
 * @param ns Die gewünschte Wartezeit in Nanosekunden.
 *
 * BERECHNET die benötigten Taktzyklen und ruft eine optimierte Assembler-Funktion auf.
 * HINWEIS: Die tatsächliche Auflösung hängt vom Systemtakt ab!
 */
#define DELAY_NS(ns) \
    do { \
        /* Berechne die Gesamtzahl der benötigten Taktzyklen. */ \
        uint64_t total_cycles = ((uint64_t)(ns) * FCLK_KHZ) / 1000000; \
        /* Nur ausführen, wenn die Wartezeit mindestens eine Schleifeniteration rechtfertigt */ \
        if (total_cycles >= CYCLES_PER_ASM_LOOP) { \
            delay_cycles_loop(total_cycles / CYCLES_PER_ASM_LOOP); \
        } \
    } while(0)
