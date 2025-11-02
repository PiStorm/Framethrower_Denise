// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#pragma GCC optimize("O3")

// =============================================================================
// --- Includes ---
// =============================================================================
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pll.h"
#include "hardware/structs/qmi.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include <stdlib.h>
#include <string.h>

#include "fifo.h"
#include "mipi.h"
#include "video_capture.pio.h"

// =============================================================================
// --- Definitionen & Globale Variablen ---
// =============================================================================
#define LINES_PER_FRAME 288
#define LINES_PER_FRAME_NTSC 240
#define ACTIVE_VIDEO 720
#define HBLANK 72
#define SAMPLES_PER_LINE 720
#define VBLANK_LINES 14

// GPIO Pin-Definitionen
#define csync 23
#define pixelclock 22
#define cck 21

// Globale Puffer und FIFO
__attribute__((aligned(4))) uint16_t framebuffer[ACTIVE_VIDEO * LINES_PER_FRAME];
__attribute__((aligned(4))) uint16_t line1[ACTIVE_VIDEO];
__attribute__((aligned(4))) uint16_t line2[ACTIVE_VIDEO];
__attribute__((aligned(4))) uint16_t temp_scanline[ACTIVE_VIDEO];
__attribute__((aligned(4))) uint16_t blackline [ACTIVE_VIDEO];

extern volatile bool mipi_busy;

// PIO Globals
PIO pio_video = pio0;
PIO pio_rga = pio1;

uint sm_video, sm_vsync;
uint sm_rga_read, sm_rga_write;
volatile uint16_t rga_1F4 = 0;
volatile uint16_t rga_1F6 = 0;

// Interrupt Global
volatile bool vsync_detected = false;
volatile uint32_t lines;
volatile uint32_t last_total_lines;
volatile bool laced = false;
volatile bool isPAL = true;
volatile bool video_go = false;
volatile bool vsync_go = true;
volatile bool is_odd_field = true;
volatile bool isPAL_prev = true;
volatile bool clear_screen = false;

volatile int scanline_level = 4;
volatile int scanline_level_laced = 4;

// =============================================================================
// --- Interrupt Service Routine ---
// =============================================================================

// Wird bei VSYNC vom PIO aufgerufen
void __not_in_flash_func(pio_irq_handler)() {
 
    if (pio_interrupt_get(pio_video, 0)) {
        pio_interrupt_clear(pio_video, 0); 
        vsync_detected = true;
        laced = last_total_lines == lines  ? false : true;
        isPAL = last_total_lines <= 300 ? false : true;
        if(isPAL_prev &  !isPAL)clear_screen=true; else clear_screen=false;
        isPAL_prev = isPAL;
        is_odd_field = (lines % 2 != 0);
        last_total_lines = lines;
        lines = 0;
    }

    if (pio_interrupt_get(pio_video, 1)) {
        pio_interrupt_clear(pio_video, 1); 
        lines++;   
    }
}


//TODO -> RGB444 zu RGB565 kann eventuell der HSTX Expander-Block machen
//Optimized to 9 cycles -> ~35Mpix/sec @ 320Mhz
static inline uint16_t __attribute__((always_inline)) __not_in_flash_func(convert_12_to_565_reordered_optimized)(uint16_t pixel_in) {
    
    uint32_t result, temp1, temp2;

    __asm volatile (
        /* --- Schritt 1: Blau-Komponente bearbeiten (3 Instruktionen) --- */
        // Extrahiere 4-Bit Blau (Bits 8-11) in das Ergebnisregister.
        "ubfx   %[res], %[in], #8, #4        \n\t"
        // Skaliere Blau auf 5 Bit durch Replikation des höchstwertigen Bits (MSB).
        // bbbbb = (bbbb << 1) | (bbbb >> 3)
        "lsr    %[t1], %[res], #3            \n\t"
        "orr    %[res], %[t1], %[res], lsl #1 \n\t"
        
        /* --- Schritt 2: Grün-Komponente bearbeiten (3 Instruktionen) --- */
        // Extrahiere 4-Bit Grün (Bits 4-7).
        "ubfx   %[t1], %[in], #4, #4        \n\t"
        // Skaliere Grün auf 6 Bit durch Replikation der zwei MSBs.
        // gggggg = (gggg << 2) | (gggg >> 2)
        "lsr    %[t2], %[t1], #2            \n\t"
        "orr    %[t1], %[t2], %[t1], lsl #2 \n\t"
        
        /* --- Schritt 3: Rot-Komponente vorbereiten (1 Instruktion) --- */
        // Extrahiere 4-Bit Rot (Bits 0-3). Dies geschieht jetzt schon,
        // damit die CPU-Pipeline die Zeit nutzen kann, während sie auf
        // das Ergebnis der Grün-Skalierung wartet.
        "ubfx   %[t2], %[in], #0, #4        \n\t"

        /* --- Schritt 4: Finale Kombination (4 Instruktionen) --- */
        // Füge das skalierte Grün (gggggg) an Bit-Position 5 in das Ergebnis ein.
        "bfi    %[res], %[t1], #5, #6        \n\t"
        // Skaliere Rot auf 5 Bit.
        "lsr    %[t1], %[t2], #3            \n\t"
        "orr    %[t2], %[t1], %[t2], lsl #1 \n\t"
        // Füge das skalierte Rot (rrrrr) an Bit-Position 11 in das Ergebnis ein.
        "bfi    %[res], %[t2], #11, #5       \n\t"

        // Definition der Operanden für den Compiler
        : [res] "=&r" (result), [t1] "=&r" (temp1), [t2] "=&r" (temp2)
        : [in] "r" (pixel_in)
        // Es werden keine weiteren Register clobbered, da alle temporären
        // Register als Outputs deklariert sind.
    );

    return (uint16_t)result;
}

void __not_in_flash_func(set_brightness_fast_levels)(uint16_t* line, int count, int level) {

    //if (level >= 4) {
    //    return;
    //}

    uint32_t* p32 = (uint32_t*)line;
    int loop_count = count >> 1;

    switch (level) {
        case 4:{
            return;
            };
            break;

        case 3: { // ~75% Helligkeit
            const uint32_t mask25 = 0xE79CE79C;
            __asm volatile (
                ".p2align 2\n"
                "1:\n"
                // Lade 2 Original-Pixel (100%)
                "ldr r0, [%[ptr]]\n"
                // Berechne die 25%-Version
                "and r1, r0, %[mask]\n"
                "lsr r1, r1, #2\n"
                // Subtrahiere 25% von 100% für 2 Pixel gleichzeitig!
                "usub16 r0, r0, r1\n"
                // Schreibe Ergebnis zurück und erhöhe Zeiger
                "str r0, [%[ptr]], #4\n"
                // Schleifenkontrolle
                "subs %[n], #1\n"
                "bne 1b\n"
                : [ptr] "+&r"(p32), [n] "+&r"(loop_count)
                : [mask] "r"(mask25)
                : "r0", "r1", "cc", "memory"
            );
            break;
        }
        case 2: { // 50% Helligkeit
            const uint32_t mask50 = 0xF7DEF7DE;
            for (int i = 0; i < loop_count; ++i) {
                p32[i] = (p32[i] & mask50) >> 1;
            }
            break;
        }
        case 1: { // 25% Helligkeit
            const uint32_t mask25 = 0xE79CE79C;
            for (int i = 0; i < loop_count; ++i) {
                p32[i] = (p32[i] & mask25) >> 2;
            }
            break;
        }
        case 0: { // 0% Helligkeit
            for (int i = 0; i < loop_count; ++i) {
                p32[i] = 0;
            }
            break;
        }
    }

    // Ggf. das letzte ungerade Pixel behandeln
    if (count & 1) {
        uint16_t p = line[count - 1];
        switch (level) {
            case 3: line[count-1] = p - ((p & 0xE79C) >> 2); break;
            case 2: line[count-1] = (p & 0xF7DE) >> 1; break;
            case 1: line[count-1] = (p & 0xE79C) >> 2; break;
            case 0: line[count-1] = 0; break;
        }
    }
}

void __not_in_flash_func(get_pio_line)(uint16_t* line_buffer) {
    pio_sm_put_blocking(pio_video, sm_video, ((SAMPLES_PER_LINE+HBLANK) / 2 )- 1);
    pio_sm_set_enabled(pio_video, sm_video, true);

    //Small hack to join RX and TX fifo on the fly, increases RX fifo to 8 entries
    pio_sm_exec(pio_video, sm_video, pio_encode_pull(false,false));
    hw_set_bits(&pio_video->sm[sm_video].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    pio_sm_exec(pio_video, sm_video, pio_encode_mov( pio_x , pio_osr));

    for (uint i = 0; i < HBLANK; ++i) {
        (void)pio_sm_get_blocking(pio_video, sm_video);
    }


    for (uint i = 0; i < SAMPLES_PER_LINE; ++i) {
        line_buffer[i] = convert_12_to_565_reordered_optimized(pio_sm_get_blocking(pio_video, sm_video));
    }
    //Small hack to join RX and TX fifo on the fly, flips the join bit back to preload OSR for the next run
    hw_clear_bits(&pio_video->sm[sm_video].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    pio_sm_set_enabled(pio_video, sm_video, false);
}

// =============================================================================
// --- PIO Setup ---
// =============================================================================
void setup_vsync_detect_sm(uint offset) {
    sm_vsync = pio_claim_unused_sm(pio_video, true);
    pio_sm_config c = vsync_detect_program_get_default_config(offset);
    sm_config_set_clkdiv(&c, 34.8f); //PIO SM für Vsync läuft mit ~10MHz
    pio_gpio_init(pio_video, csync);
    gpio_set_dir(csync, GPIO_IN);
    sm_config_set_jmp_pin(&c, csync);
    sm_config_set_in_pins(&c, csync);
    pio_sm_init(pio_video, sm_vsync, offset, &c);
    pio_sm_set_enabled(pio_video, sm_vsync, true);
}

void setup_video_capture_sm(uint offset) {
    sm_video = pio_claim_unused_sm(pio_video, true);
    pio_sm_config c = video_capture_program_get_default_config(offset);
    sm_config_set_in_pins(&c, 0);
    sm_config_set_in_shift(&c, true,true,32);
    pio_sm_set_consecutive_pindirs(pio_video, sm_video, 0, 32, false);
    sm_config_set_wrap(&c, offset + video_capture_wrap_target, offset + video_capture_wrap);
    sm_config_set_in_shift(&c, true, true, 1);
    pio_sm_init(pio_video, sm_video, offset, &c);
    pio_sm_set_enabled(pio_video, sm_video, false);
}


// =============================================================================
// --- Core 1 Entry ---
// =============================================================================
void core1_entry() {
    // Konfiguriere den PIO-Interrupt für VSYNC
    pio_set_irq0_source_enabled(pio_video, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    pio_set_irq1_source_enabled(pio_video, pis_interrupt1, true);
    irq_set_exclusive_handler(PIO0_IRQ_1, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_1, true);


    uint16_t y = 0;

    while (1) {
        if (vsync_detected) {
            vsync_detected = false; 
            vsync_go = true;

            //Vblank abwarten
            if(!isPAL) {while (lines <= (is_odd_field? VBLANK_LINES : VBLANK_LINES-1)){}} else       //NTSC
                       {while (lines <= (!(last_total_lines % 2)? VBLANK_LINES : VBLANK_LINES-1)){}} //PAL 
    
            // Aktive Videozeilen einlesen
            for (y = 0; y < (isPAL? LINES_PER_FRAME-1 : LINES_PER_FRAME_NTSC-1); y++) {
                get_pio_line(line1);
                video_go=true;
                while(!video_go){}
            }
        }
    }
}


// =============================================================================
// --- Main Funktion (Core 0) ---
// =============================================================================
int __not_in_flash_func(main)(void) {


    //Flash divider und OC
    uint clkdiv = 3;
    uint rxdelay = 3;
    hw_write_masked(
        &qmi_hw->m[0].timing,
        ((clkdiv << QMI_M0_TIMING_CLKDIV_LSB) & QMI_M0_TIMING_CLKDIV_BITS) |
        ((rxdelay << QMI_M0_TIMING_RXDELAY_LSB) & QMI_M0_TIMING_RXDELAY_BITS),
        QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS
    );
    __asm__ __volatile__("dmb sy");
    set_sys_clock_khz(348000, true);

    // Initialisiere alle GPIOs
    for (uint i = 0; i <= 47; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_input_hysteresis_enabled(i, true);
        gpio_pull_up(i);
    }

    gpio_init(csync);
    gpio_set_dir(csync, GPIO_IN);
    gpio_pull_up(pixelclock);
    gpio_set_input_hysteresis_enabled(csync, true);

    gpio_init(pixelclock);
    gpio_set_dir(pixelclock, GPIO_IN);
    gpio_pull_up(pixelclock);
    gpio_set_input_hysteresis_enabled(pixelclock, true);

    gpio_init(cck);
    gpio_set_dir(cck, GPIO_IN);
    gpio_pull_up(cck);
    gpio_set_input_hysteresis_enabled(cck, true);

    // PIO-Programme laden und State Machines einrichten
    uint offset_vsync = pio_add_program(pio_video, &vsync_detect_program);
    uint offset_video = pio_add_program(pio_video, &video_capture_program);
    setup_vsync_detect_sm(offset_vsync);
    setup_video_capture_sm(offset_video);
    pio_sm_put_blocking(pio_video, sm_video, ((SAMPLES_PER_LINE+HBLANK) / 2) - 1);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    multicore_launch_core1(core1_entry);

    mipi_init();
    DMASetup(0);

    uint32_t lines_read_count = 0;
    bool frame_active = false;

    while (1) {

        if (!frame_active) {
            // Warten auf den Beginn eines neuen Frames
            if(vsync_go){
                vsync_go = false;    
                while(mipi_busy){}mipiCsiFrameStart();
                frame_active = true;
                lines_read_count = 0;
            }
        } else {
            // Frame wird aktiv gelesen und gesendet
            if(video_go){
                memcpy(line2,line1, ACTIVE_VIDEO*2);
                video_go=false; 

                if(laced) {
                    if(isPAL){
                        if (is_odd_field) {
                            memcpy(temp_scanline,line2,ACTIVE_VIDEO*2);
                            set_brightness_fast_levels(temp_scanline, ACTIVE_VIDEO,scanline_level_laced);
                            mipiCsiSendLong(0x22, (uint8_t*)temp_scanline, ACTIVE_VIDEO*2);
                            while(mipi_busy){} mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                        } else {
                            set_brightness_fast_levels(framebuffer + (ACTIVE_VIDEO * lines_read_count), ACTIVE_VIDEO,scanline_level_laced); 
                            mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                            while(mipi_busy){} mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                        }
                    }else{
                        if (!is_odd_field) {                       
                            memcpy(temp_scanline,line2,ACTIVE_VIDEO*2);
                            set_brightness_fast_levels(temp_scanline, ACTIVE_VIDEO,scanline_level_laced); 
                            mipiCsiSendLong(0x22, (uint8_t*)temp_scanline, ACTIVE_VIDEO*2);
                            while(mipi_busy){} mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                        } else {                            
                            set_brightness_fast_levels(framebuffer + (ACTIVE_VIDEO * lines_read_count), ACTIVE_VIDEO,scanline_level_laced); 
                            mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                            while(mipi_busy){} mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                        }
                    }
                    memcpy(framebuffer + (ACTIVE_VIDEO * lines_read_count),line2, ACTIVE_VIDEO*2);
                } else {
                    mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                    set_brightness_fast_levels(line2, ACTIVE_VIDEO,scanline_level);
                    while(mipi_busy){} mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                }

                lines_read_count++;

                // Prüfen, ob der Frame vollständig übertragen wurde
                if (lines_read_count >= (isPAL ? LINES_PER_FRAME-1 : LINES_PER_FRAME_NTSC-1)) {
                    mipiCsiSendLong(0x22, (uint8_t*)blackline, ACTIVE_VIDEO*2);
                    while(mipi_busy){} mipiCsiSendLong(0x22, (uint8_t*)blackline, ACTIVE_VIDEO*2);
                    while(mipi_busy){} mipiCsiFrameEnd();
                    frame_active = false;
                }
            }
        }
    }
}
