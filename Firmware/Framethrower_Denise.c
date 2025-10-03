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
#define VIDEO_LINE_LENGTH 720
#define LINES_PER_FRAME 288
#define LINES_PER_FRAME_NTSC 240
#define ACTIVE_VIDEO 720
#define HBLANK 72
#define SAMPLES_PER_LINE 720
#define VBLANK_LINES 14

// GPIO Pin-Definitionen
#define csync 23
#define pixelclock 22

// Globale Puffer und FIFO
__attribute__((aligned(4))) uint16_t *framebuffer;
__attribute__((aligned(4))) uint16_t line1[VIDEO_LINE_LENGTH];
__attribute__((aligned(4))) uint16_t line2[VIDEO_LINE_LENGTH];
__attribute__((aligned(4))) uint16_t blackline [VIDEO_LINE_LENGTH];


// PIO Globals
PIO pio = pio0;
uint sm_video, sm_vsync;

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

// =============================================================================
// --- Interrupt Service Routine ---
// =============================================================================

// Wird bei VSYNC vom PIO aufgerufen
void __not_in_flash_func(pio_irq_handler)() {
 
    if (pio_interrupt_get(pio, 0)) {
        pio_interrupt_clear(pio, 0); 
        vsync_detected = true;
        laced = last_total_lines == lines  ? false : true;
        isPAL = last_total_lines <= 300 ? false : true;
        if(isPAL_prev &  !isPAL)clear_screen=true; else clear_screen=false;
        isPAL_prev = isPAL;
        is_odd_field = (lines % 2 != 0);
        last_total_lines = lines;
        lines = 0;
    }

    if (pio_interrupt_get(pio, 1)) {
        pio_interrupt_clear(pio, 1); 
        lines++;   
    }
}


// =============================================================================
// --- Hilfsfunktionen ---
// =============================================================================

//TODO -> RGB444 zu RGB565 kann eventuell der HSTX Expander-Block machen
static inline uint16_t __not_in_flash_func(convert_12_to_565_reordered_fast)(uint16_t pixel_in) {
    uint32_t result;
    uint32_t r, g, b, temp; // Temporäre Register

    __asm volatile (
        // --- Schritt 1: Parallele Extraktion aller Komponenten ---
        // Diese 3 Befehle sind voneinander unabhängig und können von der
        // CPU-Pipeline überlappend ausgeführt werden.
        "ubfx   %[r], %[in], #0, #4        \n\t" // r = R (rrrr)
        "ubfx   %[g], %[in], #4, #4        \n\t" // g = G (gggg)
        "ubfx   %[b], %[in], #8, #4        \n\t" // b = B (bbbb)

        // --- Schritt 2: Skalierung & initiales Packen ---
        // Skaliere B auf 5 Bit und schreibe es als erstes ins Ergebnis.
        "lsr    %[tmp], %[b], #3           \n\t" // tmp = MSB von B
        "orr    %[out], %[tmp], %[b], lsl #1 \n\t" // result = bbbbb

        // Skaliere G auf 6 Bit.
        "lsr    %[tmp], %[g], #2           \n\t" // tmp = 2 MSBs von G
        "orr    %[g], %[tmp], %[g], lsl #2 \n\t" // g = gggggg

        // Skaliere R auf 5 Bit.
        "lsr    %[tmp], %[r], #3           \n\t" // tmp = MSB von R
        "orr    %[r], %[tmp], %[r], lsl #1 \n\t" // r = rrrrr

        // --- Schritt 3: Finales Zusammensetzen ---
        // Die skalierten Werte sind nun fertig und können nacheinander
        // in das Ergebnisregister eingefügt werden.
        "bfi    %[out], %[g], #5, #6       \n\t" // Füge 6-Bit G an Position 5 ein
        "bfi    %[out], %[r], #11, #5      \n\t" // Füge 5-Bit R an Position 11 ein

        : [out] "=&r" (result), [r] "=&r" (r), [g] "=&r" (g), [b] "=&r" (b), [tmp] "=&r" (temp)
        : [in] "r" (pixel_in)
    );

    return (uint16_t)result;
}

void __not_in_flash_func(set_brightness_fast_levels)(uint16_t* line, int count, int level) {
    uint32_t* p32 = (uint32_t*)line;
    int loop_count = count >> 1;

    if (level >= 4) {
        return;
    }

    switch (level) {
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

void __not_in_flash_func(reduce_brightness_50_asm_fast)(uint16_t* line, int count) {
    const uint32_t mask = 0xF7DEF7DE;
    int n = count >> 3;
    if (n == 0) goto handle_remainder;
    __asm volatile (
        ".p2align 2\n"
        "1:\n"
        "ldmia %[ptr]!, {r4-r7}\n"
        "and r4, %[mask]\n"
        "and r5, %[mask]\n"
        "and r6, %[mask]\n"
        "and r7, %[mask]\n"
        "lsr r4, #1\n"
        "lsr r5, #1\n"
        "lsr r6, #1\n"
        "lsr r7, #1\n"
        "sub %[ptr], #16\n"
        "stmia %[ptr]!, {r4-r7}\n"
        "subs %[n], #1\n"
        "bne 1b\n"
        : [ptr] "+&r" (line),
          [n]   "+&r" (n)
        : [mask] "r" (mask)
        : "r4", "r5", "r6", "r7", "memory", "cc"
    );

handle_remainder:
    int remaining_start = count & ~7;
    for (int i = remaining_start; i < count; ++i) {
        line[i] = (line[i] & 0xF7DE) >> 1;
    }
}

void __not_in_flash_func(get_pio_line)(uint16_t* line_buffer) {
    pio_sm_put_blocking(pio, sm_video, ((SAMPLES_PER_LINE+HBLANK) / 2 )- 1);
    pio_sm_set_enabled(pio, sm_video, true);
    for (uint i = 0; i < HBLANK; ++i) {
        (void)pio_sm_get_blocking(pio, sm_video);
    }

    uint16_t shres_pixel;
    uint16_t shres_pixel2;
    for (uint i = 0; i < SAMPLES_PER_LINE; ++i) {
        line_buffer[i] = convert_12_to_565_reordered_fast(pio_sm_get_blocking(pio, sm_video));
    }
    pio_sm_set_enabled(pio, sm_video, false);
}

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;
    uint32_t flags = save_and_disable_interrupts();
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    for (volatile int i = 0; i < 1000; ++i);
    bool button_state = !(sio_hw->gpio_hi_in & SIO_GPIO_HI_IN_QSPI_CSN_BITS);
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

int scanline_level = 3;
bool button_pressed_previously = false;
void __no_inline_not_in_flash_func(check_button)() {
    bool button_is_pressed_now = get_bootsel_button();
    if (button_pressed_previously && !button_is_pressed_now) {
        scanline_level = (scanline_level + 1) % 5;
        //delay(50); 
    }
    button_pressed_previously = button_is_pressed_now;
}

// =============================================================================
// --- PIO Setup ---
// =============================================================================
void setup_vsync_detect_sm(uint offset) {
    sm_vsync = pio_claim_unused_sm(pio, true);
    pio_sm_config c = vsync_detect_program_get_default_config(offset);
    sm_config_set_clkdiv(&c, 32.0f); //PIO SM für Vsync läuft mit ~10MHz
    pio_gpio_init(pio, csync);
    gpio_set_dir(csync, GPIO_IN);
    sm_config_set_jmp_pin(&c, csync);
    sm_config_set_in_pins(&c, csync);
    pio_sm_init(pio, sm_vsync, offset, &c);
    pio_sm_set_enabled(pio, sm_vsync, true);
}

void setup_video_capture_sm(uint offset) {
    sm_video = pio_claim_unused_sm(pio, true);
    pio_sm_config c = video_capture_program_get_default_config(offset);
    sm_config_set_in_pins(&c, 0);
    sm_config_set_in_shift(&c, true,true,32);
    pio_sm_set_consecutive_pindirs(pio, sm_video, 0, 32, false);
    sm_config_set_wrap(&c, offset + video_capture_wrap_target, offset + video_capture_wrap);
    sm_config_set_in_shift(&c, true, true, 1);
    pio_sm_init(pio, sm_video, offset, &c);
    pio_sm_set_enabled(pio, sm_video, false);
}


// =============================================================================
// --- Core 1 Entry ---
// =============================================================================
void core1_entry() {
    // Konfiguriere den PIO-Interrupt für VSYNC
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    pio_set_irq1_source_enabled(pio, pis_interrupt1, true);
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
    
            // Aktive Videozeilen einlesen und in die FIFO schreiben
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
    uint clkdiv = 3;
    uint rxdelay = 3;
    hw_write_masked(
        &qmi_hw->m[0].timing,
        ((clkdiv << QMI_M0_TIMING_CLKDIV_LSB) & QMI_M0_TIMING_CLKDIV_BITS) |
        ((rxdelay << QMI_M0_TIMING_RXDELAY_LSB) & QMI_M0_TIMING_RXDELAY_BITS),
        QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS
    );
    busy_wait_us(1000);
    set_sys_clock_khz(320000, true);

    // Initialisiere alle nicht-MIPI-GPIOs
    for (uint i = 0; i <= 47; i++) {
        gpio_init(i);
        gpio_set_dir(i, 0);
        gpio_pull_up(i);
    }

    // Daten- und Sync-Pins initialisieren
    for (uint8_t i = 0; i < 12; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_input_hysteresis_enabled(i, true);
    }

    gpio_init(csync);
    gpio_set_dir(csync, GPIO_IN);
    gpio_set_input_hysteresis_enabled(csync, true);

    gpio_init(pixelclock);
    gpio_set_dir(pixelclock, GPIO_IN);
    gpio_set_input_hysteresis_enabled(pixelclock, true);

    // PIO-Programme laden und State Machines einrichten
    uint offset_vsync = pio_add_program(pio, &vsync_detect_program);
    uint offset_video = pio_add_program(pio, &video_capture_program);
    setup_vsync_detect_sm(offset_vsync);
    setup_video_capture_sm(offset_video);
    pio_sm_put_blocking(pio, sm_video, ((SAMPLES_PER_LINE+HBLANK) / 2) - 1);

    size_t num_pixels = (size_t)ACTIVE_VIDEO * LINES_PER_FRAME;
    size_t buffer_size_bytes = num_pixels * sizeof(uint16_t);
    framebuffer = (uint16_t *)malloc(buffer_size_bytes);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    multicore_launch_core1(core1_entry);

    mipi_init();
    DMASetup(0);

    uint32_t lines_read_count = 0;
    bool frame_active = false;
    __attribute__((aligned(4))) uint16_t temp_scanline[VIDEO_LINE_LENGTH];
    bool scanlines = false;


    while (1) {

        //check_button();

        if (!frame_active) {
            // Warten auf den Beginn eines neuen Frames
            if(vsync_go){
                vsync_go = false;    
                mipiCsiFrameStart();
                frame_active = true;
                lines_read_count = 0;
            }
        } else {
            // Frame wird aktiv gelesen und gesendet
            if(video_go){
                dma_memcpy2(line2,line1, ACTIVE_VIDEO*2);
                video_go=false; 

                if(laced) {
                    if(isPAL){
                        if (is_odd_field) {
                            if(scanlines){
                                memcpy(temp_scanline,line2,ACTIVE_VIDEO*2);
                                set_brightness_fast_levels(temp_scanline, ACTIVE_VIDEO,scanline_level); 
                                mipiCsiSendLong(0x22, (uint8_t*)temp_scanline, ACTIVE_VIDEO*2);
                            }else {
                                mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                            }
                            mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                        } else {
                            if(scanlines)set_brightness_fast_levels(framebuffer + (ACTIVE_VIDEO * lines_read_count), ACTIVE_VIDEO,scanline_level); 
                            mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                            mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                        }
                    }else{
                        if (!is_odd_field) {
                            if(scanlines){
                                memcpy(temp_scanline,line2,ACTIVE_VIDEO*2);
                                set_brightness_fast_levels(temp_scanline, ACTIVE_VIDEO,scanline_level); 
                                mipiCsiSendLong(0x22, (uint8_t*)temp_scanline, ACTIVE_VIDEO*2);
                            }else {
                                mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                            }
                            mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                        } else {                            
                            if(scanlines)set_brightness_fast_levels(framebuffer + (ACTIVE_VIDEO * lines_read_count), ACTIVE_VIDEO,scanline_level); 
                            mipiCsiSendLong(0x22, (uint8_t*) framebuffer + (ACTIVE_VIDEO*2 * lines_read_count), ACTIVE_VIDEO*2);
                            mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                        }
                    }
                    dma_memcpy3(framebuffer + (ACTIVE_VIDEO * lines_read_count),line2, ACTIVE_VIDEO*2);
                } else {
                    mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                    if(scanlines)set_brightness_fast_levels(line2, ACTIVE_VIDEO,scanline_level); 
                    mipiCsiSendLong(0x22, (uint8_t*)line2, ACTIVE_VIDEO*2);
                }

                lines_read_count++;

                // Prüfen, ob der Frame vollständig übertragen wurde
                if (lines_read_count >= (isPAL ? LINES_PER_FRAME-1 : LINES_PER_FRAME_NTSC-1)) {
                    mipiCsiSendLong(0x22, (uint8_t*)blackline, ACTIVE_VIDEO*2);
                    mipiCsiSendLong(0x22, (uint8_t*)blackline, ACTIVE_VIDEO*2);
                    mipiCsiFrameEnd();
                    frame_active = false;
                }
            }
        }
    }
}
