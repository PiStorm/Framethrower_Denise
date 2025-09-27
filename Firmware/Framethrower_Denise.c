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
#define VIDEO_LINE_LENGTH 1024
#define LINES_PER_FRAME 288
#define LINES_PER_FRAME_NTSC 240
#define SAMPLES_PER_LINE 800
#define VBLANK_LINES 21

//const uint32_t LINES_PER_FRAME = 288;
//const uint32_t LINES_PER_FRAME_NTSC = 240;

// GPIO Pin-Definitionen
#define csync 23
#define pixelclock 22

// Globale Puffer und FIFO
__attribute__((aligned(4))) fifo_t my_fifo;
__attribute__((aligned(4))) uint16_t videoline_in[VIDEO_LINE_LENGTH];
__attribute__((aligned(4))) uint16_t line_buffer_rgb444[VIDEO_LINE_LENGTH];

// PIO Globals
PIO pio = pio0;
uint sm_video, sm_vsync;

// Interrupt Global
volatile bool vsync_detected = false;
volatile uint32_t lines;
volatile uint32_t last_total_lines;
volatile bool laced = false;
volatile bool isPAL = false;
volatile uint16_t lines_to_capture = 0;

// =============================================================================
// --- Interrupt Service Routine ---
// =============================================================================

// Wird bei VSYNC vom PIO aufgerufen
void __not_in_flash_func(pio_irq_handler)() {
 
    if (pio_interrupt_get(pio, 0)) {
        vsync_detected = true;
        laced = last_total_lines == lines  ? false : true;
        isPAL = last_total_lines <= 300 ? false : true;
        last_total_lines = lines;
        lines_to_capture = last_total_lines - 23;
        lines = 0;     
        pio_interrupt_clear(pio, 0); 
    }

    if (pio_interrupt_get(pio, 1)) {
        lines++;   
        pio_interrupt_clear(pio, 1); 
    }

}


// =============================================================================
// --- Hilfsfunktionen ---
// =============================================================================

//TODO -> RGB444 zu RGB565 kann eventuell der HSTX Expander-Block machen
static inline uint16_t __not_in_flash_func(convert_12_to_565_fast)(uint32_t pixel_in) {
    uint32_t result;
    __asm volatile (
        // --- B-Komponente (Ziel: Bits 4-0) ---
        "ubfx   r2, %[in], #8, #4        \n\t" // r2 = bbbb
        "lsr    r3, r2, #3              \n\t" // r3 = b3 (MSB)
        "orr    %[out], r3, r2, lsl #1  \n\t" // result = bbbbb

        // --- G-Komponente (Ziel: Bits 10-5) ---
        "ubfx   r2, %[in], #4, #4        \n\t" // r2 = gggg
        "lsr    r3, r2, #2              \n\t" // r3 = g3g2 (MSBs)
        "orr    r2, r3, r2, lsl #2      \n\t" // r2 = gggggg
        "bfi    %[out], r2, #5, #6      \n\t" // result |= (gggggg << 5)

        // --- R-Komponente (Ziel: Bits 15-11) ---
        "ubfx   r2, %[in], #0, #4        \n\t" // r2 = rrrr
        "lsr    r3, r2, #3              \n\t" // r3 = r3 (MSB)
        "orr    r2, r3, r2, lsl #1      \n\t" // r2 = rrrrr
        "bfi    %[out], r2, #11, #5     \n\t" // result |= (rrrrr << 11)

        : [out] "=&r" (result)
        : [in] "r" (pixel_in)
        : "r2", "r3"
    );
    return (uint16_t)result;
}

void __not_in_flash_func(reduce_brightness_50)(uint16_t* line, int count) {
    // Die magische Maske, dupliziert für 32-Bit-Verarbeitung.
    // 0xF7DE = 0b1111011111011110
    const uint32_t mask = 0xF7DEF7DE;

    uint32_t* p32 = (uint32_t*)line;
    const int loop_count = count / 4; // Wir verarbeiten 4 Pixel pro Durchlauf
    int i;

    for (i = 0; i < loop_count; ++i) {
        // Lade 4 Pixel (zwei 32-Bit-Werte)
        uint32_t pixels1 = p32[i * 2];
        uint32_t pixels2 = p32[i * 2 + 1];

        // Führe die Operation auf beiden Wertepaaren aus
        pixels1 = (pixels1 & mask) >> 1;
        pixels2 = (pixels2 & mask) >> 1;

        // Schreibe die 4 modifizierten Pixel zurück
        p32[i * 2] = pixels1;
        p32[i * 2 + 1] = pixels2;
    }

    // Ggf. die restlichen Pixel behandeln (für 1024 nicht nötig)
    int remaining_start = loop_count * 4;
    for(i = remaining_start; i < count; ++i) {
        line[i] = (line[i] & 0xF7DE) >> 1;
    }
}

void __not_in_flash_func(get_pio_line)(void) {
    uint32_t pixel12;
    pio_sm_clear_fifos(pio, sm_video);
    pio_sm_restart(pio, sm_video);
    pio_sm_put_blocking(pio, sm_video, SAMPLES_PER_LINE / 2 - 1);
    pio_sm_set_enabled(pio, sm_video, true);

    for (int i = 0; i < SAMPLES_PER_LINE; ++i) {
        while ((pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm_video))) != 0) {
        }
        pixel12 = pio->rxf[sm_video];
        line_buffer_rgb444[i] = convert_12_to_565_fast(pixel12);
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
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    restore_interrupts(flags);
    return button_state;
}


// =============================================================================
// --- PIO Setup ---
// =============================================================================
void setup_vsync_detect_sm(uint offset) {
    sm_vsync = pio_claim_unused_sm(pio, true);
    pio_sm_config c = vsync_detect_program_get_default_config(offset);
    sm_config_set_clkdiv(&c, 34.0f); //PIO SM für Vsync läuft mit ~10MHz
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
    //uint16_t lines_to_capture = 0;

    while (1) {
        if (vsync_detected) {
            vsync_detected = false; // Flag zurücksetzen

            // Vertical Blanking abwarten
            for (y = 0; y < VBLANK_LINES; y++) {
                get_pio_line();
            }

            // Aktive Videozeilen einlesen und in die FIFO schreiben
            for (y = 0; y < (isPAL? LINES_PER_FRAME-1 : LINES_PER_FRAME_NTSC-1); y++) {
                get_pio_line();
                // H-Blanking überspringen (Offset 69) und Daten in die FIFO schreiben
                while (!fifo_write_buffer_dma(&my_fifo, line_buffer_rgb444 + 69, VIDEO_LINE_LENGTH)) {}
            }
        }
    }
}


// =============================================================================
// --- Main Funktion (Core 0) ---
// =============================================================================
int main(void) {
    uint clkdiv = 3;
    uint rxdelay = 3;
    hw_write_masked(
        &qmi_hw->m[0].timing,
        ((clkdiv << QMI_M0_TIMING_CLKDIV_LSB) & QMI_M0_TIMING_CLKDIV_BITS) |
        ((rxdelay << QMI_M0_TIMING_RXDELAY_LSB) & QMI_M0_TIMING_RXDELAY_BITS),
        QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS
    );
    busy_wait_us(1000);
    set_sys_clock_khz(340000, true);

    // Initialisiere alle nicht-MIPI-GPIOs
    for (uint i = 0; i <= 47; i++) {
        gpio_init(i);
        gpio_set_dir(i, 0);
        gpio_disable_pulls(i);
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

    fifo_init(&my_fifo);

    multicore_launch_core1(core1_entry);

    mipi_init();
    DMASetup(0);

    uint32_t lines_read_count = 0;
    bool frame_active = false;
    bool scanline = false;

    while (1) {
        if (!frame_active) {
            // Warten auf den Beginn eines neuen Frames
            if (!fifo_is_empty(&my_fifo)) {
                mipiCsiFrameStart();
                frame_active = true;
                lines_read_count = 0;
            }
        } else {
            // Frame wird aktiv gelesen und gesendet
            if (fifo_read_buffer_dma(&my_fifo, videoline_in, VIDEO_LINE_LENGTH)) {
                lines_read_count++;

                // Sende die Zeile ZWEIMAL (Line Doubling)
                mipiCsiSendLong(0x22, (uint8_t*)videoline_in, 1440);
                if(scanline){reduce_brightness_50(videoline_in,720);}
                mipiCsiSendLong(0x22, (uint8_t*)videoline_in, 1440);
/*
                if(!isPAL) {
                    for (uint8_t i = 0; i < LINES_PER_FRAME - LINES_PER_FRAME_NTSC; i++) {
                        mipiCsiSendLong(0x22, (uint8_t*)videoline_in, 1440);
                    }
                }

*/

                // Prüfen, ob der Frame vollständig übertragen wurde
                //if (lines_read_count >= (isPAL ? LINES_PER_FRAME : LINES_PER_FRAME_NTSC)) {
                if (lines_read_count >= (isPAL ? LINES_PER_FRAME-1 : LINES_PER_FRAME_NTSC-1)) {
                    mipiCsiFrameEnd();
                    frame_active = false;
                }
            }
        }
    }
}