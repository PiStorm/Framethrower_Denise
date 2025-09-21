#pragma GCC optimize("O3")

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/resets.h"
#include "hardware/xosc.h"
#include "hardware/pll.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/sio.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/structs/qmi.h"
#include "pico/sem.h"
#include <stdlib.h>
#include <string.h>
#include "pico/multicore.h"
#include "video_capture.pio.h"
#include "data_packet.h"
#include <time.h>
#include "pico/time.h"
#include "mipi.h"
#include "fifo.h"
#include "pico/platform.h"
#define VIDEO_LINE_LENGTH 1024
#define LINES_PER_FRAME 288
__attribute__((aligned(4))) fifo_t my_fifo;
__attribute__((aligned(4))) uint16_t videoline_in[VIDEO_LINE_LENGTH];
__attribute__((aligned(4))) uint16_t videoline_filter[VIDEO_LINE_LENGTH];

#include "sync_irq.pio.h"

// Globale Variablen
PIO pio = pio0;
uint sm_video, sm_vsync;

#define PIO_SM_INDEX 0 // Using State Machine 0

// GPIO pins for data (RGB444)
#define DATA_PIN_BASE 0  // GPIO0
#define NUM_DATA_PINS 12 // GPIO0 to GPIO11

// GPIO pin for pixel clock
#define PIXEL_CLOCK_PIN 22
#define SAMPLES_PER_LINE 800

//PIO pio;
//uint sm;

#define DMACH_MEMCPY 2

// --- Globale Variablen ---
volatile bool vsync_detected = false;
//volatile uint32_t vsync_frequency_scaled = 0; // Speichert Frequenz * 100 (z.B. 4992 für 49.92Hz)
//PIO pio_instance = pio1;
//uint sm_instance;
//volatile uint32_t last_pulse_type = 0; // 0=none, 1=short, 2=long



// Interrupt Service Routine für PIO IRQ 0
void pio_irq_handler() {
    // Prüfen, ob der Interrupt von unserem VSYNC-Programm (irq 0) kommt
    if (pio_interrupt_get(pio, 0)) {
        pio_interrupt_clear(pio, 0); // Wichtig: Interrupt-Flag im PIO löschen
        vsync_detected = true;
    }
}



/*
void on_vsync_irq() {
    // Statische Variable, um den letzten Zeitpunkt zu speichern
    static absolute_time_t previous_time = {0};

    // 1. Aktuellen Zeitpunkt erfassen
    absolute_time_t current_time = get_absolute_time();

    // 2. Zeitdifferenz berechnen (nur wenn wir einen vorherigen Zeitpunkt haben)
    if (!is_nil_time(previous_time)) {
        int64_t time_diff_us = absolute_time_diff_us(previous_time, current_time);
        if (time_diff_us > 0) {
            // 3. Frequenz mit Integer-Arithmetik berechnen und skalieren
            // Formel: (1 / (diff_us / 1,000,000)) * 100 = 100,000,000 / diff_us
            vsync_frequency_scaled = 100000000 / time_diff_us;
        }
    }
    
    // 4. Aktuellen Zeitpunkt für die nächste Messung speichern
    previous_time = current_time;

    // 5. Flag setzen und IRQ löschen
    vsync_detected = true;
    last_pulse_type = pio_sm_get(pio_instance, sm_instance);
    hw_clear_bits(&pio_instance->irq, 1u << 0);
}
*/


// ----------------------------------------------------------------------------
// DVI constants

#define hsync 20
#define vsync 23
#define pixelclock 22
volatile uint v_scanline = 2;
volatile uint scanout;
volatile uint scanout_end;

#define MODE_H_SYNC_POLARITY 0
#define MODE_H_FRONT_PORCH 12
#define MODE_H_SYNC_WIDTH 64
#define MODE_H_BACK_PORCH 68 //68
#define MODE_H_ACTIVE_PIXELS 720

#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH 5
#define MODE_V_SYNC_WIDTH 5
#define MODE_V_BACK_PORCH 39
#define MODE_V_ACTIVE_LINES 576
#define FRAMEBUFFER_WIDTH 720
#define FRAMEBUFFER_HEIGHT 576 / 2

#define MODE_H_TOTAL_PIXELS (                \
    MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + \
    MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS)
#define MODE_V_TOTAL_LINES (                 \
    MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + \
    MODE_V_BACK_PORCH + MODE_V_ACTIVE_LINES)


//__attribute__((aligned(4))) uint16_t *framebuffer;
__attribute__((aligned(4))) uint16_t line_buffer_rgb444[1024];




// ----------------------------------------------------------------------------
// Main program

static __force_inline uint16_t colour_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint16_t)r & 0xf8) >> 3 | ((uint16_t)g & 0xfc) << 3 | ((uint16_t)b & 0xf8) << 8;
}

static __force_inline uint16_t colour_rgb444(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint16_t)r & 0xF0) << 0 | ((uint16_t)g & 0xF0) << 4 | ((uint16_t)b & 0xF0) << 8;
}

static __force_inline uint8_t colour_rgb332(uint8_t r, uint8_t g, uint8_t b)
{
    return (r & 0xc0) >> 6 | (g & 0xe0) >> 3 | (b & 0xe0) >> 0;
}


uint16_t __not_in_flash_func(convert_12bit_to_rgb565_inline)(uint16_t p) {
    return (((p >> 8) & 0xF) << 12) | (((p >> 8) & 0xF) << 7) | // Rot (RRRR0000000RRRR0) - vereinfacht
           (((p >> 4) & 0xF) << 7) | (((p >> 4) & 0xF) << 1) |  // Grün
           ((p & 0xF) << 1) | ((p & 0xF) >> 3);                 // Blau
}

static inline uint16_t __not_in_flash_func(convert_12_to_565_fast)(uint32_t pixel_in) {
    uint32_t result;

  __asm volatile (
        // --- B-Komponente (Ziel: Bits 4-0) ---
        // 1. Extrahiere b4, skaliere zu b5 und schreibe es als Startwert ins Ergebnis.
        "ubfx   r2, %[in], #8, #4        \n\t" // r2 = bbbb
        "lsr    r3, r2, #3              \n\t" // r3 = b3 (MSB)
        "orr    %[out], r3, r2, lsl #1   \n\t" // result = bbbbb

        // --- G-Komponente (Ziel: Bits 10-5) ---
        // 2. Extrahiere g4, skaliere zu g6 und füge es mit BFI in die Mitte ein.
        "ubfx   r2, %[in], #4, #4        \n\t" // r2 = gggg
        "lsr    r3, r2, #2              \n\t" // r3 = g3g2 (MSBs)
        "orr    r2, r3, r2, lsl #2      \n\t" // r2 = gggggg
        "bfi    %[out], r2, #5, #6       \n\t" // result |= (gggggg << 5)

        // --- R-Komponente (Ziel: Bits 15-11) ---
        // 3. Extrahiere r4, skaliere zu r5 und füge es mit BFI oben ein.
        "ubfx   r2, %[in], #0, #4        \n\t" // r2 = rrrr
        "lsr    r3, r2, #3              \n\t" // r3 = r3 (MSB)
        "orr    r2, r3, r2, lsl #1      \n\t" // r2 = rrrrr
        "bfi    %[out], r2, #11, #5      \n\t" // result |= (rrrrr << 11)

        : [out] "=&r" (result)  // Ausgaberegister
        : [in] "r" (pixel_in)   // Eingaberegister
        : "r2", "r3"            // Verwendete Hilfsregister
    );


    return (uint16_t)result;
}

uint16_t videoline_out[VIDEO_LINE_LENGTH];

void __not_in_flash_func(get_pio_line)(void)
{
    uint32_t pixel12;
    pio_sm_clear_fifos(pio, sm_video);
    pio_sm_restart(pio, sm_video);
    pio_sm_put_blocking(pio, sm_video, SAMPLES_PER_LINE/2 - 1);
    pio_sm_set_enabled(pio, sm_video, true);

    for (int i = 0; i < SAMPLES_PER_LINE; ++i)
    {

        // pixel12 = pio_sm_get_blocking(pio, sm);
        while ((pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm_video))) != 0)
        {
        }
        pixel12 = pio->rxf[sm_video];
        line_buffer_rgb444[i] =convert_12_to_565_fast(pixel12);
    }
    pio_sm_set_enabled(pio, sm_video, false);
}

/*
void __not_in_flash_func(dma_memcpy)(void *dst, const void *src, size_t len)
{

    dma_channel_config c = dma_channel_get_default_config(2);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_FORCE);
    dma_channel_configure(
        DMACH_MEMCPY,
        &c,
        dst,     // Zieladresse
        src,     // Quelladresse
        len / 4, // Anzahl der 32-Bit-Wörter
        true     // Starte den Transfer
    );
    dma_channel_wait_for_finish_blocking(DMACH_MEMCPY);
}
*/

volatile bool pio_interrupt_triggered = false;

void __not_in_flash_func(pio_interrupt_handler)() {
    if (pio_interrupt_get(pio, 0)) {
        //gpio_put(28,1); 
        pio_interrupt_clear(pio, 0);
        pio_interrupt_triggered = true;
        //gpio_put(28,0); 
    }
}


bool waitForValidPulse(uint pin, uint32_t min_duration_us) {

    uint8_t count = 0;
    for (uint8_t y = 0; y < 5; y++){
    // 1. Warten, bis der Pin auf LOW geht
    while (gpio_get(pin)) {
        tight_loop_contents();
    }

    // Pin ist LOW, starte die Zeitmessung
    uint32_t start_time_us = time_us_32();

    // 2. Warten, bis der Pin wieder auf HIGH geht
    while (!gpio_get(pin)) {
        tight_loop_contents();
    }
    
    // Pin ist wieder HIGH, berechne die Dauer
    uint32_t pulse_duration_us = time_us_32() - start_time_us;

    // 3. Dauer prüfen und entsprechenden booleschen Wert zurückgeben
    if (pulse_duration_us >= min_duration_us) {
        //return true; // Gültiger Impuls
        count++;
    } else {
        //return false; // Zu kurz, wird als Störung ignoriert
    }
    }
    if (count >= 4) return true;
}



uint8_t vsync_start = 0;

void core1_entry()
{

        // Konfiguriere den PIO-Interrupt für VSYNC
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);


    uint16_t x = 0, y = 0, z = 0;
    while (1)
    {
//       if (waitForValidPulse(vsync,20)){
 if (vsync_detected) {
            vsync_detected = false; // Flag zurücksetzen

        //busy_wait_us(63);

            vsync_start = 1;
            for (y = 0; y < 21; y++)
            {
                get_pio_line();
            }
            scanout = v_scanline;
            for (y = 0; y < 288; y++)
            {
                get_pio_line();
                //while (!fifo_write_buffer(&my_fifo, line_buffer_rgb444 + 69, VIDEO_LINE_LENGTH)) {}
                while (!fifo_write_buffer_dma(&my_fifo, line_buffer_rgb444+69, VIDEO_LINE_LENGTH)) {}
                //dma_memcpy(framebuffer + (MODE_H_ACTIVE_PIXELS * y), line_buffer_rgb444 + 69, MODE_H_ACTIVE_PIXELS * 2);
            }
            scanout_end = v_scanline;
            /*
                while (!(gpioc_lo_in_get() & (1 << vsync)))
            {
            }
            */
            vsync_start = 0;
            //while(waitForValidPulse(vsync,20)){}
        }
    }
}


void gpio_callback(uint gpio, uint32_t events) {
    if(gpio==vsync)  v_scanline = MODE_V_TOTAL_LINES-( MODE_V_BACK_PORCH);
}

void __not_in_flash_func(sendFullFrameLineDoubled)(uint16_t *buffer) {
    // Die Länge einer einzelnen Zeile in Bytes bleibt unverändert
    const int bytes_per_line = 720 * sizeof(uint16_t); // 1440 Bytes

    // MIPI CSI-2 Paket-Typ für Ihr Pixelformat (z.B. RGB565)
    const int mipi_packet_type = 0x22;

    // Die Schleife läuft über alle 576 ZIEL-Zeilen
    for (int y = 0; y < 576; y++) {
        // *** HIER IST DIE ÄNDERUNG ***
        // Berechne die zu lesende Zeile aus dem Quell-Framebuffer.
        // Durch Integer-Division wird für y=0 und y=1 die source_line 0,
        // für y=2 und y=3 die source_line 1 usw. verwendet.
        int source_line = y / 2;

        // Berechne den Zeiger auf den Anfang der Quell-Zeile.
        uint8_t *line_data = (uint8_t *)(buffer + source_line * 720);

        // Sende die (ggf. wiederholte) Zeile.
        mipiCsiSendLong(mipi_packet_type, line_data, bytes_per_line);
    }
}


void __not_in_flash_func(reduce_brightness_50_rgb565)(uint16_t* line, int count) {
    // Magische Konstante, die zwei 16-Bit-Masken (0xF7DE) kombiniert.
    // 0xF7DE = 0b1111011111011110
    // Diese Maske löscht das LSB (Least Significant Bit) jedes Farbkanals.
    const uint32_t mask = 0xF7DEF7DE;

    uint32_t* p32 = (uint32_t*)line;
    int i;
    
    // Verarbeite 2 Pixel (32 Bit) auf einmal.
    // Bei 1024 Pixeln läuft die Schleife 512 Mal.
    for (i = 0; i < count / 2; ++i) {
        uint32_t pixels = p32[i];
        
        // 1. Maskieren: Lösche die LSBs der R, G und B Kanäle beider Pixel.
        pixels = pixels & mask;
        
        // 2. Shiften: Teile alle Kanäle beider Pixel durch 2.
        pixels = pixels >> 1;
        
        p32[i] = pixels;
    }

    // Falls die anzahl der Pixel ungerade ist, wird das letzte Pixel hier behandelt
    if (count % 2 != 0) {
        uint16_t pixel = line[count - 1];
        pixel = (pixel & 0xF7DE) >> 1;
        line[count - 1] = pixel;
    }
}

void __not_in_flash_func(set_brightness_rgb565)(uint16_t* line, int count, int level) {
   if (level >= 3) return;
    if (level <= 0) {
        memset(line, 0, count * sizeof(uint16_t));
        return;
    }

    uint32_t* p32 = (uint32_t*)line;
    // Wir verarbeiten 4 Pixel (zwei uint32_t) pro Durchlauf
    const int loop_count = count / 4;
    int i;

    for (i = 0; i < loop_count; ++i) {
        // Lade zwei 32-Bit-Werte (insgesamt 4 Pixel)
        uint32_t pixels1 = p32[i * 2];
        uint32_t pixels2 = p32[i * 2 + 1];

        // Extrahiere die 4 einzelnen 16-Bit-Pixel
        uint16_t pA = pixels1 & 0xFFFF;
        uint16_t pB = pixels1 >> 16;
        uint16_t pC = pixels2 & 0xFFFF;
        uint16_t pD = pixels2 >> 16;

        if (level == 1) { // Stufe 1: 25% -> Wert >> 2
            uint32_t r, g, b;
            // Pixel A
            r = (pA & 0xF800) >> 11; g = (pA & 0x07E0) >> 5; b = (pA & 0x001F);
            pA = ((r >> 2) << 11) | ((g >> 2) << 5) | (b >> 2);
            // Pixel B
            r = (pB & 0xF800) >> 11; g = (pB & 0x07E0) >> 5; b = (pB & 0x001F);
            pB = ((r >> 2) << 11) | ((g >> 2) << 5) | (b >> 2);
            // Pixel C
            r = (pC & 0xF800) >> 11; g = (pC & 0x07E0) >> 5; b = (pC & 0x001F);
            pC = ((r >> 2) << 11) | ((g >> 2) << 5) | (b >> 2);
            // Pixel D
            r = (pD & 0xF800) >> 11; g = (pD & 0x07E0) >> 5; b = (pD & 0x001F);
            pD = ((r >> 2) << 11) | ((g >> 2) << 5) | (b >> 2);
        }
        else if (level == 2) { // Stufe 2: 75% -> (Wert * 3) >> 2
            uint32_t r, g, b;
            // Pixel A
            r = (pA & 0xF800) >> 11; g = (pA & 0x07E0) >> 5; b = (pA & 0x001F);
            pA = (((r * 3) >> 2) << 11) | (((g * 3) >> 2) << 5) | ((b * 3) >> 2);
            // Pixel B
            r = (pB & 0xF800) >> 11; g = (pB & 0x07E0) >> 5; b = (pB & 0x001F);
            pB = (((r * 3) >> 2) << 11) | (((g * 3) >> 2) << 5) | ((b * 3) >> 2);
            // Pixel C
            r = (pC & 0xF800) >> 11; g = (pC & 0x07E0) >> 5; b = (pC & 0x001F);
            pC = (((r * 3) >> 2) << 11) | (((g * 3) >> 2) << 5) | ((b * 3) >> 2);
            // Pixel D
            r = (pD & 0xF800) >> 11; g = (pD & 0x07E0) >> 5; b = (pD & 0x001F);
            pD = (((r * 3) >> 2) << 11) | (((g * 3) >> 2) << 5) | ((b * 3) >> 2);
        }
        
        // Schreibe die 4 modifizierten Pixel zurück in den Speicher
        p32[i * 2] = (pB << 16) | pA;
        p32[i * 2 + 1] = (pD << 16) | pC;
    }
    
    // Optional: Restliche Pixel behandeln, falls die Anzahl nicht durch 4 teilbar ist.
    // Für Ihre 1024 Pixel ist das nicht notwendig.
}


void __not_in_flash_func(reduce_brightness_50_unrolled)(uint16_t* line, int count) {
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

void __not_in_flash_func(average_adjacent_pixels_FAST)(uint16_t* dest, const uint16_t* src, int src_count) {
    const uint32_t* p_src = (const uint32_t*)src;
    uint32_t* p_dest = (uint32_t*)dest; // Wir schreiben auch 2 Pixel (32 Bit) auf einmal

    // Masken für die bitweise Mittelwertbildung
    const uint32_t dim_mask   = 0xF7DEF7DE;
    const uint32_t round_mask = 0x08210821;
    
    // Wir lesen 4 Pixel (2x uint32_t), also ist die Schleife / 4
    const int loop_count = src_count / 4;
    for (int i = 0; i < loop_count; ++i) {
        // Lese 4 Quellpixel auf einmal
        uint32_t src_pair1 = p_src[i * 2];
        uint32_t src_pair2 = p_src[i * 2 + 1];

        // Extrahiere die einzelnen Pixel
        uint16_t pA = src_pair1 & 0xFFFF;
        uint16_t pB = src_pair1 >> 16;
        uint16_t pC = src_pair2 & 0xFFFF;
        uint16_t pD = src_pair2 >> 16;

        // Berechne den Mittelwert für das erste Paar (pA, pB)
        uint16_t avg1 = ((pA & 0xF7DE) >> 1) + ((pB & 0xF7DE) >> 1) + (pA & pB & 0x0821);

        // Berechne den Mittelwert für das zweite Paar (pC, pD)
        uint16_t avg2 = ((pC & 0xF7DE) >> 1) + ((pD & 0xF7DE) >> 1) + (pC & pD & 0x0821);
        
        // Schreibe 2 neue Pixel (als ein uint32_t) in den Zielpuffer
        p_dest[i] = (avg2 << 16) | avg1;
    }

    // Ggf. restliche Pixelpaare verarbeiten (für 1024 nicht nötig)
}

// Die bitweise Mittelwert-Funktion als Helfer (inline für maximale Performance)
static inline uint16_t avg_pixels(uint16_t pA, uint16_t pB) {
    return ((pA & 0xF7DE) >> 1) + ((pB & 0xF7DE) >> 1) + (pA & pB & 0x0821);
}

/**
 * @brief Erzeugt einen Unschärfe-Effekt (Blur/Blend) über benachbarte Pixel.
 * Das Ergebnis hat die gleiche Größe wie die Quelle.
 *
 * @param dest   Zeiger auf den Zielpuffer (muss mind. so groß sein wie src).
 * @param src    Zeiger auf den Quellpuffer mit den Originalpixeln.
 * @param count  Anzahl der Pixel in der Zeile (z.B. 1024).
 */
void __not_in_flash_func(blend_blur_adjacent_FAST)(uint16_t* dest, const uint16_t* src, int count) {
    if (count < 2) {
        if (count == 1) dest[0] = src[0];
        return;
    }

    // Wir verarbeiten 2 Pixel pro Schleifendurchlauf.
    // Dafür lesen wir immer ein Fenster von 3 Quellpixeln.
    int i;
    for (i = 0; i < count - 2; i += 2) {
        uint16_t pA = src[i];
        uint16_t pB = src[i + 1];
        uint16_t pC = src[i + 2];

        // Berechne den ersten neuen Pixel aus pA und pB
        dest[i] = avg_pixels(pA, pB);

        // Berechne den zweiten neuen Pixel aus pB und pC
        dest[i + 1] = avg_pixels(pB, pC);
    }
    
    // Die letzten 1-2 Pixel behandeln, die nicht in die 2er-Schleife passten
    for (; i < count - 1; ++i) {
        dest[i] = avg_pixels(src[i], src[i+1]);
    }
    
    // Der allerletzte Pixel kann nicht mit seinem rechten Nachbarn gemischt werden.
    // Wir kopieren ihn einfach, um die Bildbreite zu erhalten.
    dest[count - 1] = src[count - 1];
}


void setup_vsync_detect_sm(uint offset) {
    // State Machine für VSYNC beanspruchen
    sm_vsync = pio_claim_unused_sm(pio, true);
    pio_sm_config c = vsync_detect_program_get_default_config(offset);
    sm_config_set_clkdiv(&c, 30.0f);
    pio_gpio_init(pio, vsync);
    gpio_set_dir(vsync, GPIO_IN);
    sm_config_set_jmp_pin(&c, vsync);
    sm_config_set_in_pins(&c, vsync); // Wichtig für 'wait pin'
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

int main(void)
{


    //External LDO
    //vreg_set_voltage(VREG_VOLTAGE_0_85);

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


    for (uint i = 0; i <= 47; i++) {
    gpio_init(i);
    gpio_set_dir(i, 0);
    gpio_disable_pulls(i);
    }


    size_t num_pixels = (size_t)FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT;
    size_t buffer_size_bytes = num_pixels * sizeof(uint16_t);
    //framebuffer = (uint16_t *)malloc(buffer_size_bytes);


    for (uint8_t i = 0; i < 12; i++)
    {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_input_hysteresis_enabled(i, true);
    }


    gpio_init(hsync);
    gpio_set_dir(hsync, GPIO_IN);

    gpio_init(vsync);
    gpio_set_dir(vsync, GPIO_IN);
    //gpio_set_irq_enabled_with_callback(vsync,GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(pixelclock);
    gpio_set_dir(pixelclock, GPIO_IN);

    gpio_set_input_hysteresis_enabled(hsync, true);
    gpio_set_input_hysteresis_enabled(vsync, true);
    gpio_set_input_hysteresis_enabled(pixelclock, true);




    uint offset_vsync = pio_add_program(pio, &vsync_detect_program);
    uint offset_video = pio_add_program(pio, &video_capture_program);
    setup_vsync_detect_sm(offset_vsync);
    setup_video_capture_sm(offset_video);

   
    //pio = pio0;
    //uint offset = pio_add_program(pio, &video_capture_program);
    //sm = pio_claim_unused_sm(pio, true);
    //video_capture_program_init(pio, sm, offset, 0);

    // pio->input_sync_bypass = (1u<<22); //bypass clock sync

    //for (uint8_t i = 0; i < 12; i++)
    //{
    //    pio->input_sync_bypass = (1u<<i); //bypass sync
    //}
/*
    pio_set_irq0_source_enabled(pio,pis_interrupt0, true);
    pio_interrupt_clear(pio, sm);

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_interrupt_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
*/


/*
    // --- PIO Initialisierung ---
    uint offset2 = pio_add_program(pio_instance, &sync_irq_program);
    sm_instance = pio_claim_unused_sm(pio_instance, true);
    
    const uint CSYNC_PIN = vsync;
    pio_sm_config c = sync_irq_program_get_default_config(offset2);
    
    pio_gpio_init(pio_instance, CSYNC_PIN);
    sm_config_set_in_pins(&c, CSYNC_PIN);
    sm_config_set_jmp_pin(&c, CSYNC_PIN);
    
    //uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    //float div = (float)sys_clk_hz / 10000000.0f;
    //sm_config_set_clkdiv(&c, 30.0f);

    sm_config_set_clkdiv(&c, 15.0f);

    pio_sm_init(pio_instance, sm_instance, offset2, &c);
    
    // --- Interrupt-Konfiguration ---
    pio_set_irq0_source_enabled(pio_instance, pis_interrupt0 + sm_instance, true);
    irq_set_exclusive_handler(PIO1_IRQ_0, on_vsync_irq);
    irq_set_enabled(PIO1_IRQ_0, true);

    pio_sm_set_enabled(pio_instance, sm_instance, true);
*/


    fifo_init(&my_fifo);

    multicore_launch_core1(core1_entry);

    mipi_init();
    //DMASetup((uint8_t *)framebuffer);
     DMASetup(0);

    //__attribute__((aligned(4))) uint16_t videoline_in[VIDEO_LINE_LENGTH];
    //__attribute__((aligned(4))) uint16_t videoline_filter[VIDEO_LINE_LENGTH];

       // Zähler für die gelesenen Zeilen innerhalb eines Frames
    uint32_t lines_read_count = 0;
    
    // Statusvariable, die anzeigt, ob eine Frame-Übertragung aktiv ist
    bool frame_active = false;

 absolute_time_t last_vsync_time = get_absolute_time();

   while (1) {
        if (!frame_active) {
            // =========================================================
            //  ZUSTAND: IDLE - Warten auf den Beginn eines neuen Frames
            // =========================================================
            
            // Prüfen, ob der schreibende Kern begonnen hat, Daten zu senden
            if (!fifo_is_empty(&my_fifo)) {
                // Die ersten Daten sind da! Starte einen neuen Frame.
                //printf("Core 1: New frame detected. Calling mipi_start().\n");
                mipiCsiFrameStart();
                // Zustand wechseln und Zähler zurücksetzen
                frame_active = true;
                lines_read_count = 0;
            }
        } else {
            // ============================================================
            //  ZUSTAND: ACTIVE - Frame wird aktiv gelesen und gesendet
            // ============================================================
            
            // Versuche, eine komplette Videzeile zu lesen
            //if (fifo_read_buffer(&my_fifo, videoline_in, VIDEO_LINE_LENGTH)) {
            if (fifo_read_buffer_dma(&my_fifo, videoline_in, VIDEO_LINE_LENGTH)) {
                
                // Zeile erfolgreich gelesen, Zähler erhöhen
                lines_read_count++;
                
                // Sende die Zeile ZWEIMAL (Line Doubling)


                mipiCsiSendLong(0x22, (uint8_t*)videoline_in, 1440);
                //reduce_brightness_50_unrolled(videoline_filter,720);
                //reduce_brightness_50_unrolled(videoline_in,720);
                mipiCsiSendLong(0x22, (uint8_t*)videoline_in, 1440);

                // Prüfen, ob der Frame vollständig übertragen wurde
                if (lines_read_count >= LINES_PER_FRAME) {
                    // Die letzte (288.) Zeile wurde gelesen und gesendet.
                    //printf("Core 1: Frame finished (%u lines read). Calling mipi_end().\n", lines_read_count);
                    mipiCsiFrameEnd();
                    
                    // Zurück in den IDLE-Zustand für den nächsten Frame
                    frame_active = false; 
                }
            }
        }
        // Wenn keine Daten/kein voller Buffer im FIFO ist,
        // dreht die Schleife einfach weiter und versucht es erneut.
    }


    /*
    while (1)
    {

        //while(!vsync_start){}
        //if (waitForValidPulse(vsync,20)){
         mipiCsiFrameStart();
         for (int y = 0; y < 288; y++) {
            if (fifo_read_buffer(&my_fifo, videoline_in, VIDEO_LINE_LENGTH)) {
                
                // 2. Erfolg! Die Zeile wurde gelesen. Jetzt wird sie ZWEIMAL gesendet.

                // Sende die Zeile zum ersten Mal
                mipiCsiSendLong(
                    0x22, // Der Paket-Typ
                    (uint8_t*)videoline_in,     // Der Cast von uint16_t* zu uint8_t*
                    1440            // Die Länge in Bytes (2048)
                );

                // Sende dieselbe Zeile zum zweiten Mal (Line Doubling)
                mipiCsiSendLong(
                    0x22,
                    (uint8_t*)videoline_in,
                    1440
                );

            }
         }  
         mipiCsiFrameEnd();
        //}
        //while(!vsync_start){}        
    }
   */     
}
