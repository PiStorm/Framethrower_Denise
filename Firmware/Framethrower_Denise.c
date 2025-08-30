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

#define PIO_SM_INDEX 0 // Using State Machine 0

// GPIO pins for data (RGB444)
#define DATA_PIN_BASE 0  // GPIO0
#define NUM_DATA_PINS 12 // GPIO0 to GPIO11

// GPIO pin for pixel clock
#define PIXEL_CLOCK_PIN 22
#define SAMPLES_PER_LINE 800

PIO pio;
uint sm;

// ----------------------------------------------------------------------------
// DVI constants

#define hsync 20
#define vsync 23
#define pixelclock 22

#define TMDS_CTRL_00 0x354u
#define TMDS_CTRL_01 0x0abu
#define TMDS_CTRL_10 0x154u
#define TMDS_CTRL_11 0x2abu

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1_WITH_PREAMBLE (TMDS_CTRL_11 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H0_WITH_DATA_ISLAND_PREAMBLE (TMDS_CTRL_00 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_01 << 20))
#define SYNC_V0_H1_WITH_DATA_ISLAND_PREAMBLE (TMDS_CTRL_01 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_01 << 20))
#define SYNC_V1_H0_WITH_DATA_ISLAND_PREAMBLE (TMDS_CTRL_10 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_01 << 20))
#define SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE (TMDS_CTRL_11 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_01 << 20))
#define SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE (TMDS_CTRL_11 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_01 << 20))
#define VIDEO_LEADING_GUARD_BAND (0x2ccu | (0x133u << 10) | (0x2ccu << 20))


/*
#define MODE_H_SYNC_POLARITY 0
#define MODE_H_FRONT_PORCH 12
#define MODE_H_SYNC_WIDTH 64
#define MODE_H_BACK_PORCH 68
#define MODE_H_ACTIVE_PIXELS 720

#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH 5
#define MODE_V_SYNC_WIDTH 5
#define MODE_V_BACK_PORCH 72
#define MODE_V_ACTIVE_LINES 576
*/

/*
#define MODE_H_SYNC_POLARITY 0
#define MODE_H_FRONT_PORCH 12
#define MODE_H_SYNC_WIDTH 64
#define MODE_H_BACK_PORCH 68
#define MODE_H_ACTIVE_PIXELS 720

#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH 5
#define MODE_V_SYNC_WIDTH 5
#define MODE_V_BACK_PORCH 40
#define MODE_V_ACTIVE_LINES 576
*/
/*
#define MODE_H_SYNC_POLARITY 0
#define MODE_H_FRONT_PORCH 12
#define MODE_H_SYNC_WIDTH 109
#define MODE_H_BACK_PORCH 68
#define MODE_H_ACTIVE_PIXELS 720

#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH 5
#define MODE_V_SYNC_WIDTH 5
#define MODE_V_BACK_PORCH 39
#define MODE_V_ACTIVE_LINES 576
*/


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



__attribute__((aligned(4))) uint16_t *framebuffer;
__attribute__((aligned(4))) uint16_t line_buffer_rgb444[1024];

uint32_t info_avi_p[64];
uint32_t info_aif_p[64];
uint32_t acr_p[64];
uint32_t asp_p[255];
uint32_t info_avi_len;
uint32_t info_aif_len;
uint32_t acr_len;
uint32_t asp_len;

audio_sample_t audio;
volatile int framecnt;


//Audio Related
#define AUDIO_BUFFER_SIZE   256
audio_sample_t      audio_buffer[AUDIO_BUFFER_SIZE];
const int16_t sine[32] = {
    0x8000,0x98f8,0xb0fb,0xc71c,0xda82,0xea6d,0xf641,0xfd89,
    0xffff,0xfd89,0xf641,0xea6d,0xda82,0xc71c,0xb0fb,0x98f8,
    0x8000,0x6707,0x4f04,0x38e3,0x257d,0x1592,0x9be,0x276,
    0x0,0x276,0x9be,0x1592,0x257d,0x38e3,0x4f04,0x6707
};
audio_ring_t  audio_ring;
struct repeating_timer audio_timer;
volatile int audiolock;

#define FRAMEBUFFER_WIDTH MODE_H_ACTIVE_PIXELS
#define FRAMEBUFFER_HEIGHT MODE_V_ACTIVE_LINES / 2

#define MODE_H_TOTAL_PIXELS (                \
    MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + \
    MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS)
#define MODE_V_TOTAL_LINES (                 \
    MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + \
    MODE_V_BACK_PORCH + MODE_V_ACTIVE_LINES)

#define HSTX_CMD_RAW (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT (0x1u << 12)
#define HSTX_CMD_TMDS (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP (0xfu << 12)

// ----------------------------------------------------------------------------
// HSTX command lists

// Lists are padded with NOPs to be >= HSTX FIFO size, to avoid DMA rapidly
// pingponging and tripping up the IRQs.

static uint32_t vblank_line_vsync_off[] = {
    HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
    SYNC_V1_H1,
    HSTX_CMD_NOP};

static uint32_t vblank_line_vsync_on[] = {
    HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
    SYNC_V0_H1,
    HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
    SYNC_V0_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
    SYNC_V0_H1,
    HSTX_CMD_NOP};

static uint32_t vactive_line[] = {
	HSTX_CMD_RAW_REPEAT | (MODE_H_FRONT_PORCH),
	SYNC_V1_H1,
	HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH),
	SYNC_V1_H0,
	HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH-W_PREAMBLE-W_GUARDBAND),
	SYNC_V1_H1,
	HSTX_CMD_RAW_REPEAT | W_PREAMBLE,
	SYNC_V1_H1_WITH_PREAMBLE,
	HSTX_CMD_RAW_REPEAT | W_GUARDBAND,
	VIDEO_LEADING_GUARD_BAND,
	HSTX_CMD_TMDS | MODE_H_ACTIVE_PIXELS
};


void init_avi_info_packet(void)
{
	int len = 0;

	data_packet_t avi_info_frame;
	data_island_stream_t di_str;

	set_AVI_info_frame(&avi_info_frame, SCAN_INFO_NO_DATA, RGB,
			   ITU601, PIC_ASPECT_RATIO_4_3, SAME_AS_PAR, FULL, _720x576P50);
	encode(&di_str, &avi_info_frame, 0, 0);

	info_avi_p[len++] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	info_avi_p[len++] = SYNC_V0_H1;
	info_avi_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH - W_DATA_ISLAND - W_PREAMBLE);
	info_avi_p[len++] = SYNC_V0_H0;
	info_avi_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	info_avi_p[len++] = SYNC_V0_H0_WITH_DATA_ISLAND_PREAMBLE;
	info_avi_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

	/* convert from the two symbols per word for each channel to one
	 * symbol per word containing all three channels format */
	for (int i = 0; i < N_DATA_ISLAND_WORDS; i++) {
		info_avi_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		info_avi_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

	info_avi_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS);
	info_avi_p[len++] = SYNC_V0_H1;

	info_avi_len = len;
}

void init_aif_info_packet(void)
{
	int len = 0;

	data_packet_t aif_info_frame;
	data_island_stream_t di_str;

	set_audio_info_frame(&aif_info_frame, 48000);
	encode(&di_str, &aif_info_frame, 0, 0);

	info_aif_p[len++] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	info_aif_p[len++] = SYNC_V0_H1;
	info_aif_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH - W_DATA_ISLAND - W_PREAMBLE);
	info_aif_p[len++] = SYNC_V0_H0;
	info_aif_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	info_aif_p[len++] = SYNC_V0_H0_WITH_DATA_ISLAND_PREAMBLE;
	info_aif_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

	/* convert from the two symbols per word for each channel to one
	 * symbol per word containing all three channels format */
	for (int i = 0; i < N_DATA_ISLAND_WORDS; i++) {
		info_aif_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		info_aif_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

	info_aif_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS);
	info_aif_p[len++] = SYNC_V0_H1;

	info_aif_len = len;
}

void init_acr_packet(void)
{
	int len = 0;

	data_packet_t acr;
	data_island_stream_t di_str;

	set_audio_clock_regeneration(&acr, 27000, 6144);
	encode(&di_str, &acr, 0, 0);


	acr_p[len++] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	acr_p[len++] = SYNC_V0_H1;
	acr_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH - W_DATA_ISLAND - W_PREAMBLE);
	acr_p[len++] = SYNC_V0_H0;
	acr_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	acr_p[len++] = SYNC_V0_H0_WITH_DATA_ISLAND_PREAMBLE;
	acr_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

	/* convert from the two symbols per word for each channel to one
	 * symbol per word containing all three channels format */
	for (int i = 0; i < N_DATA_ISLAND_WORDS; i++) {
		acr_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		acr_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

	acr_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS);
	acr_p[len++] = SYNC_V0_H1;

	acr_len = len;
}



void init_asp_packet(void)
{
	int len = 0;

	data_packet_t asp;
	data_island_stream_t di_str;

    audio_sample_t *audio_sample_ptr = get_read_pointer(&audio_ring);
    framecnt = set_audio_sample(&asp, audio_sample_ptr, 4, framecnt);
    increase_read_pointer(&audio_ring, 4);
    encode(&di_str, &asp, 1, 1);

    asp_p[len++] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	asp_p[len++] = SYNC_V1_H1;
	asp_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH - W_DATA_ISLAND - W_PREAMBLE);
	asp_p[len++] = SYNC_V1_H0;
	asp_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	asp_p[len++] = SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE;
	asp_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;


    /* convert from the two symbols per word for each channel to one
	 * symbol per word containing all three channels format */
	for (int i = 0; i < N_DATA_ISLAND_WORDS; i++) {
		asp_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		asp_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

    audio_sample_ptr = get_read_pointer(&audio_ring);
    framecnt = set_audio_sample(&asp, audio_sample_ptr, 4, framecnt);
    increase_read_pointer(&audio_ring, 4);
    encode(&di_str, &asp, 1, 1);
    asp_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	asp_p[len++] = SYNC_V1_H1;//SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE;
	asp_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

    	for (int i = 0; i < N_DATA_ISLAND_WORDS-0; i++) {
		asp_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		asp_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

    audio_sample_ptr = get_read_pointer(&audio_ring);
    framecnt = set_audio_sample(&asp, audio_sample_ptr, 4, framecnt);
    increase_read_pointer(&audio_ring, 4);
    encode(&di_str, &asp, 1, 1);
    asp_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	asp_p[len++] = SYNC_V1_H1;//SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE;
	asp_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

    	for (int i = 0; i < N_DATA_ISLAND_WORDS-0; i++) {
		asp_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		asp_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

    audio_sample_ptr = get_read_pointer(&audio_ring);
    framecnt = set_audio_sample(&asp, audio_sample_ptr, 4, framecnt);
    increase_read_pointer(&audio_ring, 4);
    encode(&di_str, &asp, 1, 1);
    asp_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	asp_p[len++] = SYNC_V1_H1;//SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE;
	asp_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

    	for (int i = 0; i < N_DATA_ISLAND_WORDS-0; i++) {
		asp_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		asp_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

    audio_sample_ptr = get_read_pointer(&audio_ring);
    framecnt = set_audio_sample(&asp, audio_sample_ptr, 4, framecnt);
    increase_read_pointer(&audio_ring, 4);
    encode(&di_str, &asp, 1, 1);
    asp_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	asp_p[len++] = SYNC_V1_H1;//SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE;
	asp_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

    	for (int i = 0; i < N_DATA_ISLAND_WORDS-0; i++) {
		asp_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		asp_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}
    

	asp_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS - ((W_DATA_ISLAND+W_PREAMBLE) * 4));
	asp_p[len++] = SYNC_V1_H1;
    asp_p[len++] = HSTX_CMD_NOP;

	asp_len = len;
}


void init_hsync_asp_packet(void)
{
	int len = 0;

	data_packet_t asp;
	data_island_stream_t di_str;

    audio_sample_t *audio_sample_ptr = get_read_pointer(&audio_ring);
    framecnt = set_audio_sample(&asp, audio_sample_ptr, 4, framecnt);
    increase_read_pointer(&audio_ring, 4);
    encode(&di_str, &asp, 1, 1);

    asp_p[len++] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	asp_p[len++] = SYNC_V1_H1;
	asp_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH - W_DATA_ISLAND - W_PREAMBLE);
	asp_p[len++] = SYNC_V1_H0;
	asp_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	asp_p[len++] = SYNC_V1_H1_WITH_DATA_ISLAND_PREAMBLE;
	asp_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;


    /* convert from the two symbols per word for each channel to one
	 * symbol per word containing all three channels format */
	for (int i = 0; i < N_DATA_ISLAND_WORDS; i++) {
		asp_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		asp_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}
    

	asp_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS - ((W_DATA_ISLAND+W_PREAMBLE) * 4));
	asp_p[len++] = SYNC_V1_H1;
    asp_p[len++] = HSTX_CMD_NOP;
}

// ----------------------------------------------------------------------------
// DMA logic

#define DMACH_PING 0
#define DMACH_PONG 1
#define DMACH_MEMCPY 2
#define DMACH_PIO 3

// First we ping. Then we pong. Then... we ping again.
static bool dma_pong = false;

// A ping and a pong are cued up initially, so the first time we enter this
// handler it is to cue up the second ping after the first ping has completed.
// This is the third scanline overall (-> =2 because zero-based).
volatile uint v_scanline = 2;
volatile uint scanout;
volatile uint scanout_end;
//static uint v_scanline_half = 2;

// During the vertical active period, we take two IRQs per scanline: one to
// post the command list, and another to post the pixels.
static bool vactive_cmdlist_posted = false;
static bool first_line_output_done = false;

void __scratch_x("") dma_irq_handler()
{
    // dma_pong indicates the channel that just finished, which is the one
    // we're about to reload.
    uint ch_num = dma_pong ? DMACH_PONG : DMACH_PING;
    dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
    dma_hw->intr = 1u << ch_num;
    dma_pong = !dma_pong;

    dma_channel_config c = dma_get_channel_config(ch_num);
/*
    if (v_scanline >= MODE_V_FRONT_PORCH && v_scanline < (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH))
    {
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        ch->read_addr = (uintptr_t)vblank_line_vsync_on;
        ch->transfer_count = count_of(vblank_line_vsync_on);
        first_line_output_done = false;
    }
*/
    if (v_scanline >= MODE_V_FRONT_PORCH && v_scanline < (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH)+48)
    {
//        gpio_put(28,1);
        //audiolock = 1;    
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        if (v_scanline == MODE_V_FRONT_PORCH) {
                    audiolock = 0;
            ch->read_addr = (uintptr_t)info_avi_p;
			ch->transfer_count = info_avi_len;
        } else if (v_scanline == MODE_V_FRONT_PORCH +1 ){
            ch->read_addr = (uintptr_t)info_aif_p;
			ch->transfer_count = info_aif_len;
        } else if (v_scanline == MODE_V_FRONT_PORCH +2 ){
            ch->read_addr = (uintptr_t)acr_p;
			ch->transfer_count = acr_len;

        /*    
        } else if (v_scanline >= MODE_V_FRONT_PORCH +3 && v_scanline <= 48){
            ch->read_addr = (uintptr_t)asp_p;
			ch->transfer_count = asp_len;
        } else if (v_scanline == MODE_V_FRONT_PORCH +4 ){
            ch->read_addr = (uintptr_t)asp_p;
			ch->transfer_count = asp_len;
        } else if (v_scanline == MODE_V_FRONT_PORCH +5 ){
            ch->read_addr = (uintptr_t)asp_p;
			ch->transfer_count = asp_len;
        } else if (v_scanline == MODE_V_FRONT_PORCH +6 ){
            ch->read_addr = (uintptr_t)asp_p;
			ch->transfer_count = asp_len;
        */                               
        } else {
			ch->read_addr = (uintptr_t)vblank_line_vsync_on;
			ch->transfer_count = count_of(vblank_line_vsync_on);
		}
        first_line_output_done = false;
        framecnt++;
    }


    else if (v_scanline < MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH)
    {
//        gpio_put(28,0);
        if(v_scanline == MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH){
            ch->read_addr = (uintptr_t)vblank_line_vsync_off;
            ch->transfer_count = count_of(vblank_line_vsync_off);
        } else {
            ch->read_addr = (uintptr_t)asp_p;
			ch->transfer_count = asp_len;
        }

    }
    else if (!vactive_cmdlist_posted)
    {
        ch->read_addr = (uintptr_t)vactive_line;
        ch->transfer_count = count_of(vactive_line);
        vactive_cmdlist_posted = true;
    }
    else
    {
        //if (v_scanline == 100) audiolock = 0;
        // This is for posting the pixel data for the active line
        // Calculate the actual framebuffer line to read based on the current scanline and the doubling logic
        uint framebuf_line_index;
        if (!first_line_output_done)
        {
            // First output of the current framebuffer line
            framebuf_line_index = (v_scanline - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES)) / 2;
            first_line_output_done = true;
        }
        else
        {
            // Second output of the current framebuffer line
            framebuf_line_index = (v_scanline - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES) - 1) / 2;
            first_line_output_done = false; // Reset for the next unique framebuffer line
        }
//        if (first_line_output_done)gpio_put(27,1);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        ch->read_addr = (uintptr_t)&framebuffer[framebuf_line_index * MODE_H_ACTIVE_PIXELS];
        ch->transfer_count = MODE_H_ACTIVE_PIXELS / sizeof(uint16_t); // / sizeof(uint32_t);
        vactive_cmdlist_posted = false;
//        gpio_put(27,0);
    }

    if (!vactive_cmdlist_posted)
    {
        v_scanline = (v_scanline + 1) % MODE_V_TOTAL_LINES;
        // If we just finished the second output of a framebuffer line,
        // and it was the last active scanline, reset first_line_output_done for the next frame
        if (v_scanline >= (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES) && !first_line_output_done)
        {
            // This condition needs to be carefully checked.
            // When v_scanline wraps around to 0, it indicates the start of a new frame.
            // We only need to reset first_line_output_done at the beginning of the *active display period*.
            // The current logic handles the toggling for every two scanlines correctly.
            // However, at the very beginning of the active display period, first_line_output_done must be false.
            // It's already initialized to false. The only potential issue is if the frame ends mid-doubling.
            // Since we reset it when we output the second line, it should be false at the start of every active line.
        }
    }
}

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

void __not_in_flash_func(get_line)(void)
{

    uint32_t pixel12;

    for (int i = 0; i < SAMPLES_PER_LINE; ++i)
    {

        // while (!(gpioc_lo_in_get() & (1 << pixelclock))){}
        while ((pixel12 = gpioc_lo_in_get()) & (1 << pixelclock))
        {
        }
        uint8_t b4 = (pixel12 >> 7) & 0x0F; // Bits 8-11 für Rot
        uint8_t g4 = (pixel12 >> 4) & 0x0F; // Bits 4-7 für Grün
        uint8_t r4 = pixel12 & 0x0F;        // Bits 0-3 für Blau
        line_buffer_rgb444[i] = colour_rgb444(r4 << 4, g4 << 4, b4 << 4);
    }
}

void __not_in_flash_func(get_pio_line)(void)
{
    uint32_t pixel12;
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_put_blocking(pio, sm, SAMPLES_PER_LINE/2 - 1);
    pio_sm_set_enabled(pio, sm, true);

    for (int i = 0; i < SAMPLES_PER_LINE; ++i)
    {

        // pixel12 = pio_sm_get_blocking(pio, sm);
        while ((pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm))) != 0)
        {
        }
        pixel12 = pio->rxf[sm];
        uint8_t b4 = (pixel12 >> 8) & 0x07; // Bits 8-11 für Rot
                b4 = (b4 << 1);
        uint8_t g4 = (pixel12 >> 4) & 0x0F; // Bits 4-7 für Grün
        uint8_t r4 = pixel12 & 0x0F;        // Bits 0-3 für Blau
        line_buffer_rgb444[i] = colour_rgb444(r4 << 4, g4 << 4, b4 << 4);
    }
    pio_sm_set_enabled(pio, sm, false);
}

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
        return true; // Gültiger Impuls
    } else {
        return false; // Zu kurz, wird als Störung ignoriert
    }
}



void core1_entry()
{
    uint16_t x = 0, y = 0, z = 0;
    while (1)
    {

/*        
        while (gpio_get(vsync)) {
            tight_loop_contents(); 
        }
        uint32_t start_time_us = time_us_32();
        while (!gpio_get(vsync)) {
            tight_loop_contents();
        }
        uint32_t end_time_us = time_us_32();
        uint32_t pulse_duration_us = end_time_us - start_time_us;
        if (pulse_duration_us >= 20) {
*/
        if (waitForValidPulse(vsync,20)){

            for (y = 0; y < 21; y++)
            {
                get_pio_line();
            }
            scanout = v_scanline;
            for (y = 0; y < 288; y++)
            {
                get_pio_line();
                dma_memcpy(framebuffer + (MODE_H_ACTIVE_PIXELS * y), line_buffer_rgb444 + 69, MODE_H_ACTIVE_PIXELS * 2);
            }
            scanout_end = v_scanline;
            /*
                while (!(gpioc_lo_in_get() & (1 << vsync)))
            {
            }
            */
            while(waitForValidPulse(vsync,20)){}
        }



/*
        while (gpioc_lo_in_get() & (1 << vsync))
        {
        }
        while (!(gpioc_lo_in_get() & (1 << vsync)))
        {
        }
        //v_scanline = MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH;

        //v_scanline = MODE_V_TOTAL_LINES-( MODE_V_BACK_PORCH +2);

        for (y = 0; y < 21; y++)
        {
            get_pio_line();
        }
        scanout = v_scanline;
        for (y = 0; y < 288; y++)
        {
            get_pio_line();
            dma_memcpy(framebuffer + (MODE_H_ACTIVE_PIXELS * y), line_buffer_rgb444 + 69, MODE_H_ACTIVE_PIXELS * 2);
        }
        scanout_end = v_scanline;
               while (!(gpioc_lo_in_get() & (1 << vsync)))
        {
        }
*/
    }
}


void gpio_callback(uint gpio, uint32_t events) {
    if(gpio==vsync)  v_scanline = MODE_V_TOTAL_LINES-( MODE_V_BACK_PORCH);
}

//bool audio_timer_callback(void) {
bool audio_timer_callback(struct repeating_timer *t) {
    int size = get_write_size(&audio_ring, false);
    audio_sample_t *audio_ptr = get_write_pointer(&audio_ring);
    audio_sample_t sample;
    static uint sample_count = 0;
    for (int cnt = 0; cnt < size; cnt++) {
        sample.channels[0] = sine[sample_count % 32];
        sample.channels[1] = sine[sample_count % 32];
        *audio_ptr++ = sample;
        sample_count++;
    }
    increase_write_pointer(&audio_ring, size);
 
    return true;
}


int main(void)
{


    //External LDO

    //vreg_set_voltage(VREG_VOLTAGE_0_85);


    for (uint i = 0; i <= 47; i++) {
    gpio_init(i);
    gpio_set_dir(i, 0);
    gpio_disable_pulls(i);
    //gpio_pull_up(i);
    }


    sleep_ms(50);
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_80);
    sleep_ms(50);
    qmi_hw->m[0].timing = 0x40000204;
    sleep_ms(50);
//    set_sys_clock_khz(270000, true);
    sleep_ms(50);

    size_t num_pixels = (size_t)FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT;
    size_t buffer_size_bytes = num_pixels * sizeof(uint16_t);
    framebuffer = (uint16_t *)malloc(buffer_size_bytes);


    init_avi_info_packet();
    init_aif_info_packet();
    init_acr_packet();
    audio_ring_set(&audio_ring, audio_buffer, AUDIO_BUFFER_SIZE);
    add_repeating_timer_ms(2, audio_timer_callback, NULL, &audio_timer);
    sleep_ms(100);
    init_asp_packet();
    audiolock=1;

    for (uint8_t i = 0; i < 12; i++)
    {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_input_hysteresis_enabled(i, true);
    }

    //gpio_init(28);
    //gpio_set_dir(28, GPIO_OUT);

    //gpio_init(27);
    //gpio_set_dir(27, GPIO_OUT);

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

/*
    hw_write_masked(
    &pll_sys_hw->fbdiv_int,
    224 << PLL_FBDIV_INT_LSB,
    PLL_FBDIV_INT_BITS);
*/


/*
    hw_write_masked(
        &clocks_hw->clk[clk_sys].div,
        8 << CLOCKS_CLK_SYS_DIV_FRAC_LSB,
        CLOCKS_CLK_SYS_DIV_FRAC_BITS);
*/


    reset_block(RESETS_RESET_HSTX_BITS);


    hw_write_masked(
        &clocks_hw->clk[clk_hstx].ctrl,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS << CLOCKS_CLK_HSTX_CTRL_AUXSRC_LSB,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_BITS);

    hw_write_masked(
        &clocks_hw->clk[clk_hstx].div,
        2 << CLOCKS_CLK_HSTX_DIV_INT_LSB,
        CLOCKS_CLK_HSTX_DIV_INT_BITS);

    unreset_block_wait(RESETS_RESET_HSTX_BITS);

    /* RGB444 */
    hstx_ctrl_hw->expand_tmds =
        3 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB | // Red channel: 4 bits (NBITS = 3)
        0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |   // Red channel: Rotate right by 0 bits (its LSB is at bit 0)
        3 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB | // Green channel: 4 bits (NBITS = 3)
        4 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |   // Green channel: Rotate right by 4 bits (its LSB is at bit 4)
        3 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB | // Blue channel: 4 bits (NBITS = 3)
        8 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;    // Blue channel: Rotate right by 8 bits (its LSB is at bit 8)

    hstx_ctrl_hw->expand_shift =
        2 << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
        16 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
        1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
        0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

    // Serial output config: clock period of 5 cycles, pop from command
    // expander every 5 cycles, shift the output shiftreg by 2 every cycle.
    hstx_ctrl_hw->csr = 0;
    hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EXPAND_EN_BITS |
        5u << HSTX_CTRL_CSR_CLKDIV_LSB |
        5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
        2u << HSTX_CTRL_CSR_SHIFT_LSB |
        HSTX_CTRL_CSR_EN_BITS;

    // Assign clock pair to two neighbouring pins:
    hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
    hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
    for (uint lane = 0; lane < 3; ++lane)
    {
        // For each TMDS lane, assign it to the correct GPIO pair based on the
        // desired pinout:
        static const int lane_to_output_bit[3] = {0, 6, 4};
        int bit = lane_to_output_bit[lane];
        // Output even bits during first half of each HSTX cycle, and odd bits
        // during second half. The shifter advances by two bits each cycle.
        uint32_t lane_data_sel_bits =
            (lane * 10) << HSTX_CTRL_BIT0_SEL_P_LSB |
            (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
        // The two halves of each pair get identical data, but one pin is inverted.
        hstx_ctrl_hw->bit[bit] = lane_data_sel_bits;
        hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
    }

    for (int i = 12; i <= 19; ++i)
    {
        gpio_set_function(i, 0); // HSTX
    }

    // Both channels are set up identically, to transfer a whole scanline and
    // then chain to the opposite channel. Each time a channel finishes, we
    // reconfigure the one that just finished, meanwhile the opposite channel
    // is already making progress.
    dma_channel_config c;
    c = dma_channel_get_default_config(DMACH_PING);
    channel_config_set_chain_to(&c, DMACH_PONG);
    channel_config_set_dreq(&c, DREQ_HSTX);
    dma_channel_configure(
        DMACH_PING,
        &c,
        &hstx_fifo_hw->fifo,
        vblank_line_vsync_off,
        count_of(vblank_line_vsync_off),
        false);

    c = dma_channel_get_default_config(DMACH_PONG);
    channel_config_set_chain_to(&c, DMACH_PING);
    channel_config_set_dreq(&c, DREQ_HSTX);
    dma_channel_configure(
        DMACH_PONG,
        &c,
        &hstx_fifo_hw->fifo,
        vblank_line_vsync_off,
        count_of(vblank_line_vsync_off),
        false);

    dma_hw->ints0 = (1u << DMACH_PING) | (1u << DMACH_PONG);
    dma_hw->inte0 = (1u << DMACH_PING) | (1u << DMACH_PONG);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    dma_channel_start(DMACH_PING);

    pio = pio0;
    uint offset = pio_add_program(pio, &video_capture_program);
    sm = pio_claim_unused_sm(pio, true);
    video_capture_program_init(pio, sm, offset, 0);

    // pio->input_sync_bypass = (1u<<22); //bypass clock sync

    //for (uint8_t i = 0; i < 12; i++)
    //{
    //    pio->input_sync_bypass = (1u<<i); //bypass sync
    //}

    pio_set_irq0_source_enabled(pio,pis_interrupt0, true);
    pio_interrupt_clear(pio, sm);

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_interrupt_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    multicore_launch_core1(core1_entry);

    

    while (1)
    {

     //pll_usb_hw->fbdiv_int = 225;
     //pll_usb_hw->fbdiv_int = 224;

/*     
    pll_sys_hw->fbdiv_int = 224;
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    pll_sys_hw->fbdiv_int = 225;
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
    asm ("nop");asm ("nop");asm ("nop");
*/

/*
    hw_write_masked(
    &pll_sys_hw->fbdiv_int,
    224 << PLL_FBDIV_INT_LSB,
    PLL_FBDIV_INT_BITS);

    sleep_us(1);
    hw_write_masked(
    &pll_sys_hw->fbdiv_int,
    225 << PLL_FBDIV_INT_LSB,
    PLL_FBDIV_INT_BITS);
    sleep_us(1);
    hw_write_masked(
    &pll_sys_hw->fbdiv_int,
    225 << PLL_FBDIV_INT_LSB,
    PLL_FBDIV_INT_BITS);
    sleep_us(1);
*/


    //   while(!audiolock){
    //    audiolock = 1;
    //    //init_asp_packet();
    //   }
    }
}
