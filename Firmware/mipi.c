// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#include <stdio.h>
#include <alloca.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/resets.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/dma.h"
#include "mipi.h"

#define PREAMBLE_LEN 5
#define MAX_DATA_LEN 1440
#define END_LEN 2
#define MAX_TOTAL_LEN (PREAMBLE_LEN + MAX_DATA_LEN + END_LEN) // = 1447 Bytes

volatile bool mipi_busy = false;

static inline void start_hstx(void) {

    mipi_busy = true;
    uint8_t lead_in_zeros[32]={0};

    while (!(hstx_fifo_hw->stat & HSTX_FIFO_STAT_EMPTY_BITS));
    WAITFIFOEMPTY();

    gpio_put(PIN_LS_CLK_P, 0);
    gpio_put(PIN_HS_CLK_P, 0);
    DELAY_NS(50);
    gpio_put(PIN_LS_CLK_N, 0);
    gpio_put(PIN_HS_CLK_N, 0);
    DELAY_NS(100);
    gpio_set_function(PIN_HS_CLK_P, GPIO_FUNC_HSTX);
    gpio_set_function(PIN_HS_CLK_N, GPIO_FUNC_HSTX);
    DELAY_NS(200);
    dma_channel_transfer_from_buffer_now(0,lead_in_zeros,32); //zero


    gpio_put(PIN_LS_D_P, 0);
    gpio_put(PIN_HS_D0_P, 0);
    DELAY_NS(50);
    gpio_put(PIN_LS_D_N, 0);
    gpio_put(PIN_HS_D0_N, 0);
    DELAY_NS(150);
    gpio_set_function(PIN_HS_D0_P, GPIO_FUNC_HSTX);
    gpio_set_function(PIN_HS_D0_N, GPIO_FUNC_HSTX);
    
    //Clear the interrupt request.
    dma_hw->ints0 = 1u << 0;
    dma_channel_set_irq0_enabled(0, true);
}

static inline void stop_hstx(void) {
     dma_channel_set_irq0_enabled(0, false);
//TODO RUNOUT richtig
    gpio_set_function(PIN_HS_D0_P, GPIO_FUNC_SIO);
    gpio_set_function(PIN_HS_D0_N, GPIO_FUNC_SIO);
    gpio_put(PIN_LS_D_P, 1);
    gpio_put(PIN_LS_D_N, 1);
    gpio_put(PIN_HS_D0_P, 1);
    gpio_put(PIN_HS_D0_N, 1);


    while (!(hstx_fifo_hw->stat & HSTX_FIFO_STAT_EMPTY_BITS));
    WAITFIFOEMPTY();

    gpio_set_function(PIN_HS_CLK_P, GPIO_FUNC_SIO);
    gpio_set_function(PIN_HS_CLK_N, GPIO_FUNC_SIO);
    gpio_set_function(PIN_HS_D0_P, GPIO_FUNC_SIO);
    gpio_set_function(PIN_HS_D0_N, GPIO_FUNC_SIO);
    gpio_put_masked(0x000FF000,0x000FF000);
    mipi_busy = false;
}


void dma_handler() {
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << 0;
    stop_hstx();
}


void DMASetup(uint8_t *data) { 

    dma_channel_config c = dma_channel_get_default_config(0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, DREQ_HSTX);
    channel_config_set_high_priority( &c, true);

        dma_channel_configure(
        0,          // Channel to be configured
        &c,            // The configuration we just created
        &hstx_fifo_hw->fifo,           // The initial write address
        data,           // The initial read address
        1440, // Number of transfers; in this case each is 1 byte.
        false           // Start immediately.
    );
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

}


void mipi_init(void) {

    stop_hstx();
    reset_block(RESETS_RESET_HSTX_BITS);

    hw_write_masked(
        &clocks_hw->clk[clk_hstx].ctrl,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS << CLOCKS_CLK_HSTX_CTRL_AUXSRC_LSB,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_BITS
    );

    hw_write_masked(
        &clocks_hw->clk[clk_hstx].div,
        1u << CLOCKS_CLK_HSTX_DIV_INT_LSB ,
        CLOCKS_CLK_HSTX_DIV_INT_BITS    
        );

    unreset_block_wait(RESETS_RESET_HSTX_BITS);

    gpio_init(PIN_LS_CLK_P);
    gpio_init(PIN_LS_CLK_N);
    gpio_disable_pulls(PIN_LS_CLK_P);
    gpio_disable_pulls(PIN_LS_CLK_N);
    gpio_set_dir(PIN_LS_CLK_P, GPIO_OUT);
    gpio_set_dir(PIN_LS_CLK_N, GPIO_OUT);
    gpio_put(PIN_LS_CLK_P, 1);
    gpio_put(PIN_LS_CLK_N, 1);


    gpio_init(PIN_LS_D_P);
    gpio_init(PIN_LS_D_N);
    gpio_disable_pulls(PIN_LS_D_P);
    gpio_disable_pulls(PIN_LS_D_N);
    gpio_set_dir(PIN_LS_D_P, GPIO_OUT);
    gpio_set_dir(PIN_LS_D_N, GPIO_OUT);
    gpio_put(PIN_LS_D_P, 1);
    gpio_put(PIN_LS_D_N, 1);


    gpio_init(PIN_HS_D0_P);
    gpio_init(PIN_HS_D0_N);
    gpio_disable_pulls(PIN_HS_D0_P);
    gpio_disable_pulls(PIN_HS_D0_N);
    gpio_set_dir(PIN_HS_D0_P, GPIO_OUT);
    gpio_set_dir(PIN_HS_D0_N, GPIO_OUT);
    gpio_put(PIN_HS_D0_P, 1);
    gpio_put(PIN_HS_D0_N, 1);

    gpio_init(PIN_HS_CLK_P);
    gpio_init(PIN_HS_CLK_N);
    gpio_disable_pulls(PIN_HS_CLK_P);
    gpio_disable_pulls(PIN_HS_CLK_N);
    gpio_set_dir(PIN_HS_CLK_P, GPIO_OUT);
    gpio_set_dir(PIN_HS_CLK_N, GPIO_OUT);
    gpio_put(PIN_HS_CLK_P, 1);
    gpio_put(PIN_HS_CLK_N, 1);

    hstx_ctrl_hw->bit[PIN_HS_CLK_P - FIRST_HSTX_PIN] =
        HSTX_CTRL_BIT0_CLK_BITS;
    hstx_ctrl_hw->bit[PIN_HS_CLK_N - FIRST_HSTX_PIN] =
        HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;


    //#define SDR 0   
    #ifdef SDR
        hstx_ctrl_hw->bit[PIN_HS_D0_P - FIRST_HSTX_PIN] =
        (0u << HSTX_CTRL_BIT0_SEL_P_LSB) |
        (0u << HSTX_CTRL_BIT0_SEL_N_LSB); //1
    hstx_ctrl_hw->bit[PIN_HS_D0_N - FIRST_HSTX_PIN] =
        (0u << HSTX_CTRL_BIT0_SEL_P_LSB) |
        (0u << HSTX_CTRL_BIT0_SEL_N_LSB) | //1
        HSTX_CTRL_BIT0_INV_BITS;

        hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EN_BITS |
        (1u << HSTX_CTRL_CSR_SHIFT_LSB) | //2
        (8u << HSTX_CTRL_CSR_N_SHIFTS_LSB) | //4
        (1u << HSTX_CTRL_CSR_CLKPHASE_LSB ) |
        (2u << HSTX_CTRL_CSR_CLKDIV_LSB);        
    #else
        hstx_ctrl_hw->bit[PIN_HS_D0_P - FIRST_HSTX_PIN] =
        (0u << HSTX_CTRL_BIT0_SEL_P_LSB) |
        (1u << HSTX_CTRL_BIT0_SEL_N_LSB); //1
    hstx_ctrl_hw->bit[PIN_HS_D0_N - FIRST_HSTX_PIN] =
        (0u << HSTX_CTRL_BIT0_SEL_P_LSB) |
        (1u << HSTX_CTRL_BIT0_SEL_N_LSB) | //1
        HSTX_CTRL_BIT0_INV_BITS;

        hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EN_BITS |
        (2u << HSTX_CTRL_CSR_SHIFT_LSB) | //2
        (4u << HSTX_CTRL_CSR_N_SHIFTS_LSB) | //4
        (1u << HSTX_CTRL_CSR_CLKPHASE_LSB ) |
        (1u << HSTX_CTRL_CSR_CLKDIV_LSB);        
    #endif


    gpio_set_slew_rate(PIN_HS_CLK_P,GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_HS_CLK_N,GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_HS_D0_P,GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_HS_D0_N,GPIO_SLEW_RATE_FAST);

    gpio_set_slew_rate(PIN_LS_CLK_P,GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_LS_CLK_N,GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_LS_CLK_P,GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_LS_CLK_N,GPIO_SLEW_RATE_FAST);


    gpio_set_drive_strength(PIN_LS_CLK_P,GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_LS_CLK_N,GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_LS_CLK_P,GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_LS_CLK_N,GPIO_DRIVE_STRENGTH_12MA);


    gpio_set_drive_strength(PIN_HS_CLK_P,GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_HS_CLK_N,GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_HS_D0_P,GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_HS_D0_N,GPIO_DRIVE_STRENGTH_12MA);

}


void __not_in_flash_func(mipiCsiFrameStart)(void) {
    uint8_t buffer[5] ={0xB8,0x00,0x00,0x00,0x00};
    start_hstx();
    dma_channel_transfer_from_buffer_now(0,buffer,5);
    //dma_channel_wait_for_finish_blocking(0);
    //while(mipi_busy){}
    //stop_hstx();
}

void __not_in_flash_func(mipiCsiFrameEnd)(void) {
    
    uint8_t buffer[5] ={0xB8,0x01,0x00,0x00,0x07};
    start_hstx();
    dma_channel_transfer_from_buffer_now(0,buffer,5);
    //dma_channel_wait_for_finish_blocking(0);
    //while(mipi_busy){}
    //stop_hstx();
}

void __not_in_flash_func(mipiCsiSendLong)(int type, uint8_t *data, int len) {

    static uint8_t combined_buffer[MAX_TOTAL_LEN]; 
    uint8_t preamble[PREAMBLE_LEN] = {0xB8, 0x22, 0xA0, 0x05, 0x1e};
    uint8_t end[END_LEN] = {0x00, 0xff};
    memcpy(combined_buffer, preamble, PREAMBLE_LEN);
    memcpy(combined_buffer + PREAMBLE_LEN, data, len); 
    memcpy(combined_buffer + PREAMBLE_LEN + len, end, END_LEN);
    int total_len_this_call = PREAMBLE_LEN + len + END_LEN;

    start_hstx();    
    dma_channel_transfer_from_buffer_now(0, combined_buffer, total_len_this_call);
    //dma_channel_wait_for_finish_blocking(0);
    //while(mipi_busy){}
    //stop_hstx();
}