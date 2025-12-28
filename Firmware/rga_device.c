// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#include "rga_device.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <string.h>
#include "fwversion.h"
#include "video_state.h"

// Globale RingBuffer
RingBuffer rb_rx = { .head = 0, .tail = 0 };
RingBuffer rb_tx = { .head = 0, .tail = 0 };

VideoState video_status;

// Interne Variablen
__attribute__((aligned(4))) uint8_t ram_disk[RAM_DISK_SIZE] = {0};
__attribute__((aligned(4))) uint16_t rx_packet[8];
__attribute__((aligned(4))) int rx_index = 0;
__attribute__((aligned(4))) bool waiting_for_stx = true;

// Flash Buffer Variablen
__attribute__((aligned(4))) uint8_t flash_page_buf[FLASH_PAGE_SIZE];
__attribute__((aligned(4))) int flash_buf_idx = 0;
__attribute__((aligned(4))) uint32_t current_staging_addr = FLASH_STAGING_OFFSET;
__attribute__((aligned(4))) uint8_t sector_buf[FLASH_SECTOR_SIZE]; 

void rga_device_init(void) {
}

// String Helper: Holt 2 Zeichen basierend auf Offset
static uint16_t get_string_chunk(const char* str, uint16_t offset) {
    size_t len = strlen(str);
    if (offset >= len) return 0x0000; 

    uint8_t c1 = (uint8_t)str[offset];
    uint8_t c2 = 0;
    if (offset + 1 < len) c2 = (uint8_t)str[offset + 1];

    return (c1 << 8) | c2; // High Byte zuerst
}

static uint16_t __not_in_flash_func(calc_crc)(uint16_t* buf, int len) {
    uint16_t crc = 0;
    for(int i=0; i<len; i++) crc ^= buf[i];
    return crc;
}

// Flash Update
void __no_inline_not_in_flash_func(perform_update_and_reboot)(uint32_t image_size) {
    uint32_t addr = 0;
    //uint32_t ints = save_and_disable_interrupts(); 
    //pio_set_irq0_source_enabled(pio0, pis_interrupt0, false);
    //pio_set_irq1_source_enabled(pio0, pis_interrupt1, false);
    while (addr < image_size) {
        const uint8_t *src = (const uint8_t *)(XIP_BASE + FLASH_STAGING_OFFSET + addr);
        memcpy(sector_buf, src, FLASH_SECTOR_SIZE);
        flash_range_erase(addr, FLASH_SECTOR_SIZE);
        flash_range_program(addr, sector_buf, FLASH_SECTOR_SIZE);
        addr += FLASH_SECTOR_SIZE;
    }
    watchdog_enable(1, 1);
    while(1);
}

// Packet handler
static void __not_in_flash_func(process_complete_packet)(uint16_t* p, int len) {
    uint16_t received_crc = p[len-2];
    uint16_t received_etx = p[len-1];
    uint16_t calculated_crc = calc_crc(p, len-2);

    if (received_etx != ETX_MAGIC || received_crc != calculated_crc) return;

    uint8_t cmd = (p[1] >> 8) & 0xFF;
    volatile uint32_t addr = ((uint32_t)p[2] << 16) | p[3]; // Bei Commit = Size

    uint16_t status = STATUS_OK;
    uint16_t resp_data_hi = 0;
    uint16_t resp_data_lo = 0;

    if (cmd == CMD_GET_VERSION) {
        resp_data_lo = get_string_chunk(FW_VERSION, p[4]);
    }
    else if (cmd == CMD_GET_GIT) {
        resp_data_lo = get_string_chunk(GIT_HASH, p[4]);
    }

    else if (cmd == CMD_FLASH_ERASE) {
        //uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_STAGING_OFFSET, 1024 * 1024); // 1MB löschen
        //restore_interrupts(ints);
        current_staging_addr = FLASH_STAGING_OFFSET;
        flash_buf_idx = 0;
    }
    else if (cmd == CMD_FLASH_DATA) {
        uint16_t data = p[4];
        // Bus ist straight -> High Byte zuerst
        flash_page_buf[flash_buf_idx++] = (data >> 8) & 0xFF;
        flash_page_buf[flash_buf_idx++] = data & 0xFF;

        if (flash_buf_idx >= FLASH_PAGE_SIZE) {
            //uint32_t ints = save_and_disable_interrupts();
            flash_range_program(current_staging_addr, flash_page_buf, FLASH_PAGE_SIZE);
            //restore_interrupts(ints);
            current_staging_addr += FLASH_PAGE_SIZE;
            flash_buf_idx = 0;
        }
    }
    else if (cmd == CMD_FLASH_COMMIT) {
        if (flash_buf_idx > 0) { // Rest flushen
            memset(&flash_page_buf[flash_buf_idx], 0xFF, FLASH_PAGE_SIZE - flash_buf_idx);
            //uint32_t ints = save_and_disable_interrupts();
            flash_range_program(current_staging_addr, flash_page_buf, FLASH_PAGE_SIZE);
            //restore_interrupts(ints);
        }
        perform_update_and_reboot(addr); // addr = image_size
    }
    else if (addr >= RAM_DISK_SIZE - 1) {
        status = STATUS_ERR_ADDR;
    }
    else if (cmd == CMD_READ) {
        resp_data_lo = (ram_disk[addr] << 8) | ram_disk[addr+1];
    }
    else if (cmd == CMD_WRITE) {
        uint16_t data = p[4];
        ram_disk[addr] = (data >> 8) & 0xFF;
        ram_disk[addr+1] = data & 0xFF;
    }
    else {
        status = STATUS_ERR_CMD;
    }

    uint16_t response[6];
    response[0] = STX_MAGIC;
    response[1] = status;
    response[2] = resp_data_hi; 
    response[3] = resp_data_lo; 
    response[4] = calc_crc(response, 4);
    response[5] = ETX_MAGIC;

    for (int i = 0; i < 6; i++) rb_push(&rb_tx, response[i]);
}

void __not_in_flash_func(rga_process_20ms)() {
    uint16_t word;
    while (rb_pop(&rb_rx, &word)) {
        if (waiting_for_stx) {
            if (word == STX_MAGIC) {
                rx_index = 0;
                rx_packet[rx_index++] = word;
                waiting_for_stx = false;
            }
            continue; 
        }

        rx_packet[rx_index++] = word;
        int expected_len = 6; 
        if (rx_index >= 2) expected_len = 6 + (rx_packet[1] & 0xFF); 

        if (rx_index >= expected_len) {
            process_complete_packet(rx_packet, rx_index);
            waiting_for_stx = true; 
            rx_index = 0;
        }
        if (rx_index >= 8) { 
            waiting_for_stx = true; 
            rx_index = 0;
        }
    }
}