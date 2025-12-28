// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#include "rga_host.h"
#include "rga_common.h"
#include <stdio.h>
#include <stdlib.h>

#define TX_FIFO_ADDR  ((volatile uint16_t*)0x00dff1f2)
#define RX_FIFO_ADDR  ((volatile uint16_t*)0x00dff1f4)
#define DELAY_ADDR    ((volatile uint16_t*)0x00bfe001)
#define RETRY_LIMIT   350000

static uint16_t calc_crc(uint16_t* buf, int len) {
    uint16_t crc = 0;
    for(int i=0; i<len; i++) crc ^= buf[i];
    return crc;
}

void rga_flush_pipe(void) {
    int max = 1000;
    while(max-- > 0) {
        volatile uint16_t trash = *RX_FIFO_ADDR;
        if(trash == 0) break;
        for(volatile int k=0; k<50; k++);
    }
}

bool rga_exec_cmd(uint8_t cmd, uint32_t addr, uint16_t payload_out, uint16_t *payload_in) {
    uint16_t tx[8];
    int idx = 0;

    bool has_payload = (cmd == CMD_WRITE ||
                        cmd == CMD_FLASH_DATA ||
                        cmd == CMD_GET_VERSION ||
                        cmd == CMD_GET_GIT ||
                        cmd == CMD_GET_STATUS ||
                        cmd == CMD_SET_SCANLINE);

    tx[idx++] = STX_MAGIC;
    tx[idx++] = ((uint16_t)cmd << 8) | (has_payload ? 1 : 0);
    tx[idx++] = (uint16_t)(addr >> 16);
    tx[idx++] = (uint16_t)(addr & 0xFFFF);

    if (has_payload) {
        tx[idx++] = payload_out;
    }

    tx[idx] = calc_crc(tx, idx); idx++;
    tx[idx++] = ETX_MAGIC;

    for(volatile int k=0; k<50; k++); 
    for(int i=0; i<idx; i++) *TX_FIFO_ADDR = tx[i];

    volatile long tries = 0;
    bool connected = false;
    while(tries < RETRY_LIMIT) {
        if(*RX_FIFO_ADDR == STX_MAGIC) { connected = true; break; }
        tries++;
	volatile uint16_t trash = *DELAY_ADDR;
	trash = *DELAY_ADDR;
    }
    if (!connected) return false;

    uint16_t rx[6];
    rx[0] = STX_MAGIC;
    for(int i=1; i<6; i++) rx[i] = *RX_FIFO_ADDR;

    if(rx[5] != ETX_MAGIC) return false;
    if(rx[4] != calc_crc(rx, 4)) return false;
    if(rx[1] != STATUS_OK) return false;

    if (payload_in) *payload_in = rx[3];
    return true;
}

bool rga_update_firmware(const char* filename) {
    FILE *fp = fopen(filename, "rb");
    if (!fp) return false;

    fseek(fp, 0, SEEK_END);
    long filesize = ftell(fp);
    rewind(fp);

    printf("Erasing Staging Area...\n");
    if (!rga_exec_cmd(CMD_FLASH_ERASE, 0, 0, NULL)) {
        fclose(fp); return false;
    }

    sleep(5);
    printf("Uploading %ld bytes...\n", filesize);
    uint16_t buffer;
    long sent = 0;

    while(fread(&buffer, 2, 1, fp) == 1) {
        if (!rga_exec_cmd(CMD_FLASH_DATA, 0, buffer, NULL)) {
            printf("Error at byte %ld\n", sent);
            fclose(fp); return false;
        }
        sent += 2;
        if (sent % 1024 == 0) printf(".");
    }
    fclose(fp);

    printf("\nCommitting...\n");
    // Auf 4KB aufrunden
    uint32_t size_aligned = (filesize + 4095) & ~4095;
    rga_exec_cmd(CMD_FLASH_COMMIT, size_aligned, 0, NULL);

    return true;
}

bool rga_get_string(uint8_t cmd, char* buffer, int max_len) {
    int offset = 0;
    if (max_len > 0) buffer[0] = 0;

    while (offset < max_len - 1) {
        uint16_t chunk = 0;
        if (!rga_exec_cmd(cmd, 0, (uint16_t)offset, &chunk)) return false;

        char c1 = (chunk >> 8) & 0xFF;
        char c2 = chunk & 0xFF;
        if (c1 == 0) break;
        buffer[offset++] = c1;
        if (offset >= max_len - 1) break;
        if (c2 == 0) break;
        buffer[offset++] = c2;
    }
    buffer[offset] = 0;
    return true;
}

static uint32_t swap32(uint32_t v) {
    return ((v & 0xFF) << 24) | ((v & 0xFF00) << 8) | ((v >> 8) & 0xFF00) | ((v >> 24) & 0xFF);
}

bool rga_get_video_status(RGA_VideoStatus *status) {
    uint8_t *ptr = (uint8_t*)status;
    int size = sizeof(RGA_VideoStatus);
    int offset = 0;

    while (offset < size) {
        uint16_t chunk = 0;
        if (!rga_exec_cmd(CMD_GET_STATUS, 0, (uint16_t)offset, &chunk)) return false;
        if (offset < size) ptr[offset++] = (chunk >> 8) & 0xFF;
        if (offset < size) ptr[offset++] = chunk & 0xFF;
    }

    status->last_total_lines = swap32(status->last_total_lines);
    status->scanline_level = swap32(status->scanline_level);
    status->scanline_level_laced = swap32(status->scanline_level_laced);

    return true;
}

bool rga_set_scanlines(uint8_t level_normal, uint8_t level_laced) {
    if (level_normal > 4) level_normal = 4;
    if (level_laced > 4) level_laced = 4;
    uint16_t payload = (level_normal << 8) | level_laced;
    return rga_exec_cmd(CMD_SET_SCANLINE, 0, payload, NULL);
}
