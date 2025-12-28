#ifndef RGA_HOST_H
#define RGA_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include "rga_common.h"

// Low-Level API
bool rga_exec_cmd(uint8_t cmd, uint32_t addr, uint16_t payload_out, uint16_t *payload_in);

// High-Level API
bool rga_update_firmware(const char* filename);
void rga_flush_pipe(void);
bool rga_get_string(uint8_t cmd, char* buffer, int max_len);
bool rga_get_video_status(RGA_VideoStatus *status);
bool rga_set_scanlines(uint8_t level_normal, uint8_t level_laced);

#endif
