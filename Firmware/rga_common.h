// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#ifndef RGA_COMMON_H
#define RGA_COMMON_H

#include <stdint.h>

#define RAM_DISK_SIZE 8192
#define STX_MAGIC     0x55AA
#define ETX_MAGIC     0xEEFF

#define CMD_READ         0x01
#define CMD_WRITE        0x02

#define CMD_FLASH_ERASE  0x10 // Erase staging
#define CMD_FLASH_DATA   0x11 // Write staging
#define CMD_FLASH_COMMIT 0x12 // Commit staging to main flash and reboot

#define STATUS_OK        0x0000
#define STATUS_ERR_ADDR  0x0001
#define STATUS_ERR_CMD   0x0002
#define STATUS_ERR_FLASH 0x0003
#define STATUS_ERR_CRC   0xFFFF

// Staging Area beginnt bei 1 MB Offset
#define FLASH_STAGING_OFFSET 0x00100000
#define FLASH_PAGE_SIZE      256
#define FLASH_SECTOR_SIZE    4096

#endif // RGA_COMMON_H