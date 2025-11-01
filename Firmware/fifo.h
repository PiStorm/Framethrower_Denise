// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FIFO_SIZE (4 * 1024)

typedef struct {
    volatile uint16_t buffer[FIFO_SIZE];
    volatile uint32_t head; // Lesezeiger
    volatile uint32_t tail; // Schreibzeiger
} fifo_t;

void fifo_init(fifo_t *f);
bool fifo_write(fifo_t *f, uint16_t data);
bool fifo_read(fifo_t *f, uint16_t *data);
bool fifo_is_empty(const fifo_t *f);
bool fifo_is_full(const fifo_t *f);
uint32_t fifo_get_level(const fifo_t *f);
bool fifo_write_buffer(fifo_t *f, const uint16_t *buffer, uint32_t count);
bool fifo_read_buffer(fifo_t *f, uint16_t *buffer, uint32_t count);
bool fifo_write_buffer_dma(fifo_t *f, const uint16_t *buffer, uint32_t count);
bool fifo_read_buffer_dma(fifo_t *f, uint16_t *buffer, uint32_t count);
void dma_memcpy(void *dst, const void *src, size_t len);
void dma_memcpy_non_block(void *dst, const void *src, size_t len);
#endif // FIFO_H