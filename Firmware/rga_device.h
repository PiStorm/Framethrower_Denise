// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT
#ifndef RGA_DEVICE_H
#define RGA_DEVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "rga_common.h"


#define RING_BUFFER_SIZE 64
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1)

typedef struct {
    volatile uint16_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

extern RingBuffer rb_rx;
extern RingBuffer rb_tx;

static inline bool rb_push(RingBuffer *rb, uint16_t data) {
    uint16_t next = (rb->head + 1) & RING_BUFFER_MASK;
    if (next == rb->tail) return false; // Voll
    rb->buffer[rb->head] = data;
    rb->head = next;
    return true;
}

static inline bool rb_pop(RingBuffer *rb, uint16_t *data) {
    if (rb->head == rb->tail) return false; // Leer
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) & RING_BUFFER_MASK;
    return true;
}
void __not_in_flash_func(rga_process_20ms)(void);

#endif // RGA_DEVICE_H