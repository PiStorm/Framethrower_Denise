// © Copyright 2025 Claude Schwarz
// SPDX-License-Identifier: MIT

#include "fifo.h"
#include "hardware/dma.h" 

void fifo_init(fifo_t *f) {
    f->head = 0;
    f->tail = 0;
}

bool fifo_write(fifo_t *f, uint16_t data) {
    // Nächste Position für den Schreibzeiger berechnen
    uint32_t next_tail = (f->tail + 1) & (FIFO_SIZE - 1);

    // Prüfen, ob der FIFO voll ist
    if (next_tail == f->head) {
        return false; // FIFO ist voll
    }

    // Datum schreiben und Zeiger aktualisieren
    f->buffer[f->tail] = data;
    f->tail = next_tail;
    return true;
}

bool fifo_read(fifo_t *f, uint16_t *data) {
    // Prüfen, ob der FIFO leer ist
    if (f->head == f->tail) {
        return false; // FIFO ist leer
    }

    // Datum lesen und Zeiger aktualisieren
    *data = f->buffer[f->head];
    f->head = (f->head + 1) & (FIFO_SIZE - 1);
    return true;
}

bool fifo_is_empty(const fifo_t *f) {
    return f->head == f->tail;
}

bool fifo_is_full(const fifo_t *f) {
    // Berechne die nächste Position des tail-Zeigers und vergleiche mit head
    return ((f->tail + 1) & (FIFO_SIZE - 1)) == f->head;
}

uint32_t fifo_get_level(const fifo_t *f) {
    // Berechnet die Differenz unter Berücksichtigung des Überlaufs (wrap around)
    return (f->tail - f->head) & (FIFO_SIZE - 1);
}

bool fifo_write_buffer(fifo_t *f, const uint16_t *buffer, uint32_t count) {
    // Prüfen, ob genügend freier Platz für den gesamten Buffer vorhanden ist.
    // Der FIFO braucht immer einen freien Slot, um voll von leer zu unterscheiden.
    uint32_t available_space = (f->head - f->tail - 1) & (FIFO_SIZE - 1);
    if (count > available_space) {
        return false; // Nicht genug Platz
    }

    // Den gesamten Buffer in den FIFO kopieren
    for (uint32_t i = 0; i < count; i++) {
        f->buffer[f->tail] = buffer[i];
        f->tail = (f->tail + 1) & (FIFO_SIZE - 1);
    }
    return true;
}

bool fifo_read_buffer(fifo_t *f, uint16_t *buffer, uint32_t count) {
    // Prüfen, ob genügend Elemente zum Lesen im FIFO sind.
    if (fifo_get_level(f) < count) {
        return false; // Nicht genug Daten
    }

    // Den gesamten Buffer aus dem FIFO kopieren
    for (uint32_t i = 0; i < count; i++) {
        buffer[i] = f->buffer[f->head];
        f->head = (f->head + 1) & (FIFO_SIZE - 1);
    }
    return true;
}

void __not_in_flash_func(dma_memcpy)(void *dst, const void *src, size_t len)
{

    dma_channel_config c = dma_channel_get_default_config(1);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_FORCE);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        1,
        &c,
        dst,     // Zieladresse
        src,     // Quelladresse
        len / 4, // Anzahl der 32-Bit-Wörter
        true     // Starte den Transfer
    );
    dma_channel_wait_for_finish_blocking(1);
}

void __not_in_flash_func(dma_memcpy_non_block)(void *dst, const void *src, size_t len)
{

    dma_channel_config c = dma_channel_get_default_config(2);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_FORCE);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        2,
        &c,
        dst,     // Zieladresse
        src,     // Quelladresse
        len / 4, // Anzahl der 32-Bit-Wörter
        true     // Starte den Transfer
    );
}
