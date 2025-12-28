#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rga_host.h"
#include "rga_common.h" 

#define BENCH_SIZE 8192

uint8_t bench_pattern[BENCH_SIZE];

void print_dot_progress(int current, int total) {
    if (current % (total / 20) == 0) {
        printf(".");
        fflush(stdout);
    }
}

bool perform_safety_benchmark(void) {
    printf("Start Pre-Flight Check (8KB RAM Test)...\n");

    for(int i=0; i<BENCH_SIZE; i++) {
        bench_pattern[i] = (uint8_t)(i & 0xFF);
    }

    printf("  [1/2] Write RAM: ");
    for (int i = 0; i < BENCH_SIZE; i += 2) {
        // Big Endian Word bauen
        uint16_t word = (bench_pattern[i] << 8) | bench_pattern[i+1];
        if (!rga_exec_cmd(CMD_WRITE, i, word, NULL)) {
            printf(" ERROR! (Timeout/Write at Adr %d)\n", i);
            return false;
        }
        print_dot_progress(i, BENCH_SIZE);
    }
    printf(" OK\n");

    for(volatile int k=0; k<5000; k++);

    printf("  [2/2] Verfify RAM: ");
    int errors = 0;
    for (int i = 0; i < BENCH_SIZE; i += 2) {
        uint16_t val_in = 0;
        if (!rga_exec_cmd(CMD_READ, i, 0, &val_in)) {
            printf(" ERROR! (Timeout/Read at Adr %d)\n", i);
            return false;
        }

        uint8_t hi = (val_in >> 8) & 0xFF;
        uint8_t lo = val_in & 0xFF;

        if (hi != bench_pattern[i] || lo != bench_pattern[i+1]) {
            // Nur den ersten Fehler anzeigen, um Spam zu vermeiden
            if (errors == 0) printf("\n    Data mismatch at Adr %d: Soll %02X%02X, Ist %02X%02X", 
                                    i, bench_pattern[i], bench_pattern[i+1], hi, lo);
            errors++;
        }
        print_dot_progress(i, BENCH_SIZE);
    }

    if (errors > 0) {
        printf("\n  Error: %d corrupted data found!\n", errors);
        return false;
    }

    printf(" OK (100% Integrity)\n");
    return true;
}

int main(int argc, char **argv) {
    printf("=== Framethrower Denise Updater ===\n");
    const char* fw_filename = "Framethrower_Denise.bin";

    printf("Initialise connection...\n");
    rga_flush_pipe();

    if (!perform_safety_benchmark()) {
        printf("\n*** ERROR ***\n");
        printf("Hardware-Test failed.\n");
        printf("No update will be performed, to prevent bricking.\n");
        printf("Please check if Framethrower is mounted correctly and has the latest firmware flashed.\n");
        return 10;
    }

    printf("\nSystem stable. Start Firmware Update...\n");
    printf("File: %s\n", fw_filename);

    if (rga_update_firmware(fw_filename)) {
        printf("\n------------------------------------------------\n");
        printf(" SUCCESS!\n");
        printf(" Framethrower will reboot now.\n");
        printf("------------------------------------------------\n");
    } else {
        printf("\n------------------------------------------------\n");
        printf(" UPDATE FAILED!\n");
        printf(" Communication error.\n");
        printf(" Framethrower will boot the old firmware from flash.\n");
        printf("------------------------------------------------\n");
        return 20;
    }

    return 0;
}

