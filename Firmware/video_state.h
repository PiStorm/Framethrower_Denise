#include <stdbool.h>
#include <stdint.h>

typedef struct {
    volatile bool laced;
    volatile bool isPAL;
    volatile uint32_t last_total_lines;
    volatile int scanline_level;
    volatile int scanline_level_laced;
} VideoState;

extern VideoState video_state;