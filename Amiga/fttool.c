#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rga_host.h"
#include "rga_common.h"

int main(int argc, char **argv) {


    rga_flush_pipe();

    if (argc > 1) {
        if (strncmp(argv[1], "scanline=", 9) == 0) {
            int normal, laced;
            if (sscanf(argv[1] + 9, "%d,%d", &normal, &laced) == 2) {
                printf("set scanlines: Normal=%d, Laced=%d\n", normal, laced);
                if (rga_set_scanlines((uint8_t)normal, (uint8_t)laced)) {
                    return 0;
                } else {
                    printf("error!\n");
		    return 1;
                }
            } else {
                printf("usage: scanline=2,4 (level 2 normal, level 4 laced)\n");
            }
        }
    }

    char ver[32] = {0};
    char git[32] = {0};
    RGA_VideoStatus vstat;

    if (rga_get_string(CMD_GET_VERSION, ver, 32)) printf("FW Ver: %s\n", ver);
    if (rga_get_string(CMD_GET_GIT, git, 32))     printf("FW Git: %s\n", git);
    
    if (rga_get_video_status(&vstat)) {
        printf("Video:  %s %s, Lines: %d\n", 
               vstat.isPAL ? "PAL" : "NTSC", 
               vstat.laced ? "Interlaced" : "Progressive",
               vstat.last_total_lines);
        printf("Scanln: Level %d (Laced: %d)\n", vstat.scanline_level, vstat.scanline_level_laced);
    }

    return 0;
}
