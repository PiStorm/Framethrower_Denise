## Framethrower Denise - PiStorm Amiga native video passtrough

Framethrower samples the Amiga Video from Denise,converts it to a MIPI CSI2 datastream, which the Raspberry Pi on PiStorm can receive and show on HDMI

- [Hardware files, including Gerbers,BOM and Pick&Place](Hardware)
- [Firmware](Firmware)
- [Interactive BOM](https://htmlpreview.github.io/?https://github.com/PiStorm/Framethrower_Denise/blob/main/Hardware/Interactive_BOM/InteractiveBOM_Framethrower_Denise_Rev1.html)

  The Firmware is build with VSCode / PicoSDK 2.x

  Basic Setup and Information :arrow_forward: [	**Guide	**](setup.md) :arrow_backward:
  
  Precompiled Firmware can be found here [Firmware](https://github.com/PiStorm/Framethrower_Denise/raw/refs/heads/main/Firmware/build/Framethrower_Denise.uf2)
  

### Current status
Its still a work in progress, but I want to release the project now

- :heavy_check_mark: Scandoubling of non laced LORES and HIRES Modes
- :heavy_check_mark:  Deinterlace of laced LORES and HIRES Modes
- :heavy_check_mark:  PAL and NTSC
- :soon: Amiga based config tool (e.g. for scanlines)
- :x: Amiga audio passtrough
- :x: Amiga based firmware update tool

## Authors
Claude Schwarz
- aka [@captain-amygdala](https://github.com/captain-amygdala)

If you like this project and want to support me with a donation

   [![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=JQC4M73U9KKPG)

## The community around the PiStorm Project is best reached on Discord. There is a dedicated Framethrower support channel
[![](https://dcbadge.limes.pink/api/server/vyHr6nQeGn)](https://discord.gg/vyHr6nQeGn)

## Emu68 Setup
Simply add these parmaters to your cmdline.txt file on the SD Card (FAT32 Partition).
Don't forget to put all parameters in just one line, no new line.

```
unicam.boot unicam.integer
```

Also please set the HDMI vertical frequency to 50Hz for a PAL Amiga.

## Demo
Recorded directly from the Pi HDMI Output. RTG at 720p

![image](https://github.com/PiStorm/Framethrower_Denise/blob/main/ft_demo-1.gif)
