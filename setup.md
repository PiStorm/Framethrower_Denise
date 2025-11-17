## Hardware Setup

You need a Raspberry Pi camera cable or a generic A/B Type 15pin 1.0mm pitch FPC

1. Remove Denise from the Socket 
2. Place Framethrower into the Denise Socket
3. Place Denise into the Framethrower Denise Socket
4. Connect the FPC Cable to Framethrower (metallic FPC contacts facing down)
5. Connect the FPC Cable to the Raspberry Pi/PiStorm "Camera" connector
   
## EMU68 Setup

<span style="color: #f50d0d">**EMU68 1.0.5 and 1.0.6 have already basic Framethrower support build**</span>

There are some known limitations and bugs with this early support in Emu68

1. Boot Mode passtrough is broken with unicam.smooth scaler settings
2. NTSC modes shown "garbage" in the lower part of the screen
3. unicam.smooth scaler settings **might** lead to recoverable gurus on workbench

This limitations and bugs are already addressed and upcoming Emu68 1.1 will probably have this fixed :)

The easiest way to configure Emu68 for Framethrower is to edit the cmdline.txt file on the Emu68 SD-Card FAT partition.
If there is no cmdline.txt to be found anywhere on the SD card (please search for the file, some readymade distributions put them elsewhere) 
then just create one with a text editor.
Keep in mind, with cmdline.txt all parameters ***must*** be written in just one line, seperated by space

By adding these parameters at the end of the old parameter list you can enable Framethrower




```unicam.boot unicam.integer``` 

This enables integer/pixel perfect passtrough and automatic RTG/Native switching



```unicam.boot unicam.smooth unicam.b=20 unicam.c=0 unicam.phase=60```

This enables scaled to 4:3 fullscreen passtrough and automatic RTG/Native switching





Its advised to set the HDMI output to 50Hz for a PAL Amiga, otherwise you likely notice strong tearing effects.
To do this here are exemplary two settings for config.txt

720p50Hz Output :

```
disable_overscan=1
# ScreenMode: 720p 50hz
hdmi_group=1
hdmi_mode=19
```


1080p50Hz Output :

```
disable_overscan=1
# ScreenMode: 1080p 50hz
hdmi_group=1
hdmi_mode=31
```


## Framethrower Firmware update

At a later stage there will be a Amiga based Firmware update utility, but until then Firmware updates
must be done by the build in USB-C interface. No driver or special tools are needed for that.
Just make sure you use a USB-C to USB-A cable, as a direct USB-C to USB-C connection wont work

Precompiled Firmware .UF2, ready to flash : [Firmware](https://github.com/PiStorm/Framethrower_Denise/raw/refs/heads/main/Firmware/build/Framethrower_Denise.uf2)

1. Power off the Amiga
2. Press and hold the button on Framethrower
3. While the button is still pressed, power on the Amiga
4. You can release now the button and connect a USB-C to USB-A Cable to Framethrower
5. Connecting the USB Cable to a PC immediatly a drive called "RP2350" will show up
6. Just copy or drag&drop the Firmware .UF2 file to the RP2350 drive
7. The drive will disapear, the Framethrower is updated
