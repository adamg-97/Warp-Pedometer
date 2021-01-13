# Pedometer and Activity Tracker
**Adam Goldney  
atg31  
Jesus College Cambridge**

https://github.com/adamg-97/Warp-Pedometer

A step counting and calorie tracking application developed for the FRDM KL03 evaluation board. Built using the [Warp firmware](https://github.com/physical-computation/Warp-firmware) platform developed by the University of Cambridge's [Physical Computation Laboratory](http://physcomp.eng.cam.ac.uk). 

![image](doc/setup.jpg)

## Building the application

**Prerequisites:** You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., via `apt-get` on Linux or via [MacPorts](https://www.macports.org) or [Homebrew](https://brew.sh) on macOS). You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

### 1. Compiling the application
The steps here are drawn from the compilation of the [Warp firmware](https://github.com/physical-computation/Warp-hardware).

First, specify the full path to the `bin` directory of your arm cross-compiler in the environment variable `ARMGCC_DIR` by changing line 3 of `build/ksdk1.1/build.sh`:

	export ARMGCC_DIR=<full-path-to-arm-cross-compiler>

Second, specify the full path to the application directory so that JLink can correctly locate the binary output file by changing line 3 of `tools/scripts/jlink.commands`:

	loadfile <full-path-to-application>/build/ksdk1.1/work/demos/Warp/armgcc/Warp/release/Warp.srec

Then you should be able to compile the application with:

	cd build/ksdk1.1/
	./build.sh

This copies the files from `Warp/src/boot/ksdk1.1.0/` into the KSDK tree, builds, and converts the binary to SREC. _When editing source, edit the files in `Warp/src/boot/ksdk1.1.0/`, not the files in the build location, since the latter are overwritten during each build._

### 2. Pin configuration
The following pins on the FRDM KL03 evaluation board are connected to an [SSD1331 OLED display](https://www.adafruit.com/product/684).

OLED display:
```
BOARD		DISPLAY
3.3V	->	VCC
GND	->	GND
PTA8	->	MOSI
PTA9	->	SCK
PTB13	->	OCS
PTA12	->	D/C
PTB0	->	RST

```

The MMA8451Q built in to the KL03 evaluation board is used to record accelerations and is interfaced by i2c.

### 3. Loading the application onto the evaluation board
Run the firmware downloader:

	JLinkExe -device MKL03Z32XXX4 -if SWD -speed 100000 -CommanderScript ../../tools/scripts/jlink.commands

## Source File Descriptions
The core application is in `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c`. Functions for the step counting, calorie tracking and activity selection algorithms are contained within `src/boot/ksdk1.1.0/pedometer.c`. The drivers for the display are in `devSSD1331.c` and for the MMA8541Q in `devMMA8451Q.c`. The section below briefly describes all the source files in this directory. 

##### `CMakeLists.txt`
This is the CMake configuration file. Edit this to change the default size of the stack and heap.

##### `SEGGER_RTT.*`
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### `SEGGER_RTT_Conf.h`
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### `SEGGER_RTT_printf.c`
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### `devSSD1331.*`
Driver for the SSD1331 OLED display.

##### `devMMA8451Q.*`
Driver for the MMA8451Q 3-axis accelerometer sensor.

##### `gpio_pins.c`
Definition of I/O pin configurations using the KSDK `gpio_output_pin_user_config_t` structure.

##### `gpio_pins.h`
Definition of I/O pin mappings and aliases for different I/O pins to symbolic names relevant to the Warp hardware design, via `GPIO_MAKE_PIN()`.

##### `startup_MKL03Z4.S`
Initialization assembler.

##### `warp-kl03-ksdk1.1-boot.c`
Main program including initialisation and while loops which runs the program continously. The while loop runs automatically on start-up after initialisation.

##### `pedometer.*`
Contains core functions called within main while loop function to read, interpret and track steps, calories burned and current activity mode.

##### `warp.h`
Constant and data structure definitions.

## Acknowledgements
The Warp firmware is developed by Phillip Stanley-Marbell and the University of Cambridge's Physical Computation Laboratory. This application was developed as a final project for the 4B25 Embedded Systems course at the Cambridge University Engineering Department.
