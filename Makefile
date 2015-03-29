#
# Makefile for building Arduino sketches (programs) with Arduino from the
# command line.
#
#
# Code tweaking:
# Copyright (C) 2014-2015  F1RMB, Daniel Caujolle-Bert <f1rmb.daniel@gmail.com>
#
# Original version:
# Copyright (C) 2011-2013 by Volker Kuhlmann
#               http://volker.top.geek.nz/contact.html
# All rights reserved.
# Released under the terms of the GNU General Public License (GPL) Version 2.
# See http://www.gnu.org/ for details.
#
# Download: http://volker.top.geek.nz/arduino/
#
# Features:
#  * Works with all Arduino versions.
#    Tested are Arduino versions 0.22, 0.23, 1.0 - 1.0.5.
#  * Does not need the IDE program or java, but needs the IDE installed to get
#    the core sources and board definitions.
#  * Supports any number of Arduino libraries, 3rd-party Arduino libraries, or
#    Arduino libraries you made yourself.
#  * Automatic dependencies generation.
#  * Compiles with the macros ARDUINO and F_CPU being defined, same as the IDE.
#  * Compiles into a subdirectory (similar to eclipse).
#  * Runs gcc and the binutils out of $PATH by default, allowing to test
#    different gcc versions easily (same as the IDE).
#  * By default compiles with a lot more warnings, to detect shoddy progamming.
#    Comment out some of the OPT_WARN lines to turn this off.
#  * Works with multiple .ino/.pde files.
#  * Compile with LTO (link-time optimisations) by adding "LTO=1" to the make
#    command line. Note this requires at least gcc 4.6.2!
#  * Tested extensively with Arduino 1.0 and 0.23.
#  * To make your C++ source files (.cpp) compile with any Arduino version, use
#    something like this:
#      #if ARDUINO >= 100
#      #include <Arduino.h>
#      #else
#      #include <WProgram.h>
#      #endif
#
# Detailed instructions for using this Makefile:
#
#  1. Copy this file into the directory with your sketch.
#     There should be a file with the extension .ino (previously .pde).
#     cd into this directory.
#
#  2. Below, modify the settings of various variables, but at least
#        PROJECT
#        ARDUINO_MODEL
#        PORT
#        ARDUINO
#        ARDUINO_DIR
#        ARDUINO_VARIANT
#        ARDUINO_LIBS and USER_LIBS
#
#     Check the other variables, but they're not likely needing to change.
#     See the descriptions at the variables for details.
#
#  3. Function prototypes.
#     Sorry, they're necessary with every programming. The Arduino IDE tries
#     to create them automatically and does get it mostly (but not always)
#     right.
#     If you know of a way to create prototypes automaticlly, let me know.
#     It might be easiest to start with the prototypes created by the IDE.
#     Run the build in the IDE, locate the project source file created by the
#     IDE (for a project XYZ it's something like
#     /tmp/build4303013692903917981.tmp/XYZ.cpp, and copy the prototypes.
#     They're just before the first variable that is declared.
#
#     If you have multiple .ino/.pde files, put the prototypes for all of them
#     into the main file (XYZ.ino).
#
#  4. Run "make" to compile/verify your program.
#
#  5. Run "make upload" (or "make up" for short) to upload your program to the
#     Arduino board. The board is reset first.
#
#  6. Run "make help" for more options.
#
# This is an almost complete re-write of
# http://shallowsky.com/software/arduino/Makefile-1.0-v6 from Akkana Peck,
# which was an adaptation of the Makefile by mellis, eighthave, oli.keller
# shipped with the Arduino IDE until version 0.17. They didn't work too well
# and contained many errors. Maybe around 10% of that remains.
# Except for those remaining parts, this file is
#
# To do:
#  * Split this into 2 parts - one generic one, and one that is specific to the
#    program and that includes the generic part.
#
# Version 1.1 - 06 Jan 2012
# Version 1.2 - 10 Jan 2012
#   * Finish documentation.
# Version 1.3 - 17 Jan 2012
#   * Fix handling of multiple .pde files. They have to be concatenated
#     together, not compiled separately. Amend documentation.
# Version 1.4 - 23 Jan 2012
#   * Add make variable SD, to include the SD library and define USE_SD.
#   * Add -mrelax. Significant code reduction, but crashes binutils 2.22.
#     Use 2.19.1.
# Version 1.5 - 14 Jul 2012
#   * Add -flto-report with -flto.
# Version 1.6 - 04 May 2013
#   * Show avrdude details in showvars target.
#   * Set the programmer variables only from board.txt if not already set.
# Version 1.7 - 18 Oct 2013
#   * Ensure vpath is not set with an empty argument.
#   * Show values with quotes in target showvars.
# Version 1.8 - 20 Oct 2013
#   * Add OPT_OTHER.
#   * Delete gcc temp files.
#   * Move gcc temp files to object directory if not there.
#   * Fix various targets' dependency on the output directory.
#   * Move user macro definitions to the top section.
#   * Improve tar file creation.
#   * Allow projects without .ino or .pde project file.
#
# Makefile version (only used for help text).
MKVERSION = 1.8 - 20 Oct 2013

# Determine operating system environment.
# Possible values are (tested): Linux, FreeBSD (on 8.1), ...
OSNAME =	$(shell uname)

# Name of the program and source .ino (previously .pde) file.
# No extension here (e.g. PROJECT = Blink).
PROJECT =	sketch

# Project version. Only used for packing the source into an archive.
VERSION =	1.0

# Arduino model. E.g. atmega328, mega, mega2560, uno.
# Valid model names can be found in $(ARDUINO_DIR)/hardware/arduino/boards.txt
# This must be set to a valid model name.
#ARDUINO_MODEL = leonardo
#ARDUINO_MODEL = atmega328
#ARDUINO_MODEL = mega2560
ARDUINO_MODEL = nano328
#ARDUINO_MODEL = mega

# USB port the Arduino board is connected to.
# Linux: e.g. /dev/ttyUSB0, or /dev/ttyACM0 for the Uno.
# BSD:   e.g. /dev/cuaU0
# It is a good idea to use udev rules to create a device name that is constant,
# based on the serial number etc. of the USB device.
# See e.g. 97-avr-ftdi.rules from the same download location as this Makefile.
PORT =		/dev/ttyUSB0
#PORT =		/dev/ttyUSB_myboard

# Arduino version (e.g. 23 for 0023, or 105 for 1.0.5).
# Make sure this matches ARDUINO_DIR below!
#ARDUINO = 	23
ARDUINO = 	105

# Location of the official Arduino IDE.
# E.g. /usr/local/arduino, or $(HOME)/arduino
# Make sure this matches ARDUINO above!
#ARDUINO_DIR =	/usr/local/pckg/arduino/arduino-0023
ARDUINO_DIR =	/usr/share/arduino

# Arduino 0.x based on 328P now need the new programmer protocol.
#AVRDUDE_PROGRAMMER = arduino

# Arduino core sources.
ARDUINO_CORE =	$(ARDUINO_DIR)/hardware/arduino/cores/arduino

# Arduino variant (for Arduino 1.0+).
# Directory containing the pins_arduino.h file.
ifeq "$(ARDUINO_MODEL)" "leonardo"
ARDUINO_VARIANT=$(ARDUINO_DIR)/hardware/arduino/variants/leonardo
else
endif

# Standard Arduino libraries used, e.g. EEPROM, LiquidCrystal.
# Give the name of the directory containing the library source files.
ARDUINO_LIBS =
#ARDUINO_LIBS += EEPROM
#ARDUINO_LIBS += LiquidCrystal
#ARDUINO_LIBS += SPI
#ifdef SD  # Comment out this condition to always use the SD library.
#ARDUINO_LIBS += SD
#endif

# User libraries (in ~/sketchbook/libraries/).
# Give the name of the directory containing the library source files.
USER_LIBDIR =	~/sketchbook/libraries
#USER_LIBS =	ClickEncoder
#USER_LIBS +=	TimerOne
#USER_LIBS +=	TimerThree
#USER_LIBS +=	Volker
#USER_LIBS +=	AnyPrint
#USER_LIBS +=	CLIserial
#USER_LIBS +=	MemoryFree
#USER_LIBS +=	RTC_DS3232
#USER_LIBS +=	LIS3LV02
#USER_LIBS +=	DataFlash
#USER_LIBS +=	Flash
#USER_LIBS +=	Streaming
#USER_LIBS +=	SysConfig
#USER_LIBS +=	MHIheatpump
#USER_LIBS +=	SRAM23K256
#USER_LIBS +=	DallasTemperature
#USER_LIBS +=	OneWire
#USER_LIBS +=	Sensirion
#USER_LIBS +=	SHT1x
#USER_LIBS +=	WS2811
#USER_LIBS +=	FastSPI_LED2

# Additional pre-compiled libraries to link with.
# Always leave the math (m) library last!
# The Arduino core library is automatically linked in.
# If the library is in a location the compiler doesn't already know, also
# give the directory with -L.
# Note this is dealing with real libraries (libXXX.a), not Arduino "libraries"!
LDLIBS =	-L/usr/lib/avr
#LDLIBS +=	-lm

LISTING_ARGS =	-h -S
LISTING_ARGS += -t -l -C -w

SYMBOL_ARGS =	-n
SYMBOL_ARGS +=	-C

# Directory in which files are created.
# Using the current directory ('.') is untested (and probably unwise).
OUTPUT =	output

# Where are tools like avr-gcc located on your system?
# If you set this, it must end with a slash!
#AVR_TOOLS_PATH = $(ARDUINO_DIR)/hardware/tools/avr/bin/
#AVR_TOOLS_PATH = /usr/bin/
AVR_TOOLS_PATH =

# Reset command to use.
# Possible values are: "stty", "python", "perl".
RESETCMD =	stty

### Macro definitions. Place -D or -U options here.
ARCHDEF =
ifeq "$(ARDUINO_MODEL)" "leonardo"
ARCHDEF += -D__AVR_ATmega32U4__
else
ifeq "$(ARDUINO_MODEL)" "atmega328"
ARCHDEF += -D__AVR_ATmega238__
else
ifeq "$(ARDUINO_MODEL)" "mega2560"
ARCHDEF += -D__AVR_ATmega2560__
else
ifeq "$(ARDUINO_MODEL)" "nano328"
ARCHDEF += -D__AVR_ATmega238__
endif
endif
endif
endif

CDEFS = $(ARCHDEF) -DUSE_EEPROM=1 -DUSE_ETHERNET=0 -DUSE_FIRMATA=0 -DUSE_LCD=1 -DUSE_LCD4884=0 -DUSE_ODB=0 -DUSE_SD=0 -DUSE_SERVO=0 -DUSE_SOFTSERIAL=0 -DUSE_SPI=0 -DUSE_STEPPER=0 -DUSE_TINYGPS=0 -DUSE_WIRE=0

ifdef LTO
CDEFS +=	-DLTO
endif
ifdef SD
CDEFS +=	-DUSE_SD
endif
ifdef mega
CDEFS +=	-DARDUINO_MEGA
endif

############################################################################
# Below here nothing should need to be changed.
############################################################################

#ifeq ($(shell test 0$(ARDUINO) -ge 100 && echo 1),1)
#endif

# Output hex format.
HEXFORMAT =	ihex

# Name of the dependencies file (used for "make depend").
# This doesn't actually work too well.
# Drop this idea and use auto-generated dependencies (*.d) instead.
DEPFILE =	$(OUTPUT)/Makefile.depend

# Name of the tar file in which to pack the user program up in.
TARFILE =	$(PROJECT)-$(VERSION).tar

# Default reset command if still unset.
RESETCMD ?=	stty

# Set Arduino core sources location to default, if still unset.
ARDUINO_CORE ?= $(ARDUINO_DIR)/hardware/arduino/cores/arduino

# Get the upload rate, CPU model, CPU frequency and avrdude programmer type
# from the IDE files.
UPLOAD_RATE ?= $(shell \
	sed "/$(ARDUINO_MODEL)\.upload.speed/ { s/.*=//; q }; d" \
		$(ARDUINO_DIR)/hardware/arduino/boards.txt \
	)
MCU ?= $(shell \
	sed "/$(ARDUINO_MODEL)\.build.mcu/ { s/.*=//; q }; d" \
		$(ARDUINO_DIR)/hardware/arduino/boards.txt \
	)
F_CPU ?= $(shell \
	sed "/$(ARDUINO_MODEL)\.build.f_cpu/ { s/.*=//; q }; d" \
		$(ARDUINO_DIR)/hardware/arduino/boards.txt \
	)

ifeq "$(ARDUINO_MODEL)" "leonardo"
USB_VID ?= $(shell \
	sed "/$(ARDUINO_MODEL)\.build.vid/ { s/.*=//; q }; d" \
		$(ARDUINO_DIR)/hardware/arduino/boards.txt \
	)

USB_PID ?= $(shell \
	sed "/$(ARDUINO_MODEL)\.build.pid/ { s/.*=//; q }; d" \
		$(ARDUINO_DIR)/hardware/arduino/boards.txt \
	)
#ifeq "$(ARDUINO_MODEL)" "leonardo"
USB_HACK = -DUSB_VID=$(USB_VID) -DUSB_PID=$(USB_PID) \
#endif
endif

AVRDUDE_PROGRAMMER ?= $(shell \
	sed "/$(ARDUINO_MODEL)\.upload.protocol/ { s/.*=//; q }; d" \
		$(ARDUINO_DIR)/hardware/arduino/boards.txt \
	)

# Try and guess PORT if it wasn't set previously.
# Note using shell globs most likely won't work, so try first port.
ifeq "$(OSNAME)" "Linux"
ifeq "$(ARDUINO_MODEL)" "uno"
    PORT ?= /dev/ttyACM0
else
    PORT ?= /dev/ttyUSB0
endif
else
    # Not Linux, so try BSD port name
    PORT ?= /dev/cuaU0
endif

# Try and guess ARDUINO_VARIANT if it wasn't set previously.
# Possible values for Arduino 1.0 are:
#   eightanaloginputs leonardo mega micro standard
# This makefile part is incomplete. Best set variant explicitly at the top.
# Default is "standard".
ifeq ($(ARDUINO_VARIANT),)
ifeq "$(ARDUINO_MODEL)" "mega"
ARDUINO_VARIANT ?= $(ARDUINO_DIR)/hardware/arduino/variants/mega
else
ifeq "$(ARDUINO_MODEL)" "mega2560"
ARDUINO_VARIANT=$(ARDUINO_DIR)/hardware/arduino/variants/mega
else
ifeq "$(ARDUINO_MODEL)" "micro"
ARDUINO_VARIANT ?= $(ARDUINO_DIR)/hardware/arduino/variants/micro
else
ARDUINO_VARIANT ?= $(ARDUINO_DIR)/hardware/arduino/variants/standard
endif
endif
endif
endif

### Sources

# Arduino core sources.
CORESRC =	$(wildcard $(ARDUINO_CORE)/*.c)
CORECXXSRC =	$(wildcard $(ARDUINO_CORE)/*.cpp)

# Arduino official library sources.
ALIBDIRS = $(wildcard \
		$(ARDUINO_LIBS:%=$(ARDUINO_DIR)/libraries/%) \
		$(ARDUINO_LIBS:%=$(ARDUINO_DIR)/libraries/%/utility) \
		)
ALIBSRC =	$(wildcard $(ALIBDIRS:%=%/*.c))
ALIBCXXSRC =	$(wildcard $(ALIBDIRS:%=%/*.cpp))

# All Arduino library sources.
ARDUINO_ALL_LIBS = $(notdir $(wildcard $(ARDUINO_DIR)/libraries/*))
ALIBALLDIRS = $(wildcard \
		$(ARDUINO_ALL_LIBS:%=$(ARDUINO_DIR)/libraries/%) \
		$(ARDUINO_ALL_LIBS:%=$(ARDUINO_DIR)/libraries/%/utility) \
		)
ALIBALLSRC =	$(wildcard $(ALIBALLDIRS:%=%/*.c))
ALIBALLCXXSRC =	$(wildcard $(ALIBALLDIRS:%=%/*.cpp))

# User library sources.
ULIBDIRS = $(wildcard \
		$(USER_LIBS:%=$(USER_LIBDIR)/%) \
		$(USER_LIBS:%=$(USER_LIBDIR)/%/utility) \
		)
ULIBSRC =	$(wildcard $(ULIBDIRS:%=%/*.c))
ULIBCXXSRC =	$(wildcard $(ULIBDIRS:%=%/*.cpp))

# User program sources.
SRC =		$(wildcard *.c)
CXXSRC =	$(wildcard ./libraries/*.cpp)
CXXSRCINO =	$(wildcard *.ino) $(wildcard *.pde)
prjino :=	$(findstring $(PROJECT).ino,$(CXXSRCINO))
prjpde :=	$(findstring $(PROJECT).pde,$(CXXSRCINO))
# Remove project.ino and project.pde from compilation.
CXXSRCINO :=	$(filter-out $(PROJECT).ino $(PROJECT).pde,$(CXXSRCINO))
# If project.ino or project.pde exist, add output/project.cpp to compilation.
ifneq "" "$(prjino)$(prjpde)"
CXXSRC +=	$(OUTPUT)$(if $(OUTPUT),/)$(PROJECT).cpp
# Remove project.cpp from compilation if project.ino or project.pde exist.
# (This will cause problems if OUTPUT is "."!)
#CXXSRC :=	$(filter-out $(PROJECT).cpp,$(CXXSRC))
endif
# Add the remaining C++ sources; put project.* first to find errors soon.
CXXSRC +=	$(wildcard *.cpp)
# Assembler sources.
ASRC =		$(wildcard *.S)

# Paths to check for source files (pre-requisites).
# (Note: The vpath directive clears the path if the argument is empty!)
ifneq "$(ARDUINO_CORE)" ""
  vpath % $(ARDUINO_CORE)
endif
ifneq "$(ALIBDIRS)" ""
  vpath % $(ALIBDIRS)
endif
ifneq "$(ULIBDIRS)" ""
  vpath % $(ULIBDIRS)
endif
vpath % .
ifneq "$(ALIBALLDIRS)" ""
  vpath % $(ALIBALLDIRS)
endif
# Either ensure the path to vpath is not empty, or use the VPATH variable.
# The manual says to separate paths with ":", but " " works as well.
#VPATH = 	$(ARDUINO_CORE) $(ALIBDIRS) $(ULIBDIRS) . $(ALIBALLDIRS)


### Include directories.
CINCS = \
	-I. \
	-I$(ARDUINO_CORE) \
	-I$(ARDUINO_VARIANT) \
	$(ALIBDIRS:%=-I%) \
	$(ULIBDIRS:%=-I%) \
	-I/usr/lib/avr/include \
	-I/usr/share/arduino/libraries/EEPROM \
	-I/usr/share/arduino/libraries/Ethernet \
	-I/usr/share/arduino/libraries/Firmata \
	-I/usr/share/arduino/libraries/Flash \
	-I/usr/share/arduino/libraries/LCD4884 \
	-I/usr/share/arduino/libraries/LCD4Bit_mod \
	-I/usr/share/arduino/libraries/LiquidCrystal \
	-I/usr/share/arduino/libraries/SD \
	-I/usr/share/arduino/libraries/SD/utility \
	-I/usr/share/arduino/libraries/Servo \
	-I/usr/share/arduino/libraries/SevenSegment \
	-I/usr/share/arduino/libraries/SoftwareSerial \
	-I/usr/share/arduino/libraries/SPI \
	-I/usr/share/arduino/libraries/Stepper \
	-I/usr/share/arduino/libraries/TinyGPS \
	-I/usr/share/arduino/libraries/Wire \
	-I/usr/share/arduino/libraries/Wire/utility \
	-I./ClickEncoder \
	-I./TimerOne \
	-I$(USER_LIBDIR)/


### Object and dependencies files.

# Arduino core.
COREOBJ =	$(addprefix $(OUTPUT)/,$(notdir \
			$(CORESRC:.c=.o) \
			$(CORECXXSRC:.cpp=.o) \
		))

# Arduino libraries used.
ALIBOBJ =	$(addprefix $(OUTPUT)/,$(notdir \
			$(ALIBSRC:.c=.o) \
			$(ALIBCXXSRC:.cpp=.o) \
		))

# All Arduino libraries.
ALIBALLOBJ =	$(addprefix $(OUTPUT)/,$(notdir \
			$(ALIBALLSRC:.c=.o) \
			$(ALIBALLCXXSRC:.cpp=.o) \
		))

# User libraries used.
ULIBOBJ =	$(addprefix $(OUTPUT)/,$(notdir \
			$(ULIBSRC:.c=.o) \
			$(ULIBCXXSRC:.cpp=.o) \
		))

# User program.
OBJ =		$(addprefix $(OUTPUT)/,$(notdir \
			$(SRC:.c=.o) \
			$(CXXSRC:.cpp=.o) \
			$(ASRC:.S=.o) \
		))

# All object files.
#ALLOBJ =	$(COREOBJ) $(ALIBOBJ) $(ULIBOBJ) $(OBJ)
ALLOBJ =	$(OBJ) $(ULIBOBJ) $(ALIBOBJ) $(COREOBJ)

# All dependencies files.
ALLDEPS =	$(ALLOBJ:%.o=%.d)


### More macro definitions.
# -DF_CPU and -DARDUINO are mandatory.
CDEFS += 	-DF_CPU=$(F_CPU) $(USB_HACK) -DARDUINO=$(ARDUINO)


### C/C++ Compiler flags.

# C standard level.
# c89   - ISO C90 ("ANSI" C)
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions (default for C)
##CSTANDARD =	-std=gnu99

# C++ standard level.
# empty   - default
# c++98   - 1998 ISO C++ standard plus amendments. ("ANSI" C++)
# gnu++98 - c++98 plus GNU extensions (default for C++)
# c++0x   - working draft of upcoming ISO C++0x standard; experimental
# gnu++0x - c++0x plus GNU extensions
#CXXSTANDARD =	-std=gnu++0x

# Optimisations.
OPT_OPTIMS =	-Os -x c++ -s
OPT_OPTIMS +=	-fno-exceptions -ffunction-sections -fdata-sections
##OPT_OPTIMS +=	-mrelax
# -mrelax crashes binutils 2.22, 2.19.1 gives 878 byte shorter program.
# The crash with binutils 2.22 needs a patch. See sourceware #12161.
ifdef LTO
OPT_OPTIMS +=	-flto
#OPT_OPTIMS +=	-flto-report
#OPT_OPTIMS +=	-fwhole-program
# -fuse-linker-plugin requires gcc be compiled with --enable-gold, and requires
# the gold linker to be available (GNU ld 2.21+ ?).
#OPT_OPTIMS +=	-fuse-linker-plugin
endif

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
#OPT_DEBUG =	-g2 -gstabs

# Warnings.
# A bug in gcc 4.3.x related to progmem might turn a warning into an error
# when using -pedantic. This patch works around the problem:
# http://volker.top.geek.nz/arduino/avr-libc-3.7.1-pgmspace_progmem-fix.diff
# Turning on all warnings shows a large number of less-than-optimal program
# locations in the Arduino sources. Some might turn into errors. Either fix
# your Arduino sources, or turn the warnings off.
OPT_WARN =	-Wall
#OPT_WARN +=	-pedantic
#OPT_WARN +=	-Wextra
#OPT_WARN +=	-Wmissing-declarations
#OPT_WARN +=	-Wmissing-field-initializers
#OPT_WARN +=	-Wsystem-headers
#OPT_WARN +=	-Wno-variadic-macros
OPT_WARN_C =	$(OPT_WARN)
#OPT_WARN_C +=	-Wmissing-prototypes
OPT_WARN_CXX =	$(OPT_WARN)

# Other.
OPT_OTHER =
# Save gcc temp files (pre-processor, assembler):
#OPT_OTHER +=	-save-temps

# Final combined.
CFLAGS =	$(OPT_OPTIMS) -mmcu=$(MCU) \
		$(CSTANDARD) $(CDEFS) \
		$(OPT_WARN) $(OPT_OTHER) $(CEXTRA)
CXXFLAGS =	-mmcu=$(MCU) \
		$(OPT_OPTIMS) $(CXXSTANDARD) $(CDEFS) \
		$(OPT_WARN) $(OPT_OTHER) $(CEXTRA)


### Assembler flags.

#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs

# Assembler standard level.
ASTANDARD =	-x assembler-with-cpp

# Final combined.
ASFLAGS =	-mmcu=$(MCU) \
		$(CDEFS) \
		$(ASTANDARD) $(ASEXTRA)


### Linker flags.

# Optimisation setting must match compiler's, esp. for -flto.

LDFLAGS =	-mmcu=$(MCU)
LDFLAGS +=	$(OPT_OPTIMS)
LDFLAGS +=	-Wl,--gc-sections -s
#LDFLAGS +=	-Wl,--print-gc-sections


### Programming / program uploading.

AVRDUDE_FLAGS =

# Do not verify.
#AVRDUDE_FLAGS+= -V

# Override invalid signature check.
#AVRDUDE_FLAGS+= -F

# Disable auto erase for flash memory. (IDE uses this too.)
AVRDUDE_FLAGS+= -D

# Quiet -q -qq / Verbose -v -vv.
AVRDUDE_FLAGS+= -q

AVRDUDE_FLAGS+= -p $(MCU) -c $(AVRDUDE_PROGRAMMER) -b $(UPLOAD_RATE)
AVRDUDE_FLAGS+= -P $(PORT)

# avrdude config file
AVRDUDE_FLAGS+= -C /etc/avrdude.conf
#AVRDUDE_FLAGS+= -C $(ARDUINO_DIR)/hardware/tools/avrdude.conf

AVRDUDE_WRITE_FLASH = -U flash:w:$(OUTPUT)/$(PROJECT).hex:i


### Programs

AVRPREFIX =	avr-
CC =		$(AVR_TOOLS_PATH)$(AVRPREFIX)gcc
CXX =		$(AVR_TOOLS_PATH)$(AVRPREFIX)g++
OBJCOPY =	$(AVR_TOOLS_PATH)$(AVRPREFIX)objcopy
OBJDUMP =	$(AVR_TOOLS_PATH)$(AVRPREFIX)objdump
AR =		$(AVR_TOOLS_PATH)$(AVRPREFIX)ar
SIZE =		$(AVR_TOOLS_PATH)$(AVRPREFIX)size
NM =		$(AVR_TOOLS_PATH)$(AVRPREFIX)nm
AVRDUDE =	$(AVR_TOOLS_PATH)avrdude
#AVRDUDE =	$(ARDUINO_DIR)/hardware/tools/avrdude
RM =		rm -f
RMDIR = 	rmdir
MV =		mv -f
ifeq "$(OSNAME)" "Linux"
    STTY =	stty -F $(PORT)
else
    # BSD uses small f
    STTY =	stty -f $(PORT)
endif


### Implicit rules

.SUFFIXES: .ino .pde .elf .hex .eep .lss .listing .sym .symbol
.SUFFIXES: .cpp .c .S .o .a

# Compile: create object files from C++ source files.
#	  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@ $(@:%.o=%.S) $(@:%.o=%.d)"
%.o $(OUTPUT)/%.o: %.cpp
	$(CXX) -o $@ $(CXXFLAGS) -c $< \
	  $(CINCS)
#	if [ -f "$(notdir $(@:.o=.s))" -a ! -f "$(@:.o=.s)" ]; then \
#	  mv "$(notdir $(@:.o=.s))" "$(dir $@)"; fi
#	if [ -f "$(notdir $(@:.o=.ii))" -a ! -f "$(@:.o=.ii)" ]; then \
#	  mv "$(notdir $(@:.o=.ii))" "$(dir $@)"; fi

%.o $(OUTPUT)/%.o: libraries/%.cpp
	$(CXX) -o $@ $(CXXFLAGS) -c $< \
	  $(CINCS)

# Compile: create object files from C source files.
#	  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@ $(@:%.o=%.S) $(@:%.o=%.d)"
%.o $(OUTPUT)/%.o: %.c
	$(CC) $(CFLAGS) $(CINCS) -c  $< -o $@
#	if [ -f "$(notdir $(@:.o=.s))" -a ! -f "$(@:.o=.s)" ]; then \
#	  mv "$(notdir $(@:.o=.s))" "$(dir $@)"; fi
#	if [ -f "$(notdir $(@:.o=.i))" -a ! -f "$(@:.o=.i)" ]; then \
#	  mv "$(notdir $(@:.o=.i))" "$(dir $@)"; fi

# Compile: create assembler files from C++ source files.
%.S $(OUTPUT)/%.S: %.cpp
	$(CXX) -o $@ -S $(CXXFLAGS) $< \
	  -MMD -MP -MF"$(@:%.S=%.d)" -MT"$(@:%.S=%.o) $@ $(@:%.S=%.d)" \
	  $(CINCS)

# Compile: create assembler files from C source files.
%.S $(OUTPUT)/%.S: %.c
	$(CC) -o $@ -S $(CFLAGS) $< \
	  -MMD -MP -MF"$(@:%.S=%.d)" -MT"$(@:%.S=%.o) $@ $(@:%.S=%.d)" \
	  $(CINCS)

# Assemble: create object files from assembler source files.
%.o $(OUTPUT)/%.o: %.S
	$(CC) -o $@ -c $(ASFLAGS) $< \
	  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@ $(@:%.o=%.S) $(@:%.o=%.d)" \
	  $(CINCS)

# Create extended listing file from object file.
%.lss %.listing: %.o
	$(OBJDUMP) $(LISTING_ARGS) $< > $@

%.hex: %.elf
	$(OBJCOPY) -O $(HEXFORMAT) -R .eeprom  -R .eesafe $< $@

%.eep: %.elf
	$(OBJCOPY) --no-change-warning -j .eeprom \
	--change-section-lma .eeprom=0 \
	-O $(HEXFORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss %.listing: %.elf
	$(OBJDUMP) $(LISTING_ARGS) $< > $@

# Create a symbol table from ELF output file.
%.sym %.symbol: %.elf
#	$(NM) $(SYMBOL_ARGS) $< > $@
	$(NM) $(SYMBOL_ARGS) $< | uniq > $@

# Pre-processing of Arduino .ino/.pde source files.
# It creates a .cpp file based with the same name as the .pde file.
# On top of the new .cpp file comes the Arduino.h/WProgram.h header.
# Then the .cpp file will be compiled. Errors during compile will
# refer to this new, automatically generated, file.
# Not the original .pde file you actually edit...
$(OUTPUT)/%.cpp: %.ino
#	echo >  $@ "// Automatically generated by Makefile. Don't edit."
#	echo >> $@ "#include <Arduino.h>"
	cat  > $@ $< $(CXXSRCINO)
$(OUTPUT)/%.cpp: %.pde
	echo > $@ "// Automatically generated by Makefile. Don't edit."
	echo >> $@ "#if ARDUINO >= 100"
	echo >> $@ "#include <Arduino.h>"
	echo >> $@ "#else"
	echo >> $@ "#include <WProgram.h>"
	echo >> $@ "#endif"
	cat  >> $@ $< $(CXXSRCINO)


### Explicit rules.

.PHONY: all build elf hex eep lss lst sym listing symbol size tar help extra
.PHONY: coff extcoff
.PHONY: reset reset_stty reset_python reset_perl upload up clean depend mkout
.PHONY: showvars showvars2

# Default target.
all:	elf hex size eep #listing symbol size

build:	elf hex

elf:	$(OUTPUT) $(OUTPUT)/$(PROJECT).elf
hex:	$(OUTPUT) $(OUTPUT)/$(PROJECT).hex
eep:	$(OUTPUT) $(OUTPUT)/$(PROJECT).eep
lss:	$(OUTPUT) $(OUTPUT)/$(PROJECT).lss
lst:	$(OUTPUT) $(OUTPUT)/$(PROJECT).lss
sym:	$(OUTPUT) $(OUTPUT)/$(PROJECT).sym
listing: $(OUTPUT) $(OUTPUT)/$(PROJECT).listing
symbol: $(OUTPUT) $(OUTPUT)/$(PROJECT).symbol
tar:	$(TARFILE).xz
extra:	$(patsubst %,$(OUTPUT)/$(PROJECT)_2%, .elf .hex .listing .symbol) \
	$(patsubst %,$(OUTPUT)/$(PROJECT)_3%, .elf .hex .listing .symbol) \
	$(patsubst %,$(OUTPUT)/$(PROJECT)_4%, .elf .hex .listing .symbol) \
	$(patsubst %,$(OUTPUT)/$(PROJECT)_6%, .elf .hex .listing .symbol) \
	$(patsubst %,$(OUTPUT)/$(PROJECT)_8%, .elf .hex .listing .symbol) \
	$(patsubst %,$(OUTPUT)/$(PROJECT)_9%, .elf .hex .listing .symbol) \
	$(patsubst %,$(OUTPUT)/$(PROJECT)_A%, .elf .hex .listing .symbol)

help:
	@printf "\
Arduino Makefile version $(MKVERSION) by Volker Kuhlmann\n\
Makefile targets (run \"make <target>\"):\n\
   all           Compile program and create listing, symbol list etc.\n\
   upload        Upload program to Arduino board (or just use 'up')\n\
   size          Show size of all .elf and .hex files in output directory\n\
   reset         Reset Arduino board\n\
   reset_stty    Reset using stty\n\
   reset_python  Reset using Python program\n\
   reset_perl    Reset using perl program\n\
   tar           Create tar file of program\n\
   dtr           Show current state of serial port's DTR line\n\
   showvars      Show almost all makefile variables\n\
   mkout         Create output directory\n\
   depend        Put all dependencies into one file. Doesn't work, don't use.\n\
   clean         Delete all generated files\n\
"

# Show variables. Essential when developing this makefile.
showvars:
	@make --no-print-directory $(MAKEVARS) showvars2 | $${PAGER:-less}
showvars2:
	: PROJECT = "$(PROJECT)", VERSION = "$(VERSION)"
	: ARDUINO = "$(ARDUINO)"
	: ARDUINO_MODEL = "$(ARDUINO_MODEL)"
	: F_CPU = "$(F_CPU)"
	: USB_VID = "$(USB_VID)"
	: USB_PID = "$(USB_PID)"
	: PORT = "$(PORT)"
	: UPLOAD_RATE = "$(UPLOAD_RATE)"
	: MCU = "$(MCU)"
	: AVRDUDE_PROGRAMMER = "$(AVRDUDE_PROGRAMMER)"
	: AVRDUDE = "$(AVRDUDE)"
	: AVRDUDE_FLAGS = "$(AVRDUDE_FLAGS)"
	: AVRDUDE_WRITE_FLASH = "$(AVRDUDE_WRITE_FLASH)"
	: ARDUINO_DIR = "$(ARDUINO_DIR)"
	: ARDUINO_CORE = "$(ARDUINO_CORE)"
	: ARDUINO_VARIANT = "$(ARDUINO_VARIANT)"
	: ARDUINO_LIBS = "$(ARDUINO_LIBS)"
	: ALIBDIRS = "$(ALIBDIRS)"
	: USER_LIBS = "$(USER_LIBS)"
	: ULIBDIRS = "$(ULIBDIRS)"
	: CINCS = "$(CINCS)"
	: SRC = "$(SRC)"
	: CXXSRC = "$(CXXSRC)"
	: CXXSRCINO = "$(CXXSRCINO)"
	: ASRC = "$(ASRC)"
	: ULIBSRC = "$(ULIBSRC)"
	: ULIBCXXSRC = "$(ULIBCXXSRC)"
	: ALIBSRC = "$(ALIBSRC)"
	: ALIBCXXSRC = "$(ALIBCXXSRC)"
	: CORESRC = "$(CORESRC)"
	: CORECXXSRC = "$(CORECXXSRC)"
	: CFLAGS = "$(CFLAGS)"
	: CXXFLAGS = "$(CXXFLAGS)"
	: COREOBJ = "$(COREOBJ)"
	: ALIBOBJ = "$(ALIBOBJ)"
	: ULIBOBJ = "$(ULIBOBJ)"
	: OBJ = "$(OBJ)"
	: ALLOBJ = "$(ALLOBJ)"
	: ALLDEPS = "$(ALLDEPS)"
	: VPATH = "$(VPATH)"

mkout $(OUTPUT):
	mkdir -p $(OUTPUT)

# Create core library.
$(OUTPUT)/libcore.a: $(COREOBJ)
	$(AR) rcsv $@ $(COREOBJ)

# Creating these other .a libraries is an experiment to find out whether
# it reduces code size further. It doesn't, except for libcore.a.
$(OUTPUT)/libduino.a: $(ALIBOBJ)
	$(AR) rcsv $@ $(ALIBOBJ)

$(OUTPUT)/libduinoall.a: CINCS += $(ALIBALLDIRS:%=-I%)
$(OUTPUT)/libduinoall.a: $(ALIBALLOBJ)
	$(AR) rcsv $@ $(ALIBALLOBJ)

$(OUTPUT)/libuser.a: $(ULIBOBJ)
	$(AR) rcsv $@ $(ULIBOBJ)

$(OUTPUT)/libapp.a: $(OBJ)
	$(AR) rcsv $@ $(OBJ)

$(OUTPUT)/libapp2.a: $(OBJ)
	$(AR) rcsv $@ $(filter-out $(OUTPUT)/$(PROJECT).o,$(OBJ))

$(OUTPUT)/liball.a: $(ULIBOBJ) $(ALIBOBJ) $(COREOBJ)
	$(AR) rcsv $@ $(ULIBOBJ) $(ALIBOBJ) $(COREOBJ)

# Link program from objects and libraries.
$(OUTPUT)/$(PROJECT).elf: $(ALLOBJ)
	$(CXX)  $(LDLIBS) -L$(OUTPUT) -o $@ $(OBJ) $(COREOBJ) \
		$(ULIBOBJ) \
		$(ALIBOBJ) \
		-Wl,--gc-sections -s \
		-mmcu=$(MCU)
#-o bin/Release/aDCLoad.elf .objs/aDCLoad.o .objs/cores/CDC.o .objs/cores/HardwareSerial.o .objs/cores/HID.o .objs/cores/IPAddress.o .objs/cores/main.o .objs/cores/new.o .objs/cores/Print.o .objs/cores/Stream.o .objs/cores/Tone.o .objs/cores/USBCore.o .objs/cores/WInterrupts.o .objs/cores/wiring.o .objs/cores/wiring_analog.o .objs/cores/wiring_digital.o .objs/cores/wiring_pulse.o .objs/cores/wiring_shift.o .objs/cores/WMath.o .objs/cores/WString.o .objs/libraries/libraries.o .objs/sketch.o  -Wl,--gc-sections -s -mmcu=atmega2560
#	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
#		$(OBJ) \
#		$(ULIBOBJ) \
#		$(ALIBOBJ) \
#		-L$(OUTPUT) -lcore $(LDLIBS)

# Alternative linking. Experimental, goes with the additional .a libraries.
# Don't make this dependent on $(OUTPUT), or circular re-makes occur.
# _5.elf fails linking with unresolved setup(), loop().
$(OUTPUT)/$(PROJECT)_2.elf: $(ALLOBJ)
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(OBJ) $(ULIBOBJ) $(ALIBOBJ) $(COREOBJ) \
		$(LDLIBS)
$(OUTPUT)/$(PROJECT)_3.elf: $(ALLOBJ)
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(COREOBJ) $(ALIBOBJ) $(ULIBOBJ) $(OBJ) \
		$(LDLIBS)
$(OUTPUT)/$(PROJECT)_4.elf: $(ALLOBJ) $(OUTPUT)/libcore.a \
				$(OUTPUT)/libduino.a $(OUTPUT)/libuser.a
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(OBJ) \
		-L$(OUTPUT) -luser -lduino -lcore $(LDLIBS)
$(OUTPUT)/$(PROJECT)_5.elf: $(ALLOBJ) $(OUTPUT)/libcore.a \
		   $(OUTPUT)/libduino.a $(OUTPUT)/libuser.a $(OUTPUT)/libapp.a
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		-L$(OUTPUT) -lapp -luser -lduino -lcore $(LDLIBS)
$(OUTPUT)/$(PROJECT)_6.elf: $(ALLOBJ) $(OUTPUT)/libcore.a \
		   $(OUTPUT)/libduino.a $(OUTPUT)/libuser.a $(OUTPUT)/libapp2.a
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(OUTPUT)/$(PROJECT).o \
		-L$(OUTPUT) -lapp2 -luser -lduino -lcore $(LDLIBS)
# Try compiling in one big step, to ensure LTO works.
# Doesn't link - collect2 says Wire.cpp has undef refs to functions in twi.c.
# Changing order of sources doesn't fix that.
$(OUTPUT)/$(PROJECT)_7.elf: $(ALLOBJ) $(OUTPUT)/libcore.a \
		   $(OUTPUT)/libduino.a $(OUTPUT)/libuser.a $(OUTPUT)/libapp.a
	$(CXX) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(SRC) $(CXXSRC) \
		$(ULIBSRC) $(ULIBCXXSRC) \
		$(ALIBSRC) $(ALIBCXXSRC) \
		$(CORESRC) $(CORECXXSRC) \
		$(filter-out -g2 -gstabs -std=gnu++0x -pedantic \
			-Wextra,$(CXXFLAGS)) -fwhole-program -v \
		$(CINCS) \
		$(LDLIBS)
$(OUTPUT)/$(PROJECT)_8.elf: $(ALLOBJ) $(OUTPUT)/libcore.a \
				$(OUTPUT)/libduinoall.a
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(OBJ) \
		$(ULIBOBJ) \
		-L$(OUTPUT) -lduinoall -lcore $(LDLIBS)
$(OUTPUT)/$(PROJECT)_9.elf: $(ALLOBJ) $(OUTPUT)/liball.a
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(OBJ) \
		-L$(OUTPUT) -lall $(LDLIBS)
$(OUTPUT)/$(PROJECT)_A.elf: $(ALLOBJ) $(OUTPUT)/libcore.a \
				$(OUTPUT)/libduino.a
	$(CC) $(LDFLAGS) -Wl,-Map,$*.map,--cref -o $@ \
		$(OBJ) $(ULIBOBJ) \
		-L$(OUTPUT) -lduino -lcore $(LDLIBS)

# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
# UNTESTED
COFFCONVERT=$(OBJCOPY) --debugging \
    --change-section-address .data-0x800000 \
    --change-section-address .bss-0x800000 \
    --change-section-address .noinit-0x800000 \
    --change-section-address .eeprom-0x810000
coff: $(OUTPUT)/$(PROJECT).elf
	$(COFFCONVERT) -O coff-avr $(OUTPUT)/$(PROJECT).elf $(PROJECT).cof
extcoff: $(OUTPUT)/$(PROJECT).elf
	$(COFFCONVERT) -O coff-ext-avr $(OUTPUT)/$(PROJECT).elf $(PROJECT).cof

# Display size of file.
# (Actually, sizes of all $(PROJECT) .elf and .hex in $(OUTPUT).)
size:
	@echo; #echo
	-$(SIZE) $(OUTPUT)/$(PROJECT)*.elf
	@echo
	-$(SIZE) --target=$(HEXFORMAT) $(OUTPUT)/$(PROJECT)*.hex
	@#echo

# Reset the Arduino board before uploading a new program.
# The Arduino is reset on a rising edge of DTR; to make it always happen,
# make sure to set the output low before setting it high.
# Alternatively perl and python programs can be used (stty is faster).
reset: reset_$(RESETCMD)
reset_stty:
	$(STTY) -hupcl; sleep 0.1
	$(STTY) hupcl; sleep 0.1
	$(STTY) -hupcl

# Reset the Arduino board: Perl version needs libdevice-serialport-perl.
# zypper -vv in perl-Device-SerialPort
reset_perl:
	perl -MDevice::SerialPort -e \
	  'Device::SerialPort->new("$(PORT)")->pulse_dtr_off(100)'

# Reset the Arduino board: Python version needs python-serial.
# zypper -vv in python-serial
reset_python:
	python -c "\
	import serial; import time; \
	p = serial.Serial('$(PORT)', 57600); \
	p.setDTR(False); \
	time.sleep(0.1); \
	p.setDTR(True)"

# Show the current state of the DTR line.
dtr:
	$(STTY) -a | tr ' ' '\n' | grep hupcl

# Burn the fuses
fuses:
	#$(AVRDUDE) -F -p $(MCU) -C /etc/avrdude.conf -v -e -V -c usbasp -P usb -U lfuse:w:0xFF:m -U hfuse:w:0xD1:m -U efuse:w:0xCB:m -F lfuse:w:0xE1:m
	#$(AVRDUDE) -p $(MCU) -C /etc/avrdude.conf -P usb -c usbasp -b 9600 -nv
	echo "Fix me !"

# Program the Arduino, without bootloader
burn:
	#$(AVRDUDE) -F -p $(MCU) -C /etc/avrdude.conf -V -c usbasp -P usb $(AVRDUDE_WRITE_FLASH) -U lock:w:0xe8:m
	echo "Write me !"

# Program the Arduino board (upload program).
upload up: hex reset
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

# Create tar file.
# TODO: Dependencies on the header files are missing.
TAREXCL=	$(OUTPUT) back build debug
$(TARFILE).bz2: $(SRC) $(CXXSRC)
	PRJBASE=$$(basename "$$PWD"); \
	cd ..; \
	tar -cvf "$(TARFILE)$(suffix $@)" --bzip2 \
	  $(patsubst %,--exclude "%", $(TAREXCL)) \
	  $(patsubst %,--exclude "$(TARFILE)%", $(suffix $@) .??? .??) \
	  --owner=root --group=root "$$PRJBASE" \
	&& mv "$(TARFILE)$(suffix $@)" "$$OLDPWD" \
	&& echo "" && echo "Created $(TARFILE)$(suffix $@)"
$(TARFILE).xz: $(SRC) $(CXXSRC)
	PRJBASE=$$(basename "$$PWD"); \
	cd ..; \
	tar -cvf "$(TARFILE)$(suffix $@)" --xz \
	  $(patsubst %,--exclude "%", $(TAREXCL)) \
	  $(patsubst %,--exclude "$(TARFILE)%", $(suffix $@) .??? .??) \
	  --owner=root --group=root "$$PRJBASE" \
	&& mv "$(TARFILE)$(suffix $@)" "$$OLDPWD" \
	&& echo "" && echo "Created $(TARFILE)$(suffix $@)"

# Single dependencies file for all sources.
# This doesn't really work, so don't use it.
depend: $(OUTPUT) $(CXXSRCINO)
	$(CC) -M -mmcu=$(MCU) $(CDEFS) \
	    $(CINCS) \
	    $(CORESRC) \
	    $(ALIBSRC) \
	    $(ULIBSRC) \
	    $(SRC) $(ASRC) \
	    > $(DEPFILE)
	$(CXX) -M -mmcu=$(MCU) $(CDEFS) \
	    $(CINCS) \
	    $(CORECXXSRC) \
	    $(ALIBCXXSRC) \
	    $(ULIBCXXSRC) \
	    $(CXXSRC) \
	    >> $(DEPFILE)

# Target: clean project.
CLEANEXT = .elf .hex .eep .cof .lss .sym .listing .symbol .map .log
CLEANPRG = $(foreach p,_2 _3 _4 _5 _6 _7 _8 _9 _A,$(patsubst %,$p%,$(CLEANEXT)))
clean:
	-$(RM) \
	  $(DEPFILE) \
	  $(OUTPUT)/$(PROJECT).cpp \
	  $(CLEANEXT:%=$(OUTPUT)/$(PROJECT)%) \
	  $(CLEANPRG:%=$(OUTPUT)/$(PROJECT)%) \
	  $(patsubst %,$(OUTPUT)/lib%.a,core duino user app app2 duinoall all) \
	  $(ALLOBJ) \
	  $(ALLDEPS) \
	  $(ALLOBJ:%.o=%.S) \
	  $(ALIBALLOBJ) $(ALIBALLOBJ:.o=.d) \
	  $(ALLOBJ:%.o=%.s) \
	  $(ALLOBJ:%.o=%.i) \
	  $(ALLOBJ:%.o=%.ii) \
	  $(notdir $(ALLOBJ:%.o=%.s) $(ALLOBJ:%.o=%.i) $(ALLOBJ:%.o=%.ii))
	-test ! -d $(OUTPUT) || $(RMDIR) $(OUTPUT)

distclean:
	$(RM) -r .objs/ bin/ *.save *~

### Dependencies file and source path.

# This must be after the first explicit rule.

-include $(DEPFILE)

-include $(ALLDEPS)
