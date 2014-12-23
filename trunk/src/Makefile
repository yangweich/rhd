######################################################################
#
# Makefile for Robot Hardware Daemon
#
# $Id: Makefile 1919 2012-05-28 15:25:57Z jca $

######################################################################
CC = gcc
AVR32PREFIX = avr32-linux-
A32INC = -I/usr/avr32-linux/include
#CFLAGS := -O2 -Wall -export-dynamic
CFLAGS := -O0 -g3 -Wall -export-dynamic
LIBS = -pthread -lexpat -lm -ldl -L/usr/realtime/lib
INC  = -I../include

#Source files
#Core
SRC := main.c database.c globalfunc.c server.c scheduler.c 

#Binaries
X86BIN := rhd
AVR32BIN := avr32-rhd

#Path to copy binary
BINPATH := ../build/bin

RTAI_BIN := $(shell ls -d /bin/rtai-config 2>/dev/null)
RTAI_USR_BIN := $(shell ls -d /usr/bin/rtai-config 2>/dev/null)
ifeq ($(RTAI_BIN),)
  ifeq ($(RTAI_USR_BIN),)
    MSG = "   * Compiling without RTAI support (RTAI package not found)"
  else
  # normal (ubuntu) config
  LXRT_CFLAGS := $(shell rtai-config --lxrt-cflags)
  LXRT_LDFLAGS := `rtai-config --lxrt-ldflags` -llxrt
  endif
else
  #RTAI Configurations - Jensen-Nyquist configuration
  RTAIPATH := /usr/realtime
  LXRT_CFLAGS := $(shell $(RTAIPATH)/bin/rtai-config --lxrt-cflags)
  LXRT_LDFLAGS := $(shell $(RTAIPATH)/bin/rtai-config --lxrt-ldflags) -llxrt
  INC  += -I/usr/realtime/include
endif
ifeq ($(LXRT_CFLAGS),)
else
  # rtai is installed - use it
  DEFINES = -DRTAI=1
  CFLAGS += $(LXRT_CFLAGS)
  LIBS += $(LXRT_LDFLAGS)
  MSG = "   * Compiling with RTAI support (hard real time)"
endif

ALL: $(SRC)
	@echo $(MSG)
	@echo "   * RTAI C-flags '" $(LXRT_CFLAGS) "'"
	$(CC) $(CFLAGS) $(INC) $(SRC) -o $(X86BIN) $(LIBS) $(DEFINES)
	mkdir -p $(BINPATH)
	cp -f $(X86BIN) $(BINPATH)/$(X86BIN)
	@echo "   * Done"

help:
	@echo "make options"
	@echo "  default attempts to determine if RTAI is installed, and use it if so"
	@echo "  other options:"
	@echo "  make X86    (will make RHD without RTAI support)"
	@echo "  make AVR32  (will try to make for AVR32)"
	@echo "  make RTAI   (will try to make for X86 with RTAI support)"
	@echo "  "
	@echo "Has determined flag LXRT_CFLAGS  is to be " $(LXRT_CFLAGS)
	@echo "Has determined flag LXRT_LDFLAGS is to be " $(LXRT_LDFLAGS)


X86: $(SRC)
	@echo "   * compiling X86 RHD without RTAI"
	$(CC) $(CFLAGS) $(INC) $(SRC) -o $(X86BIN) $(LIBS)
	mkdir -p $(BINPATH)
	cp -f $(X86BIN) $(BINPATH)/$(X86BIN)
	@echo "   * Done"

#Target to compile with RTAI support...
RTAI: $(SRC) 
	@echo -n "   * Compliling X86 RHD with RTAI.. "
	$(CC) $(CFLAGS) $(LXRT_CFLAGS) -DRTAI=1 $(INC) $(SRC) -o $(X86BIN) $(LXRT_LDFLAGS) $(LIBS)
	mkdir -p $(BINPATH)
	cp -f $(X86BIN) $(BINPATH)/$(X86BIN)	
	@echo "   * Done"


AVR32: $(SRC)
	@echo -n "   * Compliling RHD for AVR32.. "
	$(AVR32PREFIX)$(CC) $(CFLAGS) $(INC) $(A32INC) $(SRC) -o $(AVR32BIN) $(LIBS)
	mkdir -p $(BINPATH)
	cp -f $(AVR32BIN) $(BINPATH)/$(AVR32BIN)

#all: DEFAULT

clean:
	@echo -n "   * Cleaning RHD ..\t"
	rm -f $(X86BIN) ${AVR32BIN} *~
	@echo "   * Done"
