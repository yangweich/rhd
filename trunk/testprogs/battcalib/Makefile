######################################################################
#
# Makefile for Robot Hardware Daemon
#
# $Id: Makefile 52 2008-05-18 15:36:24Z andersbeck $

######################################################################
CC = gcc
CFLAGS := -O2 -Wall -export-dynamic
#CFLAGS := -O0 -g -export-dynamic
LIBS = -lpthread -lncurses -lrhd
INC  = -I../../include -L../../librhd

#Source files
#Core
SRC := battcalib.c gui.c

#Binaries
X86BIN := battcalib

#Path to copy binary
BINPATH := ../../build/bin

X86: $(SRC)
	$(CC) $(CFLAGS) $(INC) $(SRC) -o $(X86BIN) $(LIBS)
#	mkdir -p $(BINPATH)
#	cp -f $(X86BIN) $(BINPATH)/$(X86BIN)

all: X86

clean:
	rm -f $(X86BIN) *~