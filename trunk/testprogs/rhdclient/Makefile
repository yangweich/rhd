######################################################################
#
# Makefile for Robot Hardware Daemon
#
# $Id: Makefile 1962 2012-07-26 18:36:19Z jca $

######################################################################
CC = gcc
#CFLAGS := -O2 -Wall -export-dynamic
CFLAGS := -O0 -g3 -Wall -export-dynamic
LDFLAGS = -L../../librhd -lpthread -lncurses -lrhd
CPPFLAGS  = -I../../include 

#Source files
#Core
SRC := rhdclient.c gui.c cmdparser.c
OBJ = rhdclient.o gui.o cmdparser.o

#Binaries
X86BIN := rhdtest

#Path to copy binary
BINPATH := ../../build/bin

X86: $(SRC) $(OBJ)
	$(CC) -o $(X86BIN) $(OBJ) $(LDFLAGS)
	#$(CC) $(CFLAGS) $(INC) $(SRC) -o $(X86BIN) $(LIBS)
	mkdir -p $(BINPATH)
	cp -f $(X86BIN) $(BINPATH)/$(X86BIN)

all: X86

clean:
	rm -f $(X86BIN) *.o