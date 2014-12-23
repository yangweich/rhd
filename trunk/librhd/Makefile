######################################################################
#
# Makefile for Robot Hardware Daemon Lib
#
# $Id: Makefile 1236 2010-12-14 21:39:35Z abb $

######################################################################

CC=gcc
LFLAGS=
CFLAGS =-Wall -O0 -g3 -I../include -fPIC
#Flags to link into shared lib
LDFLAGS := -shared -lc -Wl,-soname,

HDRS   = ../include/rhd.h
OBJS   = librhd.o

#Path to copy lib
BINPATH := ../build/lib

all:	librhd.a(librhd.o)
	@echo -n "   * Compliling librhd.. "
	mkdir -p $(BINPATH)
	cp -f librhd.a $(BINPATH)/librhd.a
	@echo "Done"

dyn:	$(OBJS)
	$(CC) $(LDFLAGS)librhd.so.1 -o librhd.so.1 $(OBJS) $(LIBS) 

clean:
	@echo -n "   * Cleaning librhd .. "
	rm -f ${OBJS} librhd.a
	@echo "Done"

${OBJS}: ${HDRS} Makefile

#Phony targets
.PHONY: dyn