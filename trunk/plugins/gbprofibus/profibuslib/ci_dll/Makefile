
CDEFS	= -DDONT_USE_MS_PACK -DUSED_IN_FIRMWARE -DOUTPUT_DESIRED -DDUMP_DESIRED

OBJS	= ci_dll.o linux.o
HDRS	= linux.h
LIB	= ../lib/libci_dll.a

#
# C compiler and linker
#
#CC	=
OPTS	= -O2
CFLAGS	= -Wall ${OPTS} ${CDEFS} -I../inc_glb -fPIC

#
# Build rules
#
${LIB}:	${LIB}(${OBJS})

linux.o: linux.c
	${CC} ${CFLAGS} linux.c -o linux.o -Wl,-i -nostdlib  -lz
#	${CC} ${CFLAGS} linux.c -o linux.o -Wl,-i -nostdlib -lpci -B static -lz

${OBJS}: ${HDRS} Makefile
