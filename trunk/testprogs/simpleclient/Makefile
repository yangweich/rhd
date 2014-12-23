#
# The Compiler
#
CC      = gcc
LD      = ${CC}
CFLAGS  = -Wall -O0 -g3 -I../../include
LDFLAGS = -L../../librhd

#
# Our program files
#
PROG   = simpleclient
HDRS   =
OBJS   = testprog.o
LIBS   = -lrhd

#Path to copy binary
BINPATH := ../../build/bin

all:	${PROG}

${PROG}: ${OBJS}
	${LD} -o ${@} ${LDFLAGS} ${OBJS} ${LIBS}
	mkdir -p $(BINPATH)
	cp -f $(PROG) $(BINPATH)/$(PROG)

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
