
CDEFS	= -DDONT_USE_MS_PACK -DUSED_IN_FIRMWARE -DOUTPUT_DESIRED -DDUMP_DESIRED

OBJS	= dps_fkt.o dps_prtab.o dps_syn.o dps_uif.o dps_xtrc.o
# dps_trc.o
LIB	= ../lib/libdps_base.a

#
# C compiler and linker
#
#CC	=
OPTS	= -O2
CFLAGS	= -Wall ${OPTS} ${CDEFS} -I../inc_glb -fPIC
AR	= ar

#
# Build rules
#
${LIB}:	${LIB}(${OBJS})
