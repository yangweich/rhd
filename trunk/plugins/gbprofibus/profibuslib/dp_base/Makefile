
CDEFS	= -DDONT_USE_MS_PACK -DUSED_IN_FIRMWARE -DOUTPUT_DESIRED -DDUMP_DESIRED

OBJS	= dp_auif.o dp_cvrt.o dp_fkt.o dp_prtab.o dp_syn.o dp_uif.o
# dp_trc.o
LIB	= ../lib/libdp_base.a

#
# C compiler and linker
#
#CC	=
OPTS	= -O2
CFLAGS	= -Wall ${OPTS} ${CDEFS} -I../inc_glb -I../dps_base -fPIC
AR	= ar

#
# Build rules
#
${LIB}:	${LIB}(${OBJS})
