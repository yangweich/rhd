######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 109 2013-01-23 20:44:29Z sh $
######################################################################


########## Configuration  ##########
CC = gcc
CFLAGS := -O0 -Wall -fPIC
#CFLAGS := -O0 -Wall -fPIC -g
LIBS = -lpthread -lexpat -lm -L profibuslib/lib -ldp_base -ldps_base -lci_dll -lpci
#RHD Headers to include
INC  = -I../../include -I../../src
#Flags to link into shared lib
LDFLAGS := -shared -lc -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin
BINPATH := ../../build/bin


########## Source files ##########
#Plugin-source files
SRC := gbprofibus.c profibus.c

#Plugin filename
PLUGIN := libgbprofibus.so.1

#Create list of objects
OBJ=$(SRC:.c=.o)

#default rules
all : libs plugin

libs:
	@$(MAKE) -s -C profibuslib/ci_dll
	@$(MAKE) -s -C profibuslib/dp_base
	@$(MAKE) -s -C profibuslib/dps_base

#Make the dynamic plugin library
plugin: $(OBJ)
	@$(MAKE) -s -C profibuslib/ci_dll
	@$(MAKE) -s -C profibuslib/dp_base
	@$(MAKE) -s -C profibuslib/dps_base
	$(CC) $(LDFLAGS)$(PLUGIN) -o $(PLUGIN) $(OBJ) $(LIBS) 
	mkdir -p $(PLUGINSPATH)
	cp -f fw_5613.bin $(BINPATH)
	cp -f guidebot.ldb $(BINPATH)
	cp -f $(PLUGIN) $(PLUGINSPATH)/$(PLUGIN)


#
# Hide all the profibus gunk here.
# (Special header files, defines and libraries)
#profibus.o: profibus.c
#	${CC} ${CFLAGS} -I profibuslib/inc_glb \
#		-DDONT_USE_MS_PACK -DUSED_IN_FIRMWARE \
#		-o $@ $< \
#		-Xlinker -r -nostdlib \
#		-L profibuslib/lib -ldp_base -ldps_base -lci_dll

# Clean plugin
clean:
	rm -f *~ *.o *.d $(PLUGIN) profibuslib/lib/*.a profibuslib/*/*.o


###### .c to .o #####
#%.o: %.c
#	$(CC) $(CFLAGS) $(INC) -c $<

gbprofibus.o: gbprofibus.c
	$(CC) $(CFLAGS) $(INC) -c $<

profibus.o: profibus.c libs
	$(CC) $(CFLAGS) -Iprofibuslib/inc_glb -DDONT_USE_MS_PACK -DUSED_IN_FIRMWARE -c $< -Lprofibuslib/lib -lci_dll -ldp_base -ldps_base -lpci
