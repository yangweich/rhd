######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
######################################################################



########## Configuration  ##########
CC = gcc
CFLAGS := -O2 -Wall -fPIC
#LIBS = -lexpat -lm -L/misc/vhome/lvm/src/psg/stage211/lib -lstage
LIBS = -lexpat -lm -lstage
#RHD Headers to include

INC  = -I../../include -I../../src -I/usr/local/include/stage-2.1 `pkg-config --cflags glib-2.0`
#Use at AUT if above does not work
#INC  = -I../../include -I../../src -I/misc/vhome/lvm/src/psg/stage211/include/stage-2.1 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include
#Deprecated, path to glib has changed in new versions of ubuntu
#INC  = -I../../include -I../../src -I/usr/local/include/stage-2.1 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include

#Flags to link into shared lib
LDFLAGS := -shared -lc -Wall,-no-undefined,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := simStage211.c

#Plugin output filename
PLUGIN := libsimstage211.so.1

#Create list of objects
OBJ=$(SRC:.c=.o)

#default rules
all : plugin

#Make the dynamic plugin library
plugin: $(OBJ)
	$(CC) $(LDFLAGS)$(PLUGIN) -o $(PLUGIN) $(OBJ) $(LIBS) 
	mkdir -p $(PLUGINSPATH)
	cp -f $(PLUGIN) $(PLUGINSPATH)/$(PLUGIN)

# Clean plugin
clean:
	rm -f *~ *.o *.d $(PLUGIN)


###### .c to .o #####
%.o: %.c
	$(CC) $(CFLAGS) $(INC) -c $<
