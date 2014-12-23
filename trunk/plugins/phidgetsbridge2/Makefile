######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 544 2014-08-27 16:50:14Z jcan $
######################################################################



########## Configuration  ##########
CC = gcc
#CFLAGS := -O2 -Wall -fPIC
CFLAGS := -O0 -Wall -fPIC -g
LIBS = -lpthread -lexpat -lm -lphidget21
#RHD Headers to include
INC  = -I../../include -I../../src
#Flags to link into shared lib
LDFLAGS := -shared -lc  -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := phidgetsbridge2.c

#Plugin filename
PLUGIN := phidgetsbridge2.so.1

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
