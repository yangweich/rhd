######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 96 2013-01-19 16:09:28Z jcan $
######################################################################



########## Configuration  ##########
CC = gcc
#CFLAGS := -O2 -Wall -fPIC
CFLAGS := -O0 -Wall -fPIC -g
LIBS = -lpthread -lexpat
#RHD Headers to include
INC  = -I../../include -I../../src
#Flags to link into shared lib
LDFLAGS := -shared -lc -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := kopterctrl.c

#Plugin filename
PLUGIN := kopterctrl.so.1

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
