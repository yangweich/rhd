######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 79 2012-12-15 07:02:57Z jcan $
######################################################################



########## Configuration  ##########
CC = gcc
#CFLAGS := -O2 -Wall -fPIC
CFLAGS := -O0 -Wall -fPIC -g
LIBS = -lpthread -lexpat -lm
#RHD Headers to include
INC  = -I../../include -I../../src
#Flags to link into shared lib
LDFLAGS := -shared -lc -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := hexakopter.c encode.c

#Plugin filename
PLUGIN := hexakopter.so.1

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
