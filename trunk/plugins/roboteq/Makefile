######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
######################################################################

########## Configuration  ##########
CC = gcc
CFLAGS := -O0 -g3 -Wall -fPIC
LIBS = -lpthread -lexpat -lm
#RHD Headers to include
INC  = -I../../include -I../../src
#Flags to link into shared lib
LDFLAGS := -shared -lc -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := roboteq.c

#Plugin filename
PLUGIN := librtq.so.1

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
