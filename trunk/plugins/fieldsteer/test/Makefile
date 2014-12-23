######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 277 2013-10-20 07:33:22Z jcan $
######################################################################



########## Configuration  ##########
CC = gcc
#CFLAGS := -O2 -Wall -fPIC
CFLAGS := -O0 -Wall -fPIC -g
LIBS = -lpthread -lexpat -lm
#RHD Headers to include
INC  = -I../../../include -I../../../src
#Flags to link into shared lib
#LDFLAGS := -shared -lc -Wl,-soname,

#Plugins path
#PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := teststeer.c

#Plugin filename
PLUGIN := teststeer

#Create list of objects
OBJ=$(SRC:.c=.o)

#default rules
all : plugin

#Make the dynamic plugin library
plugin: $(OBJ)
	$(CC) -o $(PLUGIN) $(OBJ) $(LIBS)

# Clean plugin
clean:
	rm -f *~ *.o *.d $(PLUGIN)


###### .c to .o #####
%.o: %.c
	$(CC) $(CFLAGS) $(INC) -c $<
