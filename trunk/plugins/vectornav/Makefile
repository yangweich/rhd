######################################################################
#
# Makefile for RHD Plugins
#
# $Id$
######################################################################

########## Configuration  ##########
CC = gcc
CFLAGS := -O0 -g3 -Wall -fPIC
LIBS = -lpthread -lexpat -lrt -lm 
#RHD Headers to include
INC  = -I../../include -I../../src 
#VN-100 Headers to include
INC += -I./include
#Flags to link into shared lib
LDFLAGS := -shared -lc -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin

########## Source files ##########
#Plugin-source files
SRC := vn100RHD.c vn100.c vncp_services.c

#Plugin filename
PLUGIN := libvn100.so.1

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
