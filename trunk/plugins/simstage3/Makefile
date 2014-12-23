######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 534 2014-08-13 12:16:43Z sh $
######################################################################



########## Configuration  ##########
CC = g++
#CFLAGS := -O2 -Wall -fPIC
CFLAGS := -O0 -g -Wall -fPIC
LIBS = -lexpat -lm -lstage
#RHD Headers to include
INC  = -I../../include -I../../src -I./stage-3.2.2.DTU
#Flags to link into shared lib
LDFLAGS := -shared -Lstage-3.2.2.DTU/libstage -lc -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := simStage3.cpp laserServer.cpp

#Plugin output filename
PLUGIN := libsimstage3.so.1

#Create list of objects
OBJ=$(SRC:.cpp=.o)

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
%.o: %.cpp
	$(CC) $(CFLAGS) $(INC) -c $<
