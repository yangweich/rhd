######################################################################
#
# Top level Makefile for RHD Testprograms
#
# $Id: Makefile 90 2008-09-26 13:46:08Z s021786 $
######################################################################

# Compile all plugins

#List testprogram folder names here
PLUGINLIST := rhdclient simpleclient battcalib

all: title $(PLUGINLIST) 

#Execute make in all plugin folders
$(PLUGINLIST): 
	@echo -n "   * Compliling $@.. "
	@$(MAKE) -s -C $@
	@echo "Done"

#Print complie title
title:
	@echo "=== Compiling RHD Testprograms ==="

#Clean all plugins
clean:
	@echo "=== Cleaning RHD Testprograms ==="
	@for plugin in $(PLUGINLIST); do\
		echo -n "   * Cleaning $$plugin.. ";\
		$(MAKE) -s -C $$plugin clean;\
		echo "Done";\
	done

#Phony targets
.PHONY: title $(PLUGINLIST) clean