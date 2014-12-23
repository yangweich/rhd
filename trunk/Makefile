######################################################################
#
# Top level Makefile for RHD Project
#
# $Id: Makefile 1921 2012-05-29 07:48:40Z jca $
######################################################################

#Destination directories
BUILDDIR := build
BINDIR := $(BUILDDIR)/bin
LIBDIR := $(BUILDDIR)/lib
INCDIR := $(BUILDDIR)/include
DOCDIR := $(BUILDDIR)/doc
CONFIGDIR := $(BINDIR)/rhdconfig


all: title rhd librhd testprog plugins copy

title: 
	@echo "====  Robot Hardware Daemon project v2.x  ===="

rhd:
	@echo "=== Compiling Robot Hardware Daemon (RHD) ==="
	@$(MAKE) -s -C src X86


librhd:
	@echo "=== Compiling librhd ==="
	@$(MAKE) -s -C librhd

plugins: 
	@$(MAKE) -s -C plugins


testprog: librhd
	@$(MAKE) -s -C testprogs

copy: 
	@echo "=== Copying files to build directory ==="
	@echo -n "   * Include headers files...  "
	@mkdir -p $(INCDIR)
	@cp include/*.h $(INCDIR)
	@echo "Done"
	@echo -n "   * XML Configuration files...  "
	@mkdir -p $(CONFIGDIR)
	@cp configurations/*.xml $(CONFIGDIR)
	@echo "Done"
	
clean:
	@echo "====   Cleaning RHD project   ===="
	@$(MAKE) -s -C src clean
	@$(MAKE) -s -C librhd clean
	@$(MAKE) -s -C testprogs clean
	@$(MAKE) -s -C plugins clean
	@echo "=== Removing binaries ==="
	@echo -n "   * Removing build directory.. "
	@rm -r ./build
	@echo "Done"


#Phony targets
.PHONY: librhd rhd testprog plugins clean title copy