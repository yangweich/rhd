######################################################################
#
# Top level Makefile for RHD Plugins
#
# $Id: Makefile 506 2014-07-09 16:25:04Z immunt $
######################################################################

# Compile all plugins

#List plugin folder names here
PLUGINLIST := auserial gps crossbow fogyro hakocan smrdserial rflex \
              rs232linesensor powercube esm slugs usbiss sf9dof \
              joycontrol gdm8246 buspiratei2c buspiratead \
              herkulex saberandmagenc imu gps2\
              smrarm rhdlog hexakopter roboteq gpsSocket \
              dynamixel phidgetsbridge2 rhdlink tcuav fcs
#Hohenheim    roboteq simstage21 gpsSocket
# PLUGINLIST := auserial gps crossbow fogyro hakocan smrdserial rflex \
#               rs232linesensor powercube esm slugs usbiss sd84 sf9dof \
#               joycontrol gdm8246 buspiratei2c buspiratead \
#               herkulex saberandmagenc \
#               smrarm

all: title $(PLUGINLIST)  

#gbprofibus:
#	@echo "   * Compliling profibus -  if possible (compiles only on robot with profibus or jensen)"
#	@-$(MAKE) -s -C gbprofibus

#Execute make in all plugin folders
$(PLUGINLIST):
	@echo -n "   * Compliling $@.."
	@$(MAKE) -s -C $@
	@echo "Done"

#Print complie title
title:
	@echo "=== Compiling RHD Plugins ==="

#Clean all plugins
clean:
	@echo "=== Cleaning RHD Plugins ==="
	@for plugin in $(PLUGINLIST); do\
		echo -n "   * Cleaning $$plugin.. ";\
		$(MAKE) -s -C $$plugin clean;\
		echo "Done";\
	done

#Phony targets
.PHONY: title gbprofibus $(PLUGINLIST) clean
