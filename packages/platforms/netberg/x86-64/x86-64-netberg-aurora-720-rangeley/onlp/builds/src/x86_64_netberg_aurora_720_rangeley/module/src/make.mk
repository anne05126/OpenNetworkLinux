###############################################################################
#
# 
#
###############################################################################

LIBRARY := x86_64_netberg_aurora_720_rangeley
$(LIBRARY)_SUBDIR := $(dir $(lastword $(MAKEFILE_LIST)))
include $(BUILDER)/lib.mk