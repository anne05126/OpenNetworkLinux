###############################################################################
#
# 
#
###############################################################################

LIBRARY := x86_64_extremenetworks_8730_32d
$(LIBRARY)_SUBDIR := $(dir $(lastword $(MAKEFILE_LIST)))
include $(BUILDER)/lib.mk
