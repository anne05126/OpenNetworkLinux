###############################################################################
#
# 
#
###############################################################################

LIBRARY := x86_64_delta_ak7448
$(LIBRARY)_SUBDIR := $(dir $(lastword $(MAKEFILE_LIST)))
include $(BUILDER)/lib.mk