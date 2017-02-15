PLATFORM_UTGARD = mt2601 mt6572 mt6580 mt6582 mt6592
PLATFORM_MIDGARD = mt6735 mt6752

PLATFORM_NAME = $(subst ",,$(CONFIG_MTK_PLATFORM))

ifneq (, $(findstring $(PLATFORM_NAME), $(PLATFORM_UTGARD)))
    $(info *GPU in $(PLATFORM_NAME) is: UTGARD-GPU)
    obj-y += mali_utgard/
else
    $(info *GPU in $(PLATFORM_NAME) is: MIDGARD-GPU)
    obj-y += mali_midgard/
endif