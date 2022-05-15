# Switch Class Driver
ifneq ($(filter PD2005F_EX,$(BBK_PRODUCT_MODEL)),)
$(info "****** $(BBK_PRODUCT_MODEL)  need customize *********")
ccflags-y += -DCUSTOMIZE_PD2005F
endif
obj-y    += sarpower_class.o
obj-y    += sar_power.o

