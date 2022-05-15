ifneq ($(filter PD1941 PD1941F_EX PD1941BF_EX PD1963 STD2009 PD2012,$(BBK_PRODUCT_MODEL)),) ## 平台供电
$(info "******************************** $(BBK_PRODUCT_MODEL)  use platform power supply ********************************")
ccflags-y += -DUSE_PLAT_POWER_SUPPLY
endif

gf3658-objs := gf3658_spi.o gf3658_platform.o gf3658_netlink.o
obj-$(CONFIG_BBK_GF3658) := gf3658.o
