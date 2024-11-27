PWD         := $(shell pwd)
# KVERSION    := $(shell uname -r)
KVERSION    :=6.6.51+rpt-rpi-v8
KERNEL_HEAD := /root/rpi/kernel8/root
KERNEL_DIR   = $(KERNEL_HEAD)/lib/modules/$(KVERSION)/build

MODULE_NAME  = panel-dmb-kd029qhfid001
obj-m       := $(MODULE_NAME).o

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
