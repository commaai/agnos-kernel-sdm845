obj-m := GobiNet.o
GobiNet-objs := GobiUSBNet.o QMIDevice.o QMI.o usbnet_2_6_32.o usbnet_3_0_6.o \
	            usbnet_2_6_35.o usbnet_3_10_21.o usbnet_3_12_xx.o usbnet_4_4_xx.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
OUTPUTDIR=/lib/modules/`uname -r`/kernel/drivers/net/usb/

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: all
	mkdir -p $(OUTPUTDIR)
	cp -f GobiNet.ko $(OUTPUTDIR)
	depmod

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.* modules.order .cache.mk


