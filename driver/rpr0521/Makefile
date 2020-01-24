
OBJ := rpr0521

KDIR :=  /lib/modules/$(shell uname -r)/build/
SIGNING :=

obj-m := $(OBJ).o
PWD   := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	$(SIGNING)

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

dtbo:
	$(KDIR)/scripts/dtc/dtc -W no-unit_address_vs_reg -I dts -O dtb -o rpr0521_rpi.dtbo rpr0521_rpi.dts

dtbo_install:
	cp rpr0521_rpi.dtbo /boot/overlays

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

cppcheck:
	cppcheck --enable=all $(OBJ).c $(OBJ).h

sparse:
	$(MAKE) C=2 -C $(KDIR) M=$(PWD) modules
	$(SIGNING)

check: sparse cppcheck
