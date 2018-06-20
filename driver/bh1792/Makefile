OBJ := bh1792
KERNEL :=  /lib/modules/$(shell uname -r)/build/

obj-m := $(OBJ).o
PWD   := $(shell pwd)

.PHONY: modules checkpatch sparse dtbo install clean

default: modules

modules:
	make -C $(KERNEL) M=$(PWD) modules

checkpatch:
	$(KERNEL)/scripts/checkpatch.pl --no-tree --terse -f $(OBJ).c

cppcheck:
	cppcheck --enable=all $(OBJ).c $(OBJ).h

sparse:
	make -C $(KERNEL) M=$(PWD) modules C=2

dtbo:
	$(KERNEL)/scripts/dtc/dtc -@ -W no-unit_address_vs_reg -I dts -O dtb -o bh1792_rpi.dtbo bh1792_rpi_overlay.dts

dtbo_install:
	cp bh1792_rpi.dtbo /boot/overlays

install:
	make -C $(KERNEL) M=$(PWD) modules_install

clean:
	make -C $(KERNEL) M=$(PWD) clean
	rm -f bh1792_rpi_overlay.dtbo

