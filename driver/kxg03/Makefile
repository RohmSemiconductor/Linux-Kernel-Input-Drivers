export CROSS_COMPILE:=aarch64-linux-android-
export ARCH:=arm64
export CFLAGS_MODULE:=-fno-pic

OBJ := kxg03
KDIR_SRC :=  ../../../kernel
KDIR  := ../../../out/target/product/msm8916_64/obj/KERNEL_OBJ

SIGNING := perl $(KDIR_SRC)/scripts/sign-file sha512 $(KDIR)/signing_key.priv $(KDIR)/signing_key.x509 $(OBJ).ko

obj-m := $(OBJ).o
PWD   := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	$(SIGNING)

cppcheck:
	cppcheck --enable=all $(OBJ).c $(OBJ).h

sparse:
	$(MAKE) -C $(KDIR) M=$(PWD) modules C=2

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
