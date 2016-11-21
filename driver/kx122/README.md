# Kionix KX122 Tri-Axis Accelerometer driver for Linux kernel #

[Kionix KX122 Tri-Axis Accelerometer](http://www.kionix.com/product/KX122-1037)

KX122 Linux kernel driver uses I2C bus to communicate with sensor. Driver uses input device to report sensor data to user space. 

Qualcomm Dragonboard 410c device running [Android 5.1/Linux Kernel 3.10](https://developer.qualcomm.com/hardware/dragonboard-410c) was used as driver development environment.

By default driver uses Qualcomm sensors_classdev. If you want to disable usage of sensors_classdev please comment out "#define QCOM_SENSORS" from kx122.c.

# Driver configuration #

**Example** driver configuration is as follows

- kx122 I2C slave adress is 0x1f
- sensor INT1 pin routed to host gpio 69
- swap x and y axis
- negate all axis
- accelerometer range +/- 8g
- data report using interrupt (use-drdy-int)

**Example** device tree configuration

Sensor i2c configuration 

```
kernel/arch/arm/boot/dts/qcom/apq8016-sbc.dtsi

		kx122@1f { /* Kionix accelerometer */
			compatible = "kionix,kx122";
			reg = <0x1f>;
			pinctrl-names = "kx122_int_default", "kx122_int_sleep";
			pinctrl-0 = <&kx122_int_default>;
			pinctrl-1 = <&kx122_int_sleep>;
			interrupt-parent = <&msm_gpio>;
			interrupts = <69 0x1>;
			kionix,gpio-int1 = <&msm_gpio 69 0x1>;
			kionix,x-map = <1>;
			kionix,y-map = <0>;
			kionix,z-map = <2>;
			kionix,x-negate = <1>;
			kionix,y-negate = <1>;
			kionix,z-negate = <1>;
			kionix,g-range = <8>;
			kionix,use-drdy-int;
		};
```
Sensor gpio interrupt pincontrol configuration:
```
kernel/arch/arm/boot/dts/qcom/msm8916-pinctrl.dtsi

		kx122_int_pin {
			qcom,pins = <&gp 69>;
			qcom,pin-func = <0>;
			qcom,num-grp-pins = <1>;
			label = "kx122-int";
			kx122_int_default: kx122_int_default {
				drive-strength = <2>;
				bias-disable = <0>;	/* No PULL */
			};
			kx122_int_sleep: kx122_int_sleep {
				drive-strength = <2>;
				bias-disable = <0>;	/* No PULL */
			};
		};

```
# How to build and install driver module #

Following steps can be used to build driver as loadable kernel module.

1. Update ARCH, CROSS_COMPILER and kernel paths to reference Makefile.
2. Run make to compile driver.
3. Once make completes successfully move kx122 module to target device.
4. Install driver using insmod. 
5. Verify that driver was properly loaded and sensor found.


**Example** build and install commands

```
kx122$ make
kx122$ adb push kx122.ko /data/
kx122$ adb shell
root@msm8916_64:/ # insmod /data/kx122.ko
root@msm8916_64:/ # dmesg | grep kx122

<6>[ 5611.900505] kx122 0-001f: Kionix kx122 detected
<6>[ 5611.907933] input: kx122-accel as /devices/soc.0/78b6000.i2c/i2c-0/0-001f/input/input8
```

**Example** insmod driver on device boot before Android sensor HAL module gets loaded

```
adb push kx122.ko /data/
adb shell sync
adb reboot
adb wait-for-device
adb shell su -c insmod /data/kx122.ko
```

# How to add driver to kernel build #

1. Copy driver source files to drivers/input/misc
2. Add driver config to drivers/input/misc/Kconfig
3. Add driver object to drivers/input/misc/Makefile
4. Select Kionix KX122 driver from Kernel Configuration "Device Drivers -> Input device support -> Miscellaneous devices" 
5. Build kernel image.

**Example** kx122 drivers/input/misc/Kconfig
```
config SENSORS_KX122
	tristate "Kionix KX122 tri-axis accelerometer"
	depends on I2C
	help
	  Say Y here to enable support for the KX122 tri-axis accelerometer

	  To compile this driver as a module, choose M here: the module will
	  be called kx122.
	  
```

**Example** kx122 drivers/input/misc/Makefile

```
obj-$(CONFIG_SENSORS_KX122)		+= kx122.o
```

# Driver usage #

## driver sysfs attributes ##

Input dev sysfs interface - /sys/class/input/input8

Qualcomm sensors sysfs interface - /sys/class/sensors/kx122


## Input device events ##


**Accelerometer input device events**
```
ABS_X 
ABS_Y
ABS_Z
SYN_TIME_SEC
SYN_TIME_NSEC
```
# Android CTS tests #

KX122 passes Android 5.1 R13 cts-tradefed tests.
