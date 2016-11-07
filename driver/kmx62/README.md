# Kionix KMX62 Tri-Axis Magnetometer / Tri-Axis Accelerometer driver for Linux kernel #

[Kionix KMX62 Tri-Axis Magnetometer / Tri-Axis Accelerometer](http://www.kionix.com/product/KMX62)

KMX62 Linux kernel driver uses I2C bus to communicate with sensor. Driver uses input device to report sensor data to user space. One input device is allocated for magnetometer and one for accelerometer.

Qualcomm Dragonboard 410c device running [Android 5.1/Linux Kernel 3.10](https://developer.qualcomm.com/hardware/dragonboard-410c) was used as driver development environment.

By default driver uses Qualcomm sensors_classdev. If you want to disable usage of sensors_classdev please comment out "#define QCOM_SENSORS" from kmx62.c.

# Driver configuration #

**Example** driver configuration is as follows

- kmx62 I2C slave address is 0x0f
- Sensor INT2 pin routed to host gpio 69
- No change to sensor axis mapping
- Accelerometer range +/- 8g
- Data report using interrupt (use-drdy-int)

**Example** device tree configuration

Sensor i2c configuration 

```
kernel/arch/arm/boot/dts/qcom/apq8016-sbc.dtsi

		kmx62@f { /* Kionix accelerometer/magnetometer combo */
			compatible = "kionix,kmx62";
			reg = <0x0f>;
			pinctrl-names = "kmx62_int2_default", "kmx62_int2_sleep";
			pinctrl-0 = <&kmx62_int2_default>;
			pinctrl-1 = <&kmx62_int2_sleep>;
			interrupt-parent = <&msm_gpio>;
			interrupts = <69 0x1>;
			kionix,gpio-int2 = <&msm_gpio 69 0x1>;
			kionix,x-map = <0>;
			kionix,y-map = <1>;
			kionix,z-map = <2>;
			kionix,x-negate = <0>;
			kionix,y-negate = <0>;
			kionix,z-negate = <0>;
			kionix,g-range = <8>;
			kionix,use-drdy-int;
		};
```
Sensor gpio interrupt pincontrol configuration:
```
kernel/arch/arm/boot/dts/qcom/msm8916-pinctrl.dtsi

		kmx62_int2_pin {
			qcom,pins = <&gp 69>;
			qcom,pin-func = <0>;
			qcom,num-grp-pins = <1>;
			label = "kmx62-irq2";
			kmx62_int2_default: kmx62_int2_default {
				drive-strength = <2>;
				bias-disable = <0>;	/* No PULL */
			};
			kmx62_int2_sleep: kmx62_int2_sleep {
				drive-strength = <2>;
				bias-disable = <0>;	/* No PULL */
			};
		};

```
# How to build and install driver module #

Following steps can be used to build driver as loadable kernel module.

1. Update ARCH, CROSS_COMPILER and kernel paths to reference Makefile.
2. Run make to compile driver.
3. Once make completes successfully move kmx62 module to target device.
4. Install driver using insmod. 
5. Verify that driver was properly loaded and sensor found.


**Example** build and install commands

```
kmx62$ make
kmx62$ adb push kmx62.ko /data/
kmx62$ adb shell
root@msm8916_64:/ # insmod /data/kmx62.ko
root@msm8916_64:/ # dmesg | grep kmx62

<6>[  211.002901] kmx62 0-000f: Kionix KMX62 detected
<6>[  211.005319] input: kmx62-accel as /devices/soc.0/78b6000.i2c/i2c-0/0-000f/input/input7
<6>[  211.006874] input: kmx62-mag as /devices/soc.0/78b6000.i2c/i2c-0/0-000f/input/input8

```

**Example** insmod driver on device boot before Android sensor HAL module gets loaded

```
adb push kmx62.ko /data/
adb shell sync
adb reboot
adb wait-for-device
adb shell su -c insmod /data/kmx62.ko
```

# How to add driver to kernel build #

1. Copy driver source files to drivers/input/misc
2. Add driver config to drivers/input/misc/Kconfig
3. Add driver object to drivers/input/misc/Makefile
4. Select Kionix KMX62 driver from Kernel Configuration "Device Drivers -> Input device support -> Miscellaneous devices" 
5. Build kernel image.

**Example** kmx62 drivers/input/misc/Kconfig
```
config SENSORS_KMX62
	tristate "Kionix KMX62 tri-axis accelerometer"
	depends on I2C
	help
	  Say Y here to enable support for the Tri-Axis Magnetometer / 
	  Tri-Axis Accelerometer

	  To compile this driver as a module, choose M here: the module will
	  be called kmx62.
	  
```

**Example** kmx62 drivers/input/misc/Makefile

```
obj-$(CONFIG_SENSORS_KMX62)		+= kmx62.o
```

# Driver usage #

## driver sysfs attributes ##

Accel input dev sysfs interface - /sys/class/input/input7
Mag input dev sysfs interface- /sys/class/input/input8

Qualcomm sensors sysfs interface - /sys/class/sensors/kmx62-accel
Qualcomm sensors sysfs interface - /sys/class/sensors/kmx62-mag

## Input device events ##

**Accelerometer input device events**
```
ABS_X 
ABS_Y
ABS_Z
```

**Magnetometer input device events**
```
ABS_X 
ABS_Y
ABS_Z
```