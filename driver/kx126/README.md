# Kionix KX126 Tri-Axis Accelerometer driver for Linux kernel #

[Kionix KX126 Tri-Axis Accelerometer](http://www.kionix.com/product/KX126-1063)

KX126 Linux kernel driver uses I2C bus to communicate with sensor. Driver uses input devices to report sensor data to user space. One input device is allocated for sensor xyz-data, one for step counter and one for step detector event. 

Qualcomm Dragonboard 410c device running [Android 6.0/Linux Kernel 3.10](https://developer.qualcomm.com/hardware/dragonboard-410c) was used as driver development and verification environment.


# Driver configuration #

**Example** driver configuration is as follows

- kx126 I2C slave address is 0x1f
- sensor INT1 pin routed to host gpio 69
- swap x and y axis
- negate all axis
- accelerometer range +/- 8g
- data report using interrupt (use-drdy-int)


**Example** device tree configuration
```
kernel/arch/arm/boot/dts/qcom/apq8016-sbc.dtsi

		kx126@1f { /* Kionix accelerometer */
			compatible = "kionix,kx126";
			reg = <0x1f>;
			pinctrl-names = "kx126_int_default", "kx126_int_sleep";
			pinctrl-0 = <&kx126_int_default>;
			pinctrl-1 = <&kx126_int_sleep>;
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

		kx126_int_pin {
			qcom,pins = <&gp 69>;
			qcom,pin-func = <0>;
			qcom,num-grp-pins = <1>;
			label = "kx126-int1";
			kx126_int_default: kx126_int_default {
				drive-strength = <2>;
				bias-disable = <0>;
			};
			kx126_int_sleep: kx126_int_sleep {
				drive-strength = <2>;
				bias-disable = <0>;
			};
		};
```
# How to build and install driver module #

Following steps can be used to build driver as loadable kernel module.

1. Update ARCH, CROSS_COMPILER and kernel paths to reference Makefile.
2. Run make to compile driver.
3. Once make completes successfully move kx126 module to target device.
4. Install driver using insmod. 
5. Verify that driver was properly loaded and sensor found.

**Example** build and install commands
```
kx126$ make
kx126$ adb push kx126.ko /data/
kx126$ adb shell
root@msm8916_64:/ # insmod /data/kx126.ko
root@msm8916_64:/ # dmesg | grep kx126

[ 3638.923422] kx126 0-001f: Kionix kx126 detected
[ 3638.928696] input: kx126-accel as /devices/soc.0/78b6000.i2c/i2c-0/0-001f/input/input6
[ 3638.932570] input: kx126-stepdet as /devices/soc.0/78b6000.i2c/i2c-0/0-001f/input/input7
[ 3638.935198] input: kx126-stepcnt as /devices/soc.0/78b6000.i2c/i2c-0/0-001f/input/input8
```

**Example** insmod driver on device boot before Android sensor HAL module gets loaded
```
adb push kx126.ko /data/
adb shell sync
adb reboot
adb wait-for-device
adb shell su -c insmod /data/kx126.ko
```

# How to add driver to kernel build #

1. Copy driver source files to drivers/input/misc
2. Add driver config to drivers/input/misc/Kconfig
3. Add driver object to drivers/input/misc/Makefile
4. Select Kionix KX126 driver from Kernel Configuration "Device Drivers -> Input device support -> Miscellaneous devices" 
5. Build kernel image.

**Example** kx126 drivers/input/misc/Kconfig
```
config SENSORS_KX126
	tristate "Kionix KX126 tri-axis accelerometer"
	depends on I2C
	help
	  Say Y here to enable support for the KX126 tri-axis accelerometer

	  To compile this driver as a module, choose M here: the module will
	  be called kx126.
	  
```

**Example** kx126 drivers/input/misc/Makefile
```
obj-$(CONFIG_SENSORS_KX126)		+= kx126.o
```

# Driver usage #

## Driver control interface (sysfs attributes) ##

**Input dev sysfs interface**
```
kx126-accel : /sys/class/input/input6
kx126-stepdet : /sys/class/input/input7
kx126-stepcnt : /sys/class/input/input8
```

**Qualcomm sensors sysfs interface**
```
root@msm8916_64:/ # ls /sys/class/sensors/
kx126-accel
kx126-stepcnt
kx126-stepdet
```

## Driver data interface (input device) ##

**Accelerometer xyz input events (kx126-accel)**
```
ABS_X 
ABS_Y
ABS_Z
SYN_TIME_SEC
SYN_TIME_NSEC
```

**Step detector input events (kx126-stepdet)**
```
ABS_MISC (step detect indication)
SYN_TIME_SEC
SYN_TIME_NSEC
```

**Step counter input events (kx126-stepcnt)**
```
ABS_MISC (total step count)
SYN_TIME_SEC
SYN_TIME_NSEC
```

# Android CTS tests #

KX126 passes Android 6.0 R18 cts-tradefed tests.

# How to disable Qualcomm Sensors interface #

By default driver uses Qualcomm Linux kernel sensors interface. If you want to disable usage of Qualcomm sensors comment out QCOM SENSORS define from kx126.c. 

**Example** Disable Qualcomm sensors interface usage

```
kx126.c

	//#define QCOM_SENSORS
```
