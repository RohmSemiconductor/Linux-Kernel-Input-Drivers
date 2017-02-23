# Kionix KXG03 Tri-axis Gyroscope/Tri-Axis Accelerometer driver for Linux kernel #

[Kionix KXG03 Tri-axis Gyroscope/Tri-Axis Accelerometer](http://www.kionix.com/product/KXG03)

KXG03 Linux kernel driver uses I2C bus to communicate with sensor. Driver uses input device to report sensor data to user space. One input device is allocated for gyroscope and one for accelerometer.

Qualcomm Dragonboard 410c device running [Android 6.0/Linux Kernel 3.10](https://developer.qualcomm.com/hardware/dragonboard-410c) was used as driver development environment.

By default driver uses Qualcomm sensors_classdev. If you want to disable usage of sensors_classdev please comment out QCOM_SENSORS from kxg03.c.

# Driver configuration #

Driver can be configured using platform data or device tree. 

**Example** driver configuration is as follows. More information can be found from kxg03 header file.

- swap x and y axis
- negate all axis
- default poll rate 100ms
- accelerometer range +/- 8g
- gyroscope range +/- 1024 deg/sec
- sensor INT1 pin routed to host gpio 69
- enable data reporting interrupt

**Example** platform data configuration

```
#define KXG03_I2C_ADDR 0x4f

static struct kxg03_platform_data kxg03_init_data = {
	.init = NULL,
	.release = NULL,

	.x_map = 1,
	.y_map = 0,
	.z_map = 2,
	.x_negate = 1,
	.y_negate = 1,
	.z_negate = 1,
	.poll_rate_ms = 100,
	.accel_range = KXG03_ACC_RANGE_8G,
	.gyro_range = KXG03_GYRO_RANGE_1024,
	.gpio_int1 = 69,
	.gpio_int2 = -1,
	.use_drdy_int = 1,
};

static struct i2c_board_info kxg03_sensor_i2c_info = {
	I2C_BOARD_INFO(KXG03_DEV_NAME, KXG03_I2C_ADDR),
	.platform_data	= &kxg03_init_data,
};
```

**Example** device tree configuration

Sensor i2c configuration 

```
kernel/arch/arm/boot/dts/qcom/apq8016-sbc.dtsi

kxg03@4f { /* Kionix accelerometer/gyroscope combo */
	compatible = "kionix,kxg03";
	reg = <0x4f>;
	pinctrl-names = "kxg03_default", "kxg03_sleep";
	pinctrl-0 = <&kxg03_default>;
	pinctrl-1 = <&kxg03_sleep>;
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
	kionix,gyro-range = <1024>;
	kionix,use-drdy-int;
};
```
Sensor gpio interrupt pincontrol configuration:
```
kernel/arch/arm/boot/dts/qcom/msm8916-pinctrl.dtsi

kxg03_int_pin {
	qcom,pins = <&gp 69>;
	qcom,pin-func = <0>;
	qcom,num-grp-pins = <1>;
	label = "kxg03-irq1";
	kxg03_default: kxg03_default {
		drive-strength = <2>;
		bias-disable = <0>;	/* No PULL */
	};
	kxg03_sleep: kxg03_sleep {
		drive-strength = <2>;
		bias-disable = <0>;	/* No PULL */
	};
};

```
# How to build and install driver module #

Following steps can be used to build driver as loadable kernel module.

1. Update ARCH, CROSS_COMPILER and kernel paths to reference Makefile.
2. Run make to compile driver.
3. Once make completes successfully move kxg03 module to target device.
4. Install driver using insmod. 
5. Verify that driver was properly loaded and sensor found.


**Example** build and install commands

```
kxg03$ make
kxg03$ adb push kxg03.ko /data/
kxg03$ adb shell
root@msm8916_64:/ # insmod /data/kxg03.ko
root@msm8916_64:/ # dmesg | grep kxg03

<6>[  101.301475] input: kxg03-accel as /devices/soc.0/78b6000.i2c/i2c-0/0-004f/input/input6
<6>[  101.303244] input: kxg03-gyro as /devices/soc.0/78b6000.i2c/i2c-0/0-004f/input/input7
```

**Example** insmod driver on device boot before Android sensor HAL module gets loaded

```
adb push kxg03.ko /data/
adb shell sync
adb reboot
adb wait-for-device
adb shell su -c insmod /data/kxg03.ko
```

# How to add driver to kernel build #

1. Copy driver source files to drivers/input/misc
2. Add driver config to drivers/input/misc/Kconfig
3. Add driver object to drivers/input/misc/Makefile
4. Select Kionix KXG03 driver from Kernel Configuration "Device Drivers -> Input device support -> Miscellaneous devices" 
5. Build kernel image.

**Example** kxg03 drivers/input/misc/Kconfig
```
config SENSORS_KXG03
	tristate "Kionix KXG03 tri-axis accelerometer / tri-axis gyroscope"
	depends on I2C
	help
	  Say Y here to enable support for the KXG03 tri-axis accelerometer
	  / tri-axis gyroscope

	  To compile this driver as a module, choose M here: the module will
	  be called kxg03.
	  
```

**Example** kxg03 drivers/input/misc/Makefile

```
obj-$(CONFIG_SENSORS_KXG03)		+= kxg03.o
```

# Driver usage #

## driver sysfs attributes ##

delay

set/get gyroscope or accelerometer data report interval. Interval is set as milliseconds.

enable

set/get gyroscope or accelerometer data report mode

0 = disable data reporting

1 = enable wake mode data reporting


**Example** set gyroscope report interval to 100ms and enable gyroscope
```
# echo 100 > /sys/class/input/input7/delay
# cat /sys/class/input/input7/delay
# echo 1 > /sys/class/input/input7/enable
# cat /sys/class/input/input7/enable
```

**Example** disable gyroscope and enable accelerometer
```
# echo 0 > /sys/class/input/input7/enable
# echo 1 > /sys/class/input/input6/enable
```

## Input device events ##

**Gyroscope input device events**
```
ABS_RX
ABS_RY
ABS_RZ
SYN_TIME_SEC
SYN_TIME_NSEC
```
**Accelerometer input device events**
```
ABS_X 
ABS_Y
ABS_Z
SYN_TIME_SEC
SYN_TIME_NSEC
```

# Android CTS tests #

KXG03 passes Android 6.0 R14 cts-tradefed tests.
