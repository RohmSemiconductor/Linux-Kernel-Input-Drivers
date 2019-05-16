# ROHM RPR0521 Proximity and Ambient Light Sensor for Linux kernel #

rpr0521 Linux kernel driver uses I2C bus to communicate with sensor. Driver uses input devices to report sensor data to user space.
This module was tested on Raspberry pi 3.

Find the datasheet of the sensor here: http://rohmfs.rohm.com/en/products/databook/datasheet/opto/optical_sensor/opto_module/rpr-0521rs-e.pdf

# Physical connection to target hardware

|rpi pin | sensor pin |
|:------:|------------|
|   1    | VDD        |
|   9    | GND        |
|   3    | SDA        |
|   5    | SCL        |
|   33   | INT        |


# Device-tree configuration #

The driver requires device tree configuration, i2c-address (0x38) and interrupt information (depends on hw setup, here we use gpio 13) are mandatory if using interrupt mode otherwise polling will be used. There are optional parameters for sensor default settings:

- *proximity_delay*: Integer value for delay of proximity measurement time, with allowed values 0 (standby), 10, 40, 50, 100, and 400 ms.
- *als_delay*: Integer value with for delay of ambient light sensor measurement time, with allowed values 0 (standby), 50, 100, 400 ms. This value and the *proximity_delay* are used in a set of defined combinations, so not any random combination is correct (check datasheet).
- *proximity_offset*: Integer value for threshold of proximity interrupt, allowed values are 0-511

**Example** device tree configuration. I2C address is defined to be 0x39 and gpio interrupt 13 with falling edge is used as interrupt. Device raises interrupt when clear channel data exceeds 511 (0x1FF).
```
        rpr0521: rpr0521@38 {
				compatible = "rohm,rpr0521";
				reg = <0x38>;
				pinctrl-name = "default";
				pinctrl-0 = <&rpr0521_pins>;
				interrupt-parent = <&gpio>;
				interrupts = <13 0x2>;
				rohm,use-drdy-int;
				rohm,als-delay = <100>;
				rohm,proximity-delay = <100>;
				rohm,proximity-offset = <0>;
			};

```

# How to build and install driver module #

Kernel headers are needed to succesfully build the module, install them first.
```
sudo apt update
sudo apt upgrade
sudo reboot
sudo apt install raspberrypi-kernel-headers
```

Device-tree overlay for Raspberry pi3 is called rpr0521_rpi.dts. Modify it for your needs, build and install with following commands:
```
make dtbo
sudo make dtbo_install
```
Add following line to /boot/config.txt
```
dtoverlay=rpr0521_rpi
```
The device tree overlay can be loaded dynamically without a reboot also
```
sudo dtoverlay rpr0521_rpi.dtbo
```
To reload it again it must be removed first, to do that use -r parameter
```
sudo dtoverlay -r rpr0521_rpi
```
There's a udev-rule that is used by the test application to create a name
distinguishable symlink in /dev/input. Since this sensor has two sensors
included (proximity and ambient light), two device interfaces are created, this
is true also in the sysfs interface. After installing the udev rule, it will
create a rpr0521_proximity and rpr0521_als symlink in /dev/input, to install it use
```
sudo cp 10-rpr0521.rules /etc/udev/rules.d/
```
After installing dtbo one needs to reboot Raspberry pi to activate changes. If using dynamic overlay loadind, some changes will not work, so install and reboot is highly recommended.

Compile the driver module
```
make
```
and load it
```
sudo insmod rpr0521.ko
```
or make sure that module is loaded on boot.

```
sudo make install
sudo depmod -a
sudo sh -c 'echo rpr0521 >> /etc/modules'
```

# Driver usage #

rpr0521/test directory contains an example application that reads sensor data.
This may serve as a starting point for your own application. After loading the
driver start the measurement with default values and start example application.

**Example**
```
# read sensor data of proximity
sudo ./read_rpr0521 -p

# read sensor data of ambient light sensor
sudo ./read_rpr0521 -a

# or both
sudo ./read_rpr0521 -a -p

# change proximity delay then ALS delay and read data
sudo ./read_rpr0521 -a -p -s 50 -l 50

# print application usage with all parameters
./read_rpr0521 -H

# list current configurations
./read_rpr0521 -C
```

## Driver control interface ##

**sysfs interface**
files in are all readable and writable.
```
/sys/class/input/eventX/device/
	enable
		0 = measurement disabled, 1 = measurement enabled
	proximity_delay
		proximity measurement time in milliseconds, 0 (standby), 10, 40, 50, 100 or 400
	als_delay
		ALS measurement time in milliseconds, 0 (standby), 50, 100 or 400
	proximity_offset
		proximity interrupt threshold, 0-511

```

## Driver data interface (input device) ##

The driver reports raw sensor data as input event of type EV_ABS,
for actual values, check include/uapi/linux/input-event-codes.h

```
proximity 	ABS_DISTANCE
ALS LUX		ABS_MISC
```
