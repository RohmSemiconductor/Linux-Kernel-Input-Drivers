# ROHM BH1749 Ambient Light Sensor for Linux kernel #

bh1749 Linux kernel driver uses I2C bus to communicate with sensor. Driver uses input devices to report sensor data to user space.
Raspberry pi3 was used as a development target.

Find the datasheet of the sensor here: https://www.rohm.com/datasheet/BH1749NUC/bh1749nuc-e

# Physical connection to target hardware

|rpi pin | sensor pin |
|:------:|------------|
|   1    | VCC        |
|   2    | led (5V)   |
|   3    | SDA        |
|   5    | SCL        |
|   33   | INT        |
|   9    | GND        |

This pin order is used with ROHM Multi-Platform Adapter Board A1.

# Device-tree configuration #

The driver requires device tree configuration, i2c-address (0x39) and interrupt information (depends on hw setup, here we use gpio 13) are mandatory. There are optional parameters for sensor default settings:

- *ir-gain*: integer with values 1 or 32. Gain factor for IR-measurement.
- *rgb-gain*: integer with value 1 or 32. Gain factor for RGBC-measurement.
- *measurement-time*: integer with value 35, 120 or 240. Measurement time in milliseconds.
- One of following or none. If none defined interrupts are disabled and rest of the fields are not used.
  - *int-active-after-every-measurement*: interrupt is raised after a measurement is ready.
  - *int-judge-after-1-measurement*: interrupt is raised if measured value is below threshold-low or above threshold-high.
  - *int-judge-after-4-measurement*: interrupt is raised if 4 sequential measured values are below threshold-low or above threshold-high.
  - *int-judge-after-8-measurement*: interrupt is raised if 8 sequential measured values are below threshold-low or above threshold-high.
- *interrupt-threshold-high*: integer with value from 0 to 0xFFFF. threshold-high value, effective when int-judge-after-X-measurement is defined.
- *interrupt-threshold-low*: integer with value from 0  to 0xFFFF. threshold-low value, effective when int-judge-after-X-measurement is defined
- *interrupt-source*: 1-character string; "r" for red, "g" for green, "b" for blue. Defines channel that uses the interrupt.

**Example** device tree configuration. I2C address is defined to be 0x39 and gpio interrupt 13 with falling edge is used as interrupt. Device raises interrupt when clear channel data exceeds 511 (0x1FF).
```
        bh1749@5b{
                compatible = "rohm,bh1749";
                reg = <0x39>;
                interrupt-parent = <&gpio>;
                interrupt = <13 2>;
                rohm,ir-gain = <1>;
                rohm,rgb-gain = <1>;
                rohm,measurement-time = <240>;
                rohm,int-judge-after-1-measurement;
		rohm,interrupt-source = "r";
                rohm,interrupt-threshold-high = <0x1FF>;
		rohm,interrupt-threshold-low = <0>
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

Device-tree overlay for Raspberry pi3 is called bh1749_rpi.dts. Modify it for your needs, build and install with following commands:
```
make dtbo
sudo make dtbo_install
```
Add following line to /boot/config.txt
```
dtoverlay=bh1749_rpi
```
There's a udev-rule that is used by the test application to create a name
distinguishable symlink in /dev/, to install it use
```
sudo cp 10-bh1749.rules /etc/udev/rules.d/
```
After installing dtbo one needs to reboot Raspberry pi to activate changes. If using dynamic overlay loadind, some changes will not work, so install and reboot is highly recommended.

Compile the driver module
```
make
```
and load it
```
sudo insmod bh1749.ko
```
or make sure that module is loaded on boot.

```
sudo make install
sudo depmod -a
sudo sh -c 'echo bh1749 >> /etc/modules'
```

# Driver usage #

bh1749/test directory contains an example application that reads sensor data.
This may serve as a starting point for your own application. After loading the
driver start the measurement with default values and start example application.

**Example**
```
# read sensor data with measurement time of 120 ms
sudo ./read_bh1749 -m 120

# print application usage with all parameters
./read_bh1749 -H

# list current configurations
./read_bh1749 -C
```

## Driver control interface ##

**sysfs interface**
files in are all readable and writable.
```
/sys/class/input/eventX/device/
	enable
		0 = measurement disabled, 1 = measurement enabled
	measurement_time
		measurement time in milliseconds, 35, 120 or 240
	rgb_gain
		gain of RGBC measurement, 1 or 32
	ir_gain
		gain of IR measurement, 1 or 32
	int_judgement
                interrupt judgement value, 0 for always interrupt. 1, 4, 8 for sequential threshold comparison.
	int_threshold_high
		0-65535, when measured value exceeds this value, interrupt is raised.
	int_threshold_low
		0-65535, when measured value is below this value, interrupt is raised.
	int_source
		channel which is used for interrupt. 'r', 'g', 'b'
```

## Driver data interface (input device) ##

The driver reports raw sensor data as input event of type EV_MSC,
for actual values, check include/uapi/linux/input-event-codes.h

```
red		MSC_SERIAL
green		MSC_PULSELED
blue		MSC_GESTURE
green2		MSC_TIMESTAMP
IR		MSC_SCAN
```
