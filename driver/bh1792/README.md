# ROHM BH1792 Pulse Sensor driver for Linux kernel #

[ROHM Optical Sensor for Heart Rate Monitor IC - BH1792GLC](http://www.rohm.com/web/global/datasheet/BH1792GLC/bh1792glc-e)

bh1792 Linux kernel driver uses I2C bus to communicate with sensor. Driver uses input devices to report sensor data to user space.

Raspberry pi3 was used as a development target.

# Physical connection to target hardware

|rpi pin | sensor pin |
|:------:|------------|
|   1    | VCC        |
|   2    | led (5V)   |
|   3    | SDA        |
|   5    | SCL        |
|   33   | INT        |
|   9    | GND        |

This pin order is used with ROHM Multi-Platform Adapter Board A1. When using A1 board 5V for led is available at pin 4 of connector J1.

# Device-tree configuration #

The driver requires device tree configuration, i2c-address (0x5b) and interrupt information (depends on hw setup, here we use gpio 13) are mandatory. There are optional parameters for sensor default settings:

- *operation-mode*: integer value 0-3 
  -  0=SYNCHRONIZED,
  -  1=NON_SYNCHRONIZED,
  -  2=SINGLE_GREEN,
  -  3=SINGLE_IR
- *led1-current*: integer value 0-63, current in milliamps
- *led2-current*: integer value 0-63, current in milliamps
- *ir-threshold*: integer value 0-65535, interrupt threshold on non-syncronized measurement
- *frequency*: integer value, one of 32, 64, 128, 256, 1024, synchronized measurement frequency
- *irq-gpio*: for targets using Android 6.0 or some other variant with relatively old kernel, this setting overrides interrupt and interrupt-parent fields

**Example** device tree configuration
```
        bh1792@5b{
                compatible = "rohm,bh1792";
                reg = <0x5b>;
                interrupt-parent = <&gpio>;
                interrupt = <13 2>;
                /* optional parameters */
                rohm,operation-mode = <0>;
                rohm,led1-current = <4>;
                rohm,led2-current = <0>;
                rohm,ir-threshold = <0>;
                rohm,frequency = <64>;
        };

```


# How to build and install driver module #

Kernel headers are needed to succesfully build the module, install them first.
```
sudo apt update
sudo apt upgrade
sudo reboot # maybe a good idea if your kernel was updated in previous step
sudo apt install raspberrypi-kernel-headers
```

Installing udev-rule needed for test-application
```
sudo cp 10-bh1792.rules /etc/udev/rules.d/
```
Device-tree overlay for Raspberry pi3 is called bh1792_rpi_overlay.dts. Modify it for your needs, build and install with following commands:
```
make dtbo
sudo make dtbo_install
```
Add following line to /boot/config.txt
```
dtoverlay=bh1792_rpi
```
After installing dtbo one needs to reboot Raspberry pi to activate changes. If using dynamic overlay loadind, some changes will not work, so install and reboot is highly recommended.

Compile the driver module
```
make
```
and load it
```
sudo insmod bh1792.ko
```
or make sure that module is loaded on boot.

```
sudo make install
sudo depmod -a
sudo sh -c 'echo bh1792 >> /etc/modules'

```
Please note that module gets automatically loaded when booting the target next time.


# Driver usage #

bh1792/test directory contains example application that reads sensor data. This may serve as a starting point for your own application. After loading the driver, start the measurement with default values and start example application
```
cd test
make
sudo ./read_bh1792 -m 0 -f 1024 -c 1
```

## Driver control interface ##

**sysfs interface**
files in are all readable and writable.
```
/sys/class/input/inputX/
	enable
		0 = measurement disabled, 1 = measurement enabled
	freq
		measurement frequency in synchronized mode (Hz) 32, 64, 128, 256, 1024
		Cannot be changed if measurement enabled
	led1_current
		led1 current (mA) 0-63
	led2_current
		led2 current (mA) 0-63
	operating_mode
		0 = synchronized mode (Green light)
		1 = non-synchronized mode (IR light)
		2 = single measurement (Green light)
		3 = single measurement (IR light)
	threshold
		0-65535, IR threshold value used in non-synchronized mode.
		When measured value exceeds the threshold, interrupt is raised.


```

## Driver data interface (input device) ##

The driver reports raw sensor data as Misc input event (EV_MSC)
with following event codes:
```
measurement data                 MSC_SERIAL
fifo interrupt timestamp MSB     MSC_PULSELED
fifo interttup timestamp LSB     MSC_SCAN
```
