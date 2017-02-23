/*
The MIT License (MIT)
Copyright (c) 2016 Kionix Inc.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __KXG03_REGISTERS_H__
#define __KXG03_REGISTERS_H__
/* registers */
#define KXG03_TEMP_OUT_L 0x00
#define KXG03_TEMP_OUT_H 0x01
#define KXG03_GYRO_XOUT_L 0x02
#define KXG03_GYRO_XOUT_H 0x03
#define KXG03_GYRO_YOUT_L 0x04
#define KXG03_GYRO_YOUT_H 0x05
#define KXG03_GYRO_ZOUT_L 0x06
#define KXG03_GYRO_ZOUT_H 0x07
#define KXG03_ACC_XOUT_L 0x08
#define KXG03_ACC_XOUT_H 0x09
#define KXG03_ACC_YOUT_L 0x0A
#define KXG03_ACC_YOUT_H 0x0B
#define KXG03_ACC_ZOUT_L 0x0C
#define KXG03_ACC_ZOUT_H 0x0D
// Auxiliary Sensor #1 output data bytes AUX1_OUT1 through AUX1_OUT6
#define KXG03_AUX1_OUT1 0x0E
#define KXG03_AUX1_OUT2 0x0F
#define KXG03_AUX1_OUT3 0x10
#define KXG03_AUX1_OUT4 0x11
#define KXG03_AUX1_OUT5 0x12
#define KXG03_AUX1_OUT6 0x13
// Auxiliary Sensor #2 output data bytes AUX2_OUT1 through AUX2_OUT6
#define KXG03_AUX2_OUT1 0x14
#define KXG03_AUX2_OUT2 0x15
#define KXG03_AUX2_OUT3 0x16
#define KXG03_AUX2_OUT4 0x17
#define KXG03_AUX2_OUT5 0x18
#define KXG03_AUX2_OUT6 0x19
// Number of ODR cycles spent in wake state as measured in accelerometer ODRa_wake/ODRa_sleep periods. Data byte WAKE_CNT_L and WAKE_CNT_H.
#define KXG03_WAKE_CNT_L 0x1A
#define KXG03_WAKE_CNT_H 0x1B
// Number of ODR cycles spent in sleep state as measured in accelerometer ODRa_wake/ODRa_sleep periods. Data byte SLEEP_CNT_L and SLEEP_CNT_H.
#define KXG03_SLEEP_CNT_L 0x1C
#define KXG03_SLEEP_CNT_H 0x1D
// Reports the number of data packets (ODR cycles) currently stored in the buffer.
#define KXG03_BUF_SMPLEV_L 0x1E
#define KXG03_BUF_SMPLEV_H 0x1F
// Reports the number of data packets lost since buffer has been filled.
#define KXG03_BUF_PAST_L 0x20
#define KXG03_BUF_PAST_H 0x21
// Reports the status of Auxiliary Sensors AUX1 and AUX2.
#define KXG03_AUX_STATUS 0x22
// WHO_AM_I
#define KXG03_WHO_AM_I 0x30
// Individual Identification (serial number).
#define KXG03_SN1_MIR 0x31
#define KXG03_SN2_MIR 0x32
#define KXG03_SN3_MIR 0x33
#define KXG03_SN4_MIR 0x34
#define KXG03_STATUS1 0x36
#define KXG03_INT1_SRC1 0x37
#define KXG03_INT1_SRC2 0x38
// Reading this register releases int1 source registers
#define KXG03_INT1_L 0x39
#define KXG03_STATUS2 0x3A
#define KXG03_INT2_SRC1 0x3B
#define KXG03_INT2_SRC2 0x3C
// Reading this register releases int2 source registers
#define KXG03_INT2_L 0x3D
// Accelerometer Wake Mode Control register.
#define KXG03_ACCEL_ODR_WAKE 0x3E
#define KXG03_ACCEL_ODR_SLEEP 0x3F
#define KXG03_ACCEL_CTL 0x40
// Gyroscope Wake Mode Control register.
#define KXG03_GYRO_ODR_WAKE 0x41
#define KXG03_GYRO_ODR_SLEEP 0x42
#define KXG03_STDBY 0x43
// Special control register 1
#define KXG03_CTL_REG_1 0x44
#define KXG03_INT_PIN_CTL 0x45
// Physical interrupt pin INT1 select register.
#define KXG03_INT_PIN1_SEL 0x46
// Physical interrupt pin INT2 select register.
#define KXG03_INT_PIN2_SEL 0x47
// Buffer Full Interrupt enable/mask bit.
#define KXG03_INT_MASK1 0x48
// which axis and direction of detected motion can cause an interrupt.
#define KXG03_INT_MASK2 0x49
// External Synchronous control register.
#define KXG03_FSYNC_CTL 0x4A
#define KXG03_WAKE_SLEEP_CTL1 0x4B
// WUF and BTS threshold mode.
#define KXG03_WAKE_SLEEP_CTL2 0x4C
#define KXG03_WUF_TH 0x4D
#define KXG03_WUF_COUNTER 0x4E
#define KXG03_BTS_TH 0x4F
#define KXG03_BTS_COUNTER 0x50
#define KXG03_AUX_I2C_CTL_REG 0x51
// Read/Write that should be used to store the SAD for auxiliary I2C device 1
#define KXG03_AUX_I2C_SAD1 0x52
// Read/Write that should be used to store the starting data register address for auxiliary I2C device 1
#define KXG03_AUX_I2C_REG1 0x53
// Register address for enable/disable control register for auxiliary I2C device
#define KXG03_AUX_I2C_CTL1 0x54
// Defines bits to toggle in the control register for auxiliary I2C device 1
#define KXG03_AUX_I2C_BIT1 0x55
// Defines register read controls for auxiliary I2C device 1
#define KXG03_AUX_I2C_ODR1_W 0x56
// Defines register read controls for auxiliary I2C device 1
#define KXG03_AUX_I2C_ODR1_S 0x57
#define KXG03_AUX_I2C_SAD2 0x58
#define KXG03_AUX_I2C_REG2 0x59
#define KXG03_AUX_I2C_CTL2 0x5A
#define KXG03_AUX_I2C_BIT2 0x5B
#define KXG03_AUX_I2C_ODR2_W 0x5C
// Defines register read controls for auxiliary I2C device 1
#define KXG03_AUX_I2C_ODR1_S 0x57
// Buffer watermark threshold level L
#define KXG03_BUF_WMITH_L 0x75
// Buffer watermark threshold level H
#define KXG03_BUF_WMITH_H 0x76
// Buffer Trigger mode threshold L
#define KXG03_BUF_TRIGTH_L 0x77
// Buffer Trigger mode threshold H
#define KXG03_BUF_TRIGTH_H 0x78
// Read/write control register that controls sample buffer input in wake mode.
#define KXG03_BUF_CTL2 0x79
// Read/write control register that controls sample buffer input in sleep mode
#define KXG03_BUF_CTL3 0x7A
// Read/write control register that controls aux1 and aux2 buffer input.
#define KXG03_BUF_CTL4 0x7B
// Read/write control register that controls sample buffer operation.
#define KXG03_BUF_EN 0x7C
// This register reports the status of the sample buffer trigger function.
#define KXG03_BUF_STATUS 0x7D
// Latched buffer status information and the entire sample buffer are cleared when any data is written to this register.
#define KXG03_BUF_CLEAR 0x7E
// Data from the buffer should be read using a single SAD+R command
#define KXG03_BUF_READ 0x7F
/* registers bits */
// Aux2 command sequence failure flag
#define KXG03_AUX_STATUS_AUX2FAIL (0x01 << 7)
// Aux2 data read error flag.
#define KXG03_AUX_STATUS_AUX2ERR (0x01 << 6)
// Aux1 has not been enabled or ASIC has successfully sent disable cmd.
#define KXG03_AUX_STATUS_AUX2ST_AUX2_DISABLED (0x00 << 4)
// ASIC is attempting to enable aux sensor via enable sequence.
#define KXG03_AUX_STATUS_AUX2ST_AUX2_WAITING_ENABLE (0x01 << 4)
// ASIC is attempting to disable aux sensor via disable sequence.
#define KXG03_AUX_STATUS_AUX2ST_AUX2_WAITING_DISABLE (0x02 << 4)
// ASIC has successfully sent aux enable cmd.
#define KXG03_AUX_STATUS_AUX2ST_AUX2_RUNNING (0x03 << 4)
// Aux1 command sequence failure flag
#define KXG03_AUX_STATUS_AUX1FAIL (0x01 << 3)
// Aux1 data read error flag.
#define KXG03_AUX_STATUS_AUX1ERR (0x01 << 2)
// Aux1 has not been enabled or ASIC has successfully sent disable cmd.
#define KXG03_AUX_STATUS_AUX1ST_AUX1_DISABLED (0x00 << 0)
// ASIC is attempting to enable aux sensor via enable sequence.
#define KXG03_AUX_STATUS_AUX1ST_AUX1_WAITING_ENABLE (0x01 << 0)
// ASIC is attempting to disable aux sensor via disable sequence.
#define KXG03_AUX_STATUS_AUX1ST_AUX1_WAITING_DISABLE (0x02 << 0)
// ASIC has successfully sent aux enable cmd.
#define KXG03_AUX_STATUS_AUX1ST_AUX1_RUNNING (0x03 << 0)
// WHO_AM_I -value
#define KXG03_WHO_AM_I_WIA_ID (0x24 << 0)
// Reports logical OR of non-masked interrupt sources sent to INT1
#define KXG03_STATUS1_INT1 (0x01 << 7)
// Reset indicator.
#define KXG03_STATUS1_POR (0x01 << 6)
// Auxiliary sensor #2 active flag.
#define KXG03_STATUS1_AUX2_ACT (0x01 << 5)
// Auxiliary sensor #1 active flag.
#define KXG03_STATUS1_AUX1_ACT (0x01 << 4)
#define KXG03_STATUS1_AUX_ERR (0x01 << 3)
// Sleep mode status
#define KXG03_STATUS1_WAKE_SLEEP_SLEEP_MODE (0x00 << 2)
// Wake mode status
#define KXG03_STATUS1_WAKE_SLEEP_WAKE_MODE (0x01 << 2)
// Gyro's run status
#define KXG03_STATUS1_GYRO_RUN (0x01 << 1)
// Gyro's start status
#define KXG03_STATUS1_GYRO_START (0x01 << 0)
// Buffer full interrupt.
#define KXG03_INT1_SRC1_INT1_BFI (0x01 << 7)
// Buffer water mark interrupt.
#define KXG03_INT1_SRC1_INT1_WMI (0x01 << 6)
// Wake up function interrupt.
#define KXG03_INT1_SRC1_INT1_WUFS (0x01 << 5)
// Back to sleep interrupt.
#define KXG03_INT1_SRC1_INT1_BTS (0x01 << 4)
// Aux2 data ready interrupt.
#define KXG03_INT1_SRC1_INT1_DRDY_AUX2 (0x01 << 3)
// Aux1 data ready interrupt.
#define KXG03_INT1_SRC1_INT1_DRDY_AUX1 (0x01 << 2)
// Accelerometer / Temperature data ready interrupt.
#define KXG03_INT1_SRC1_INT1_DRDY_ACCTEMP (0x01 << 1)
// Gyro data ready interrupt.
#define KXG03_INT1_SRC1_INT1_DRDY_GYRO (0x01 << 0)
// Wake up event detected on x-axis, negative direction
#define KXG03_INT1_SRC2_INT1_XNWU (0x01 << 5)
// Wake up event detected on x-axis, positive direction.
#define KXG03_INT1_SRC2_INT1_XPWU (0x01 << 4)
// Wake up event detected on y-axis, negative direction
#define KXG03_INT1_SRC2_INT1_YNWU (0x01 << 3)
// Wake up event detected on y-axis, positive direction.
#define KXG03_INT1_SRC2_INT1_YPWU (0x01 << 2)
// Wake up event detected on z-axis, negative direction.
#define KXG03_INT1_SRC2_INT1_ZNWU (0x01 << 1)
// Wake up event detected on z-axis, positive direction.
#define KXG03_INT1_SRC2_INT1_ZPWU (0x01 << 0)
// Reports logical OR of non-masked interrupt sources sent to INT2
#define KXG03_STATUS2_INT2 (0x01 << 7)
// Reset indicator.
#define KXG03_STATUS2_POR (0x01 << 6)
// Auxiliary sensor #2 active flag.
#define KXG03_STATUS2_AUX2_ACT (0x01 << 5)
// Auxiliary sensor #1 active flag.
#define KXG03_STATUS2_AUX1_ACT (0x01 << 4)
#define KXG03_STATUS2_AUX_ERR (0x01 << 3)
// Sleep mode status
#define KXG03_STATUS2_WAKE_SLEEP_SLEEP_MODE (0x00 << 2)
// Wake mode status
#define KXG03_STATUS2_WAKE_SLEEP_WAKE_MODE (0x01 << 2)
// Gyro's run status
#define KXG03_STATUS2_GYRO_RUN (0x01 << 1)
// Gyro's start status
#define KXG03_STATUS2_GYRO_START (0x01 << 0)
// Buffer full interrupt.
#define KXG03_INT2_SRC1_INT2_BFI (0x01 << 7)
// Buffer water mark interrupt.
#define KXG03_INT2_SRC1_INT2_WMI (0x01 << 6)
// Wake up function interrupt.
#define KXG03_INT2_SRC1_INT2_WUFS (0x01 << 5)
// Back to sleep interrupt.
#define KXG03_INT2_SRC1_INT2_BTS (0x01 << 4)
// Aux2 data ready interrupt.
#define KXG03_INT2_SRC1_INT2_DRDY_AUX2 (0x01 << 3)
// Aux1 data ready interrupt.
#define KXG03_INT2_SRC1_INT2_DRDY_AUX1 (0x01 << 2)
// Accelerometer / Temperature data ready interrupt.
#define KXG03_INT2_SRC1_INT2_DRDY_ACCTEMP (0x01 << 1)
// Gyro data ready interrupt.
#define KXG03_INT2_SRC1_INT2_DRDY_GYRO (0x01 << 0)
// Wake up event detected on x-axis, negative direction
#define KXG03_INT2_SRC2_INT2_XNWU (0x01 << 5)
// Wake up event detected on x-axis, positive direction.
#define KXG03_INT2_SRC2_INT2_XPWU (0x01 << 4)
// Wake up event detected on y-axis, negative direction
#define KXG03_INT2_SRC2_INT2_YNWU (0x01 << 3)
// Wake up event detected on y-axis, positive direction.
#define KXG03_INT2_SRC2_INT2_YPWU (0x01 << 2)
// Wake up event detected on z-axis, negative direction.
#define KXG03_INT2_SRC2_INT2_ZNWU (0x01 << 1)
// Wake up event detected on z-axis, positive direction.
#define KXG03_INT2_SRC2_INT2_ZPWU (0x01 << 0)
// Accel low power mode is disabled in wake state. Accel operates at max sampling rate and navg_wake is ignored.
#define KXG03_ACCEL_ODR_WAKE_LPMODE_W_DISABLED (0x00 << 7)
// Accel low power mode is enabled in wake state. Accel operates in duty cycle mode with number of samples set by navg_wake.
#define KXG03_ACCEL_ODR_WAKE_LPMODE_W_ENABLED (0x01 << 7)
// no average
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_NO_AVG (0x00 << 4)
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_2_SAMPLE_AVG (0x01 << 4)
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_4_SAMPLE_AVG (0x02 << 4)
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_8_SAMPLE_AVG (0x03 << 4)
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_16_SAMPLE_AVG (0x04 << 4)
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_32_SAMPLE_AVG (0x05 << 4)
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_64_SAMPLE_AVG (0x06 << 4)
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_128_SAMPLE_AVG (0x07 << 4)
// odr 0.781Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_25 (0x05 << 0)
// odr 50Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_50 (0x06 << 0)
// odr 100Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_100 (0x07 << 0)
// odr 200Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_200 (0x08 << 0)
// odr 400Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_400 (0x09 << 0)
// odr 800Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_1600 (0x0B << 0)
// odr 3200Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_3200 (0x0C << 0)
// odr 6400Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_6400 (0x0D << 0)
// odr 12800Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_12800 (0x0E << 0)
// odr 51200Hz
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_51200 (0x0F << 0)
#define KXG03_ACCEL_ODR_SLEEP_LPMODE_S_DISABLED (0x00 << 7)
#define KXG03_ACCEL_ODR_SLEEP_LPMODE_S_ENABLED (0x01 << 7)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_NO_AVG (0x00 << 4)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_2_SAMPLE_AVG (0x01 << 4)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_4_SAMPLE_AVG (0x02 << 4)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_8_SAMPLE_AVG (0x03 << 4)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_16_SAMPLE_AVG (0x04 << 4)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_32_SAMPLE_AVG (0x05 << 4)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_64_SAMPLE_AVG (0x06 << 4)
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_128_SAMPLE_AVG (0x07 << 4)
// odr 0.781Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_25 (0x05 << 0)
// odr 50Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_50 (0x06 << 0)
// odr 100Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_100 (0x07 << 0)
// odr 200Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_200 (0x08 << 0)
// odr 400Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_400 (0x09 << 0)
// odr 800Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_1600 (0x0B << 0)
// odr 3200Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_3200 (0x0C << 0)
// odr 6400Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_6400 (0x0D << 0)
// odr 12800Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_12800 (0x0E << 0)
// odr 51200Hz
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_51200 (0x0F << 0)
#define KXG03_ACCEL_CTL_ACC_FS_S_2G (0x00 << 6)
#define KXG03_ACCEL_CTL_ACC_FS_S_4G (0x01 << 6)
#define KXG03_ACCEL_CTL_ACC_FS_S_8G (0x02 << 6)
#define KXG03_ACCEL_CTL_ACC_FS_S_16G (0x03 << 6)
#define KXG03_ACCEL_CTL_ACC_FS_W_2G (0x00 << 2)
#define KXG03_ACCEL_CTL_ACC_FS_W_4G (0x01 << 2)
#define KXG03_ACCEL_CTL_ACC_FS_W_8G (0x02 << 2)
#define KXG03_ACCEL_CTL_ACC_FS_W_16G (0x03 << 2)
// 256dps
#define KXG03_GYRO_ODR_WAKE_GYRO_FS_W_256 (0x00 << 6)
// 512dps
#define KXG03_GYRO_ODR_WAKE_GYRO_FS_W_512 (0x01 << 6)
// 1024dps
#define KXG03_GYRO_ODR_WAKE_GYRO_FS_W_1024 (0x02 << 6)
// 2048dps
#define KXG03_GYRO_ODR_WAKE_GYRO_FS_W_2048 (0x03 << 6)
// 10Hz
#define KXG03_GYRO_ODR_WAKE_GYRO_BW_W_10 (0x00 << 4)
// 20Hz
#define KXG03_GYRO_ODR_WAKE_GYRO_BW_W_20 (0x01 << 4)
// 40Hz
#define KXG03_GYRO_ODR_WAKE_GYRO_BW_W_40 (0x02 << 4)
// 160Hz
#define KXG03_GYRO_ODR_WAKE_GYRO_BW_W_160 (0x03 << 4)
// odr 0.781Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_25 (0x05 << 0)
// odr 50Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_50 (0x06 << 0)
// odr 100Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_100 (0x07 << 0)
// odr 200Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_200 (0x08 << 0)
// odr 400Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_400 (0x09 << 0)
// odr 800Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_1600 (0x0B << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_1600_2 (0x0C << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_1600_3 (0x0D << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_1600_4 (0x0E << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_WAKE_ODRG_W_1600_5 (0x0F << 0)
// 256dps
#define KXG03_GYRO_ODR_SLEEP_GYRO_FS_S_256 (0x00 << 6)
// 512dps
#define KXG03_GYRO_ODR_SLEEP_GYRO_FS_S_512 (0x01 << 6)
// 1024dps
#define KXG03_GYRO_ODR_SLEEP_GYRO_FS_S_1024 (0x02 << 6)
// 2048dps
#define KXG03_GYRO_ODR_SLEEP_GYRO_FS_S_2048 (0x03 << 6)
// 10Hz
#define KXG03_GYRO_ODR_SLEEP_GYRO_BW_S_10 (0x00 << 4)
// 20Hz
#define KXG03_GYRO_ODR_SLEEP_GYRO_BW_S_20 (0x01 << 4)
// 40Hz
#define KXG03_GYRO_ODR_SLEEP_GYRO_BW_S_40 (0x02 << 4)
// 160Hz
#define KXG03_GYRO_ODR_SLEEP_GYRO_BW_S_160 (0x03 << 4)
// odr 0.781Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_25 (0x05 << 0)
// odr 50Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_50 (0x06 << 0)
// odr 100Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_100 (0x07 << 0)
// odr 200Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_200 (0x08 << 0)
// odr 400Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_400 (0x09 << 0)
// odr 800Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_1600 (0x0B << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_1600_2 (0x0C << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_1600_3 (0x0D << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_1600_4 (0x0E << 0)
// odr 1600Hz
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_1600_5 (0x0F << 0)
// Aux2 sensor is enabled in sleep state.
#define KXG03_STDBY_AUX2_STDBY_S_ENABLED (0x00 << 7)
// Aux2 sensor is disabled in sleep state.
#define KXG03_STDBY_AUX2_STDBY_S_DISABLED (0x01 << 7)
// Aux1 sensor is enabled in sleep state.
#define KXG03_STDBY_AUX1_STDBY_S_ENABLED (0x00 << 6)
// Aux1 sensor is disabled in sleep state.
#define KXG03_STDBY_AUX1_STDBY_S_DISABLED (0x01 << 6)
// Gyro sensor is enabled in sleep state.
#define KXG03_STDBY_GYRO_STDBY_S_ENABLED (0x00 << 5)
// Gyro sensor is disabled in sleep state.
#define KXG03_STDBY_GYRO_STDBY_S_DISABLED (0x01 << 5)
// Aux2 sensor is enabled in wake state.
#define KXG03_STDBY_AUX2_STDBY_W_ENABLED (0x00 << 3)
// Aux2 sensor is disabled in wake state.
#define KXG03_STDBY_AUX2_STDBY_W_DISABLED (0x01 << 3)
// Aux1 sensor is enabled in wake state.
#define KXG03_STDBY_AUX1_STDBY_W_ENABLED (0x00 << 2)
// Aux1 sensor is disabled in wake state.
#define KXG03_STDBY_AUX1_STDBY_W_DISABLED (0x01 << 2)
// Gyro sensor is enabled in wake state.
#define KXG03_STDBY_GYRO_STDBY_W_ENABLED (0x00 << 1)
// Gyro sensor is disabled in wake state.
#define KXG03_STDBY_GYRO_STDBY_W_DISABLED (0x01 << 1)
// Accel sensor is enabled.
#define KXG03_STDBY_ACC_STDBY_ENABLED (0x00 << 0)
// Accel sensor is disabled.
#define KXG03_STDBY_ACC_STDBY_DISABLED (0x01 << 0)
// Active high soft reset.
#define KXG03_CTL_REG_1_RST (0x01 << 7)
// I2C interface is not disabled
#define KXG03_CTL_REG_1_I2C_DIS_ENABLED (0x00 << 5)
// I2C interface is disabled
#define KXG03_CTL_REG_1_I2C_DIS_DISABLED (0x01 << 5)
// Temperature output is enabled in sleep mode.
#define KXG03_CTL_REG_1_TEMP_STDBY_S_ENABLED (0x00 << 4)
// Temperature output is disabled in sleep mode.
#define KXG03_CTL_REG_1_TEMP_STDBY_S_DISABLED (0x01 << 4)
// Temperature output is enabled in wake mode.
#define KXG03_CTL_REG_1_TEMP_STDBY_W_ENABLED (0x00 << 3)
// Temperature output is disabled in wake mode.
#define KXG03_CTL_REG_1_TEMP_STDBY_W_DISABLED (0x01 << 3)
// Accel self-test polarity is not inverted..
#define KXG03_CTL_REG_1_ACC_STPOL_NOT_INVERTED (0x00 << 1)
// Accel self-test polarity is inverted..
#define KXG03_CTL_REG_1_ACC_STPOL_INVERTED (0x01 << 1)
// Accel self-test is enabled.
#define KXG03_CTL_REG_1_ACC_ST (0x01 << 0)
// Active high enable for INT2 pin.
#define KXG03_INT_PIN_CTL_IEN2 (0x01 << 7)
#define KXG03_INT_PIN_CTL_IEA2_ACTIVE_LOW (0x00 << 6)
#define KXG03_INT_PIN_CTL_IEA2_ACTIVE_HIGH (0x01 << 6)
#define KXG03_INT_PIN_CTL_IEL2_LATCHED (0x00 << 4)
#define KXG03_INT_PIN_CTL_IEL2_PULSED_40US (0x01 << 4)
#define KXG03_INT_PIN_CTL_IEL2_PULSED_160US (0x02 << 4)
#define KXG03_INT_PIN_CTL_IEL2_REALTIME (0x03 << 4)
// Active high enable for INT1 pin.
#define KXG03_INT_PIN_CTL_IEN1 (0x01 << 3)
#define KXG03_INT_PIN_CTL_IEA1_ACTIVE_LOW (0x00 << 2)
#define KXG03_INT_PIN_CTL_IEA1_ACTIVE_HIGH (0x01 << 2)
#define KXG03_INT_PIN_CTL_IEL1_LATCHED (0x00 << 0)
#define KXG03_INT_PIN_CTL_IEL1_PULSED_40US (0x01 << 0)
#define KXG03_INT_PIN_CTL_IEL1_PULSED_160US (0x02 << 0)
#define KXG03_INT_PIN_CTL_IEL1_REALTIME (0x03 << 0)
// Buffer Full Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_BFI_P1 (0x01 << 7)
// Water Mark Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_WMI_P1 (0x01 << 6)
// Wake Up Function Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_WUF_P1 (0x01 << 5)
// Back To Sleep Function Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_BTS_P1 (0x01 << 4)
// Data Ready Aux2 Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_DRDY_AUX2_P1 (0x01 << 3)
// Data Ready AUX1 Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_DRDY_AUX1_P1 (0x01 << 2)
// Data Ready Accelerometer Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_DRDY_ACCTEMP_P1 (0x01 << 1)
// Data Ready Gyroscope Interrupt for INT1 pin.
#define KXG03_INT_PIN1_SEL_DRDY_GYRO_P1 (0x01 << 0)
// Buffer Full Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_BFI_P2 (0x01 << 7)
// Water Mark Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_WMI_P2 (0x01 << 6)
// Wake Up Function Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_WUF_P2 (0x01 << 5)
// Back To Sleep Function Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_BTS_P2 (0x01 << 4)
// Data Ready Aux2 Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_DRDY_AUX2_P2 (0x01 << 3)
// Data Ready AUX1 Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_DRDY_AUX1_P2 (0x01 << 2)
// Data Ready Accelerometer Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_DRDY_ACCTEMP_P2 (0x01 << 1)
// Data Ready Gyroscope Interrupt for INT2 pin.
#define KXG03_INT_PIN2_SEL_DRDY_GYRO_P2 (0x01 << 0)
// Buffer Full Interrupt enable/mask bit.
#define KXG03_INT_MASK1_BFIE (0x01 << 7)
// Water Mark Interrupt enable/mask bit
#define KXG03_INT_MASK1_WMIE (0x01 << 6)
// Wake-up Function Interrupt enable/mask bit.
#define KXG03_INT_MASK1_WUFE (0x01 << 5)
// Back-to-sleep Function Interrupt enable/mask bit.
#define KXG03_INT_MASK1_BTSE (0x01 << 4)
// Data Ready Aux2 Interrupt enable/mask bit.
#define KXG03_INT_MASK1_DRDY_AUX2 (0x01 << 3)
// Data Ready AUX1 Interrupt enable/mask bit.
#define KXG03_INT_MASK1_DRDY_AUX1 (0x01 << 2)
// Data Ready Accelerometer / Temperature Interrupt enable/mask bit.
#define KXG03_INT_MASK1_DRDY_ACCTEMP (0x01 << 1)
// Data Ready Gyroscope Interrupt enable/mask bit.
#define KXG03_INT_MASK1_DRDY_GYRO (0x01 << 0)
// x negative (x-) mask for WUF/BTS
#define KXG03_INT_MASK2_NXWUE (0x01 << 5)
// x positive (x+) mask for WUF/BTS
#define KXG03_INT_MASK2_PXWUE (0x01 << 4)
// y negative (y-) mask for WUF/BTS
#define KXG03_INT_MASK2_NYWUE (0x01 << 3)
// y positive (y+) mask for WUF/BTS
#define KXG03_INT_MASK2_PYWUE (0x01 << 2)
// z negative (z-) mask for WUF/BTS
#define KXG03_INT_MASK2_NZWUE (0x01 << 1)
// z positive (z+) mask for WUF/BTS
#define KXG03_INT_MASK2_PZWUE (0x01 << 0)
// FSYNC is disabled. SYNC pin is tri-stated.
#define KXG03_FSYNC_CTL_FSYNC_MODE_DISABLED (0x00 << 4)
// FSYNC is enabled. Sync pin is onfigured as input pin. Buffer is updated in sync with external clock applied at SYNC pin.
#define KXG03_FSYNC_CTL_FSYNC_MODE_INPUT_EXT (0x01 << 4)
// FSYNC is enabled. Sync pin is configured as input pin.State of SYNC pin is stored in selected sensor's LSB bit.
#define KXG03_FSYNC_CTL_FSYNC_MODE_INPUT (0x02 << 4)
// FSYNC is disabled. SYNC pin is configured as output pin.
#define KXG03_FSYNC_CTL_FSYNC_MODE_OUTPUT (0x03 << 4)
// SYNC function disabled
#define KXG03_FSYNC_CTL_FSYNC_SEL_DISABLED (0x00 << 0)
// State of SYNC pin is stored in gyro x LSB bit
#define KXG03_FSYNC_CTL_FSYNC_SEL_GYRO_X_LSB (0x01 << 0)
// State of SYNC pin is stored in gyro y LSB bit.
#define KXG03_FSYNC_CTL_FSYNC_SEL_GYRO_Y_LSB (0x02 << 0)
// State of SYNC pin is stored in gyro.z LSB bt
#define KXG03_FSYNC_CTL_FSYNC_SEL_GYRO_Z_LSB (0x03 << 0)
// State of SYNC pin is stored in accel x LSB bit.
#define KXG03_FSYNC_CTL_FSYNC_SEL_ACCEL_X_LSB (0x04 << 0)
// State of SYNC pin is stored in accel y LSB bit.
#define KXG03_FSYNC_CTL_FSYNC_SEL_ACCEL_Y_LSB (0x05 << 0)
// State of SYNC pin is stored in accel z LSB bit.
#define KXG03_FSYNC_CTL_FSYNC_SEL_ACCEL_Z_LSB (0x06 << 0)
// State of SYNC pin is stored in temperature LSB bit
#define KXG03_FSYNC_CTL_FSYNC_SEL_TEMPERATURE_LSB (0x07 << 0)
// SYNC pin outputs gyroscope ODR clock.
#define KXG03_FSYNC_CTL_FSYNC_SEL_GYRO_ODR (0x00 << 0)
// SYNC pin outputs accelerometers ODR clock.
#define KXG03_FSYNC_CTL_FSYNC_SEL_ACC_ODR (0x01 << 0)
// SYNC pin outputs aux1 ODR clock
#define KXG03_FSYNC_CTL_FSYNC_SEL_AUX1_ODR (0x02 << 0)
// SYNC pin outputs aux2 ODR clock
#define KXG03_FSYNC_CTL_FSYNC_SEL_AUX2_ODR (0x03 << 0)
// SYNC pin disabled.
#define KXG03_FSYNC_CTL_FSYNC_SEL_DISABLED_1 (0x04 << 0)
// SYNC pin disabled.
#define KXG03_FSYNC_CTL_FSYNC_SEL_DISABLED_2 (0x05 << 0)
// SYNC pin disabled.
#define KXG03_FSYNC_CTL_FSYNC_SEL_DISABLED_3 (0x06 << 0)
// SYNC pin disabled.
#define KXG03_FSYNC_CTL_FSYNC_SEL_DISABLED_4 (0x07 << 0)
// Active high back-to-sleep function enable
#define KXG03_WAKE_SLEEP_CTL1_BTS_EN (0x01 << 7)
// Active high wake-up function enable.
#define KXG03_WAKE_SLEEP_CTL1_WUF_EN (0x01 << 6)
// Forces transition to sleep state.
#define KXG03_WAKE_SLEEP_CTL1_MAN_SLEEP (0x01 << 5)
// Forces transition to wake state.
#define KXG03_WAKE_SLEEP_CTL1_MAN_WAKE (0x01 << 4)
// odr 0.781Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_25 (0x05 << 0)
// odr 50Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_50 (0x06 << 0)
// odr 100Hz
#define KXG03_WAKE_SLEEP_CTL1_OWUF_100 (0x07 << 0)
#define KXG03_WAKE_SLEEP_CTL2_TH_MODE_ABSOLUTE_THRESHOLD (0x00 << 1)
#define KXG03_WAKE_SLEEP_CTL2_TH_MODE_RELATIVE_THRESHOLD (0x01 << 1)
#define KXG03_WAKE_SLEEP_CTL2_C_MODE_COUNTER_CLEAR (0x00 << 0)
#define KXG03_WAKE_SLEEP_CTL2_C_MODE_COUNTER_DECREASE (0x01 << 0)
// Defines control bit polarity for aux2 enable/disable command sequences.
#define KXG03_AUX_I2C_CTL_REG_AUX_CTL_POL2 (0x01 << 5)
// Defines control bit polarity for aux1 enable/disable command sequences.
#define KXG03_AUX_I2C_CTL_REG_AUX_CTL_POL1 (0x01 << 4)
// FS speed
#define KXG03_AUX_I2C_CTL_REG_AUX_BUS_SPD_100HZ (0x00 << 3)
// HS speed
#define KXG03_AUX_I2C_CTL_REG_AUX_BUS_SPD_400HZ (0x01 << 3)
// Pull up disabled
#define KXG03_AUX_I2C_CTL_REG_AUX_PULL_UP_DISABLED (0x00 << 2)
// Internal 1.5kohm pullup resistor enabled
#define KXG03_AUX_I2C_CTL_REG_AUX_PULL_UP_ENABLED (0x01 << 2)
// AUX I2C not bypassed
#define KXG03_AUX_I2C_CTL_REG_AUX_BYPASS_BYPASSED (0x00 << 1)
// AUX I2C connected to main I2C pins. Pull up disabled
#define KXG03_AUX_I2C_CTL_REG_AUX_BYPASS_NOT_BYPASSED (0x01 << 1)
// No read back
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_NO_READ_BACK (0x00 << 4)
// 1 read back
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_1_READ_BACK (0x01 << 4)
// 2 read back
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_2_READ_BACK (0x02 << 4)
// 3 read back
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_3_READ_BACK (0x03 << 4)
// 4 read back
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_4_READ_BACK (0x04 << 4)
// 5 read back
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_5_READ_BACK (0x05 << 4)
// 6 read back
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_6_READ_BACK (0x06 << 4)
// DNE
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_DNE (0x07 << 4)
// odr 0.781Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_25 (0x05 << 0)
// odr 50Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_50 (0x06 << 0)
// odr 100Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_100 (0x07 << 0)
// odr 200Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_200 (0x08 << 0)
// odr 400Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_400 (0x09 << 0)
// odr 800Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_1600 (0x0B << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_1600_2 (0x0C << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_1600_3 (0x0D << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_1600_4 (0x0E << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_1600_5 (0x0F << 0)
// odr 0.781Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_25 (0x05 << 0)
// odr 50Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_50 (0x06 << 0)
// odr 100Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_100 (0x07 << 0)
// odr 200Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_200 (0x08 << 0)
// odr 400Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_400 (0x09 << 0)
// odr 800Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_1600 (0x0B << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_1600_2 (0x0C << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_1600_3 (0x0D << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_1600_4 (0x0E << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_1600_5 (0x0F << 0)
// No read back
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_NO_READ_BACK (0x00 << 4)
// 1 read back
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_1_READ_BACK (0x01 << 4)
// 2 read back
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_2_READ_BACK (0x02 << 4)
// 3 read back
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_3_READ_BACK (0x03 << 4)
// 4 read back
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_4_READ_BACK (0x04 << 4)
// 5 read back
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_5_READ_BACK (0x05 << 4)
// 6 read back
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_6_READ_BACK (0x06 << 4)
// DNE
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_DNE (0x07 << 4)
// odr 0.781Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_25 (0x05 << 0)
// odr 50Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_50 (0x06 << 0)
// odr 100Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_100 (0x07 << 0)
// odr 200Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_200 (0x08 << 0)
// odr 400Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_400 (0x09 << 0)
// odr 800Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_1600 (0x0B << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_1600_2 (0x0C << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_1600_3 (0x0D << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_1600_4 (0x0E << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_1600_5 (0x0F << 0)
// odr 0.781Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_0P781 (0x00 << 0)
// odr 1.563Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_1P563 (0x01 << 0)
// odr 3.125Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_3P125 (0x02 << 0)
// odr 6.25Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_6P25 (0x03 << 0)
// odr 12.5Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_12P5 (0x04 << 0)
// odr 25Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_25 (0x05 << 0)
// odr 50Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_50 (0x06 << 0)
// odr 100Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_100 (0x07 << 0)
// odr 200Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_200 (0x08 << 0)
// odr 400Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_400 (0x09 << 0)
// odr 800Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_800 (0x0A << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_1600 (0x0B << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_1600_2 (0x0C << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_1600_3 (0x0D << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_1600_4 (0x0E << 0)
// odr 1600Hz
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_1600_5 (0x0F << 0)
// controls the Temperature input into the sample buffer
#define KXG03_BUF_CTL2_BUF_TEMP_W (0x01 << 6)
// controls the Accelerometer axis input into the sample buffer
#define KXG03_BUF_CTL2_BUF_ACC_W_X (0x01 << 5)
#define KXG03_BUF_CTL2_BUF_ACC_W_Y (0x01 << 4)
#define KXG03_BUF_CTL2_BUF_ACC_W_Z (0x01 << 3)
// controls the Gyroscope axis input into the sample buffer.
#define KXG03_BUF_CTL2_BUF_GYR_W_X (0x01 << 2)
#define KXG03_BUF_CTL2_BUF_GYR_W_Y (0x01 << 1)
#define KXG03_BUF_CTL2_BUF_GYR_W_Z (0x01 << 0)
// controls the Temperature input into the sample buffer.
#define KXG03_BUF_CTL3_BUF_TEMP_W (0x01 << 6)
// controls the Accelerometer axis input into the sample buffer.
#define KXG03_BUF_CTL3_BUF_ACC_W_X (0x01 << 5)
#define KXG03_BUF_CTL3_BUF_ACC_W_Y (0x01 << 4)
#define KXG03_BUF_CTL3_BUF_ACC_W_Z (0x01 << 3)
// controls the Gyroscope axis input into the sample buffer.
#define KXG03_BUF_CTL3_BUF_GYR_W_X (0x01 << 2)
#define KXG03_BUF_CTL3_BUF_GYR_W_Y (0x01 << 1)
#define KXG03_BUF_CTL3_BUF_GYR_W_Z (0x01 << 0)
// controls the aux2 input into the sample buffer in sleep mode.
#define KXG03_BUF_CTL4_BUF_AUX2_S (0x01 << 3)
// controls the aux1 axis input into the sample buffer in sleep mode
#define KXG03_BUF_CTL4_BUF_AUX1_S (0x01 << 2)
// controls the aux2 input into the sample buffer in wake mode.
#define KXG03_BUF_CTL4_BUF_AUX2_W (0x01 << 1)
#define KXG03_BUF_CTL4_BUF_AUX1_W (0x01 << 0)
#define KXG03_BUF_EN_BUFE (0x01 << 7)
#define KXG03_BUF_EN_BUF_SYM_SYMBOL_MODE_DISABLED (0x00 << 2)
#define KXG03_BUF_EN_BUF_SYM_SINGLE_SYMBOL_MODE_ENABLED (0x01 << 2)
#define KXG03_BUF_EN_BUF_SYM_DUAL_SYMBOL_TRANS_MODE_ENABLED (0x02 << 2)
#define KXG03_BUF_EN_BUF_SYM_DUAL_SYMBOL_FRAME_MODE_ENABLED (0x03 << 2)
#define KXG03_BUF_EN_BUF_M_FIFO (0x00 << 0)
#define KXG03_BUF_EN_BUF_M_STREAM (0x01 << 0)
#define KXG03_BUF_EN_BUF_M_TRIGGER (0x02 << 0)
#define KXG03_BUF_EN_BUF_M_FILO (0x03 << 0)
 /*registers bit masks */

#define KXG03_AUX_STATUS_AUX2ST_MASK 0x30

#define KXG03_AUX_STATUS_AUX1ST_MASK 0x03

#define KXG03_WHO_AM_I_WIA_MASK 0xFF
#define KXG03_STATUS1_WAKE_SLEEP_MASK 0x04
#define KXG03_STATUS2_WAKE_SLEEP_MASK 0x04

#define KXG03_ACCEL_ODR_WAKE_LPMODE_W_MASK 0x80
// The max over sampling rate (or max number of samples averaged) varies with ODR
#define KXG03_ACCEL_ODR_WAKE_NAVG_W_MASK 0x70
#define KXG03_ACCEL_ODR_WAKE_ODRA_W_MASK 0x0F

#define KXG03_ACCEL_ODR_SLEEP_LPMODE_S_MASK 0x80
#define KXG03_ACCEL_ODR_SLEEP_NAVG_S_MASK 0x70
#define KXG03_ACCEL_ODR_SLEEP_ODRA_S_MASK 0x0F
// Accelerometer sleep mode full scale range select.
#define KXG03_ACCEL_CTL_ACC_FS_S_MASK 0xC0
#define KXG03_ACCEL_CTL_ACC_FS_W_MASK 0x0C
#define KXG03_GYRO_ODR_WAKE_GYRO_FS_W_MASK 0xC0
// Gyroscope bandwidth selection in wake mode.
#define KXG03_GYRO_ODR_WAKE_GYRO_BW_W_MASK 0x30
// gyroscope ODR in wake mode
#define KXG03_GYRO_ODR_WAKE_ODRG_W_MASK 0x0F
// Gyroscope angular velocity range sleep mode
#define KXG03_GYRO_ODR_SLEEP_GYRO_FS_S_MASK 0xC0
// Gyroscope bandwidth selection in sleep mode.
#define KXG03_GYRO_ODR_SLEEP_GYRO_BW_S_MASK 0x30
// gyroscope ODR in sleep mode
#define KXG03_GYRO_ODR_SLEEP_ODRG_S_MASK 0x0F

#define KXG03_STDBY_AUX2_STDBY_S_MASK 0x80

#define KXG03_STDBY_AUX1_STDBY_S_MASK 0x40

#define KXG03_STDBY_GYRO_STDBY_S_MASK 0x20

#define KXG03_STDBY_AUX2_STDBY_W_MASK 0x08

#define KXG03_STDBY_AUX1_STDBY_W_MASK 0x04

#define KXG03_STDBY_GYRO_STDBY_W_MASK 0x02

#define KXG03_STDBY_ACC_STDBY_MASK 0x01
// Active high I2C disable bit
#define KXG03_CTL_REG_1_I2C_DIS_MASK 0x20

#define KXG03_CTL_REG_1_TEMP_STDBY_S_MASK 0x10

#define KXG03_CTL_REG_1_TEMP_STDBY_W_MASK 0x08

#define KXG03_CTL_REG_1_ACC_STPOL_MASK 0x02
// Interrupt polarity select for INT2 pin.
#define KXG03_INT_PIN_CTL_IEA2_MASK 0x40
#define KXG03_INT_PIN_CTL_IEL2_MASK 0x30
// Interrupt polarity select for INT1 pin.
#define KXG03_INT_PIN_CTL_IEA1_MASK 0x04
#define KXG03_INT_PIN_CTL_IEL1_MASK 0x03

#define KXG03_FSYNC_CTL_FSYNC_MODE_MASK 0x30
// if(fsync_mode=2'b10)
#define KXG03_FSYNC_CTL_FSYNC_SEL_MASK 0x07
// if(fsync_mode=2'b11)
#define KXG03_FSYNC_CTL_FSYNC_SEL_MASK 0x07
// the Output Data Rate for the wake up (motion detection).
#define KXG03_WAKE_SLEEP_CTL1_OWUF_MASK 0x07

#define KXG03_WAKE_SLEEP_CTL2_TH_MODE_MASK 0x02
#define KXG03_WAKE_SLEEP_CTL2_C_MODE_MASK 0x01
// Sets I2C bus speed.
#define KXG03_AUX_I2C_CTL_REG_AUX_BUS_SPD_MASK 0x08
// Active high pull up enable.
#define KXG03_AUX_I2C_CTL_REG_AUX_PULL_UP_MASK 0x04
// Active high bypass enable
#define KXG03_AUX_I2C_CTL_REG_AUX_BYPASS_MASK 0x02
// Number of bytes read back via Auxiliary I2C bus from device 1
#define KXG03_AUX_I2C_ODR1_W_AUX1_D_MASK 0x70
// Determines rate at which aux1 output is polled by ASIC in aux1 wake state
#define KXG03_AUX_I2C_ODR1_W_AUX1ODRW_MASK 0x0F
// Determines rate at which aux1 output is polled by ASIC in aux1 sleep
#define KXG03_AUX_I2C_ODR1_S_AUX1ODRS_MASK 0x0F
// Number of bytes read back via Auxiliary I2C bus from device 2
#define KXG03_AUX_I2C_ODR2_W_AUX2_D_MASK 0x70
// Determines rate at which aux2 output is polled by ASIC in aux2 wake state
#define KXG03_AUX_I2C_ODR2_W_AUX2ODRW_MASK 0x0F
// Determines rate at which aux2 output is polled by ASIC in aux2 sleep
#define KXG03_AUX_I2C_ODR1_S_AUX2ODRS_MASK 0x0F
#define KXG03_BUF_EN_BUF_SYM_MASK 0x0C
#define KXG03_BUF_EN_BUF_M_MASK 0x03

#endif

