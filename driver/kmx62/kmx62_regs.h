
 /* registers */
//"This register can be used for supplier recognition
#define KMX62_WHO_AM_I 0x00
//This Register tells which function caused an interrupt.
#define KMX62_INS1 0x01
//
#define KMX62_INS2 0x02
//
#define KMX62_INS3 0x03
//
#define KMX62_INL 0x05
//
#define KMX62_ACCEL_XOUT_L 0x0A
//
#define KMX62_ACCEL_YOUT_L 0x0C
//
#define KMX62_ACCEL_ZOUT_L 0x0E
//
#define KMX62_MAG_XOUT_L 0x10
//
#define KMX62_MAG_YOUT_L 0x12
//
#define KMX62_MAG_ZOUT_L 0x14
//
#define KMX62_TEMP_OUT_L 0x16
//Interrupts reported on GPIO1.
#define KMX62_INC1 0x2A
//Interrupts reported on GPIO2.
#define KMX62_INC2 0x2B
//Interrupt control 3 GPIO pin configuration.
#define KMX62_INC3 0x2C
//This register controls which accelerometer axis and direction of detected motion can cause an interrupt.
#define KMX62_INC4 0x2D
//This register controls which magnetometer axis and direction of detected motion can cause an interrupt.
#define KMX62_INC5 0x2E
//This register has control settings for the accelerometer motion interrupt function.
#define KMX62_AMI_CNTL1 0x2F
//This register has control settings for the accelerometer motion interrupt function.
#define KMX62_AMI_CNTL2 0x30
//Accelerometer Motion Control 3: This register has control settings for the accelerometer motion interrupt function.
#define KMX62_AMI_CNTL3 0x31
//Magnetometer Motion Control 1 This register has control settings for the magnetometer motion interrupt function.
#define KMX62_MMI_CNTL1 0x32
//Magnetometer Motion Control 2 This register has control settings for the magnetometer motion interrupt function.
#define KMX62_MMI_CNTL2 0x33
//Magnetometer Motion Control 3 This register has control settings for the magnetometer motion interrupt function.
#define KMX62_MMI_CNTL3 0x34
//Free Fall Control 1 This register has control settings for the free fall interrupt function.
#define KMX62_FFI_CNTL1 0x35
//Free Fall Control 2 This register has control settings for the free fall interrupt function.
#define KMX62_FFI_CNTL2 0x36
//Free Fall Control 3 This register has control settings for the free fall interrupt function.
#define KMX62_FFI_CNTL3 0x37
//Output data control register
#define KMX62_ODCNTL 0x38
//Control register 1 Control register that controls the main feature set.
#define KMX62_CNTL1 0x39
//Control register 2 This is used to enable and disable the sensors.
#define KMX62_CNTL2 0x3A
//
#define KMX62_COTR 0x3C
//These registers control the buffer sample buffer operation.
#define KMX62_BUF_CTRL_1 0x77
//These registers control the buffer sample buffer operation.
#define KMX62_BUF_CTRL_2 0x78
//These registers control the buffer sample buffer operation.
#define KMX62_BUF_CTRL_3 0x79
//Latched buffer status information and the entire sample buffer are cleared when any data is written to this register.
#define KMX62_BUF_CLEAR 0x7A
//
#define KMX62_BUF_STATUS_1 0x7B
//
#define KMX62_BUF_STATUS_2 0x7C
//
#define KMX62_BUF_STATUS_3 0x7D

 /* registers bits */ 
//no interrupt event
#define KMX62_INS1_INT_NO_INT (0x0 << 7)
//interrupt event has occurred
#define KMX62_INS1_INT_INT (0x1 << 7)
//Buffer is not full
#define KMX62_INS1_BFI_0 (0x0 << 6)
//Buffer is full
#define KMX62_INS1_BFI_1 (0x1 << 6)
//Buffer watermark not reached
#define KMX62_INS1_WMI_0 (0x0 << 5)
//Buffer watermark reached
#define KMX62_INS1_WMI_1 (0x1 << 5)
//New acceleration data not available
#define KMX62_INS1_DRDY_A_0 (0x0 << 4)
//New acceleration data available
#define KMX62_INS1_DRDY_A_1 (0x1 << 4)
//New acceleration data not available
#define KMX62_INS1_DRDY_M_0 (0x0 << 3)
//New acceleration data available
#define KMX62_INS1_DRDY_M_1 (0x1 << 3)
//Free fall has activated the interrupt
#define KMX62_INS1_FFI_1 (0x1 << 2)
//No free fall
#define KMX62_INS1_FFI_0 (0x0 << 2)
//Motion has activated the interrupt
#define KMX62_INS1_AMI_1 (0x1 << 1)
//No motion
#define KMX62_INS1_AMI_0 (0x0 << 1)
//Motion has activated the interrupt
#define KMX62_INS1_MMI_1 (0x1 << 0)
//No motion
#define KMX62_INS1_MMI_0 (0x0 << 0)
//x negative (x-)
#define KMX62_INS2_AXNI (0x01 << 5)
//x positive (x+)
#define KMX62_INS2_AXPI (0x01 << 4)
//y negative (y-)
#define KMX62_INS2_AYNI (0x01 << 3)
//y positive (y+)
#define KMX62_INS2_AYPI (0x01 << 2)
//z negative (z-)
#define KMX62_INS2_AZNI (0x01 << 1)
//z positive (z+)
#define KMX62_INS2_AZPI (0x01 << 0)
//x negative (x-)
#define KMX62_INS3_MXNI (0x01 << 5)
//x positive (x+)
#define KMX62_INS3_MXPI (0x01 << 4)
//y negative (y-)
#define KMX62_INS3_MYNI (0x01 << 3)
//y positive (y+)
#define KMX62_INS3_MYPI (0x01 << 2)
//z negative (z-)
#define KMX62_INS3_MZNI (0x01 << 1)
//z positive (z+)
#define KMX62_INS3_MZPI (0x01 << 0)
//Buffer full interrupt reported on GPIO1
#define KMX62_INC1_BFI1 (0x01 << 6)
//Watermark interrupt reported on GPIO1
#define KMX62_INC1_WMI1 (0x01 << 5)
//Accelerometer Data ready reported on GPIO1
#define KMX62_INC1_DRDY_A1 (0x01 << 4)
//Magnetometer Data ready reported on GPIO1
#define KMX62_INC1_DRDY_M1 (0x01 << 3)
//Accelerometer Freefall interrupt reported on GPIO1
#define KMX62_INC1_FFI1 (0x01 << 2)
//Accelerometer motion interrupt reported on GPIO1
#define KMX62_INC1_AMI1 (0x01 << 1)
//Magnetometer motion interrupt reported on GPIO1
#define KMX62_INC1_MMI1 (0x01 << 0)
//Buffer full interrupt reported on GPIO2
#define KMX62_INC2_BFI2 (0x01 << 6)
//Watermark interrupt reported on GPIO2
#define KMX62_INC2_WMI2 (0x01 << 5)
//Accelerometer Data ready reported on GPIO2
#define KMX62_INC2_DRDY_A2 (0x01 << 4)
//Magnetometer Data ready reported on GPIO2
#define KMX62_INC2_DRDY_M2 (0x01 << 3)
//Accelerometer Freefall interrupt reported on GPIO2
#define KMX62_INC2_FFI2 (0x01 << 2)
//Accelerometer motion interrupt reported on GPIO2
#define KMX62_INC2_AMI2 (0x01 << 1)
//Magnetometer motion interrupt reported on GPIO2
#define KMX62_INC2_MMI2 (0x01 << 0)
//push-pull
#define KMX62_INC3_IED2_0 (0x0 << 7)
//open-drain
#define KMX62_INC3_IED2_1 (0x1 << 7)
//active low
#define KMX62_INC3_IEA2_0 (0x0 << 6)
//active high
#define KMX62_INC3_IEA2_1 (0x1 << 6)
//"latched/unlatched. Unlatched feature is available for FFI
#define KMX62_INC3_IEL2_LATCHED (0x0 << 4)
//pulsed.  In pulse mode the pulse width is 50us for normal mode and 10us for debug mode (high ODR rates).
#define KMX62_INC3_IEL2_PULSED (0x1 << 4)
//trigger input for FIFO
#define KMX62_INC3_IEL2_2 (0x2 << 4)
//trigger input for FIFO
#define KMX62_INC3_IEL2_3 (0x3 << 4)
//push-pull
#define KMX62_INC3_IED1_0 (0x0 << 3)
//open-drain
#define KMX62_INC3_IED1_1 (0x1 << 3)
//active low
#define KMX62_INC3_IEA1_0 (0x0 << 2)
//active high
#define KMX62_INC3_IEA1_1 (0x1 << 2)
//"latched/unlatched. Unlatched feature is available for FFI
#define KMX62_INC3_IEL1_LATCHED (0x0 << 0)
//pulsed.  In pulse mode the pulse width is 50us for normal mode and 10us for debug mode (high ODR rates).
#define KMX62_INC3_IEL1_PULSED (0x1 << 0)
//trigger input for FIFO
#define KMX62_INC3_IEL1_2 (0x2 << 0)
//trigger input for FIFO
#define KMX62_INC3_IEL1_3 (0x3 << 0)
//12.5Hz
#define KMX62_ODCNTL_OSA_12P5 (0x0 << 0)
//25Hz
#define KMX62_ODCNTL_OSA_25 (0x1 << 0)
//50Hz
#define KMX62_ODCNTL_OSA_50 (0x2 << 0)
//100Hz
#define KMX62_ODCNTL_OSA_100 (0x3 << 0)
//200Hz
#define KMX62_ODCNTL_OSA_200 (0x4 << 0)
//400Hz
#define KMX62_ODCNTL_OSA_400 (0x5 << 0)
//800Hz
#define KMX62_ODCNTL_OSA_800 (0x6 << 0)
//1600Hz
#define KMX62_ODCNTL_OSA_1600 (0x7 << 0)
//0.781Hz
#define KMX62_ODCNTL_OSA_0P781 (0x8 << 0)
//1.563Hz
#define KMX62_ODCNTL_OSA_1P563 (0x9 << 0)
//3.125Hz
#define KMX62_ODCNTL_OSA_3P125 (0xa << 0)
//6.25Hz
#define KMX62_ODCNTL_OSA_6P25 (0xb << 0)
//"25.6kHz
#define KMX62_ODCNTL_OSA_25P6ST0P8 (0xc << 0)
//"25.6kHz
#define KMX62_ODCNTL_OSA_25P6ST1P6 (0xd << 0)
//"25.6kHz
#define KMX62_ODCNTL_OSA_25P6ST3P2 (0xe << 0)
//25.6kHz
#define KMX62_ODCNTL_OSA_25P6 (0xf << 0)
//12.5Hz
#define KMX62_ODCNTL_OSM_12P5 (0x0 << 4)
//25Hz
#define KMX62_ODCNTL_OSM_25 (0x1 << 4)
//50Hz
#define KMX62_ODCNTL_OSM_50 (0x2 << 4)
//100Hz
#define KMX62_ODCNTL_OSM_100 (0x3 << 4)
//200Hz
#define KMX62_ODCNTL_OSM_200 (0x4 << 4)
//400Hz
#define KMX62_ODCNTL_OSM_400 (0x5 << 4)
//800Hz
#define KMX62_ODCNTL_OSM_800 (0x6 << 4)
//1600Hz
#define KMX62_ODCNTL_OSM_1600 (0x7 << 4)
//0.781Hz
#define KMX62_ODCNTL_OSM_0P781 (0x8 << 4)
//1.563Hz
#define KMX62_ODCNTL_OSM_1P563 (0x9 << 4)
//3.125Hz
#define KMX62_ODCNTL_OSM_3P125 (0xa << 4)
//6.25Hz
#define KMX62_ODCNTL_OSM_6P25 (0xb << 4)
//"25.6kHz
#define KMX62_ODCNTL_OSM_25P6A (0xc << 4)
//25.6kHz
#define KMX62_ODCNTL_OSM_25P6B (0xd << 4)
//25.6kHz
#define KMX62_ODCNTL_OSM_25P6C (0xe << 4)
//25.6kHz
#define KMX62_ODCNTL_OSM_25P6 (0xf << 4)
//No action
#define KMX62_CNTL1_SRST_0 (0x0 << 7)
//Start POR routine
#define KMX62_CNTL1_SRST_1 (0x1 << 7)
//This bit enables the self-test mode that will produce a change in both the accelerometer and magnetometer transducers and can be measured in the output registers
#define KMX62_CNTL1_STEN (0x01 << 6)
//ST polarity is positive
#define KMX62_CNTL1_STPOL_0 (0x0 << 5)
//ST polarity is negative.
#define KMX62_CNTL1_STPOL_1 (0x1 << 5)
//no action
#define KMX62_CNTL1_COTC_0 (0x0 << 3)
//"sets AAh to COTR register
#define KMX62_CNTL1_COTC_1 (0x1 << 3)
//standby mode
#define KMX62_CNTL2_TEMP_EN_0 (0x0 << 6)
//"operating mode
#define KMX62_CNTL2_TEMP_EN_1 (0x1 << 6)
//
#define KMX62_CNTL2_GSEL_2G (0x0 << 4)
//
#define KMX62_CNTL2_GSEL_4G (0x1 << 4)
//
#define KMX62_CNTL2_GSEL_8G (0x2 << 4)
//
#define KMX62_CNTL2_GSEL_16G (0x3 << 4)
//
#define KMX62_CNTL2_RES_A4M2 (0x0 << 2)
//
#define KMX62_CNTL2_RES_A32M16 (0x1 << 2)
//
#define KMX62_CNTL2_RES_MAX1 (0x2 << 2)
//
#define KMX62_CNTL2_RES_MAX (0x3 << 2)
//FIXME name generated incorrectly
#define KMX62_CNTL2_MAG_EN (0x01 << 1)
//FIXME name generated incorrectly
#define KMX62_CNTL2_ACCEL_EN (0x01 << 0)
//"The buffer collects 384 bytes of data until full
#define KMX62_BUF_CTRL_2_BUF_M_FIFO (0x0 << 1)
//"The buffer holds the last 384 bytes of data.  Once the buffer is full
#define KMX62_BUF_CTRL_2_BUF_M_STREAM (0x1 << 1)
//"When a trigger event occurs (logic high input on TRIG pin)
#define KMX62_BUF_CTRL_2_BUF_M_TRIGGER (0x2 << 1)
//"The buffer holds the last 384 bytes of data.  Once the buffer is full
#define KMX62_BUF_CTRL_2_BUF_M_FILO (0x3 << 1)
//controls the buffer full interrupt
#define KMX62_BUF_CTRL_3_BFI_EN (0x01 << 7)
//ax to be buffered
#define KMX62_BUF_CTRL_3_BUF_AX (0x01 << 6)
//ay to be buffered
#define KMX62_BUF_CTRL_3_BUF_AY (0x01 << 5)
//az to be buffered
#define KMX62_BUF_CTRL_3_BUF_AZ (0x01 << 4)
//mx to be buffered
#define KMX62_BUF_CTRL_3_BUF_MX (0x01 << 3)
//my to be buffered
#define KMX62_BUF_CTRL_3_BUF_MY (0x01 << 2)
//mz to be buffered
#define KMX62_BUF_CTRL_3_BUF_MZ (0x01 << 1)
//temperature to be buffered
#define KMX62_BUF_CTRL_3_BUF_TEMP (0x01 << 0)

 /*registers bit masks */
//reports the combined (OR) interrupt information of all enabled interrupt.
#define KMX62_INS1_INT_MASK 0x80
//indicates that the buffer is full.  This bit is cleared when the data is read until the buffer is not full.
#define KMX62_INS1_BFI_MASK 0x40
//indicates that user-defined buffer watermark has been reached.  This bit is cleared when the data is read until the sample level in the buffer is smaller than the watermark threshold.
#define KMX62_INS1_WMI_MASK 0x20
//indicates that new acceleration data is available.  This bit is cleared when the data is read or the interrupt release register (INL Register) is read.
#define KMX62_INS1_DRDY_A_MASK 0x10
//indicates that new magnetometer data is available.  This bit is cleared when the data is read or the interrupt release register (INL Register) is read.
#define KMX62_INS1_DRDY_M_MASK 0x8
//
#define KMX62_INS1_FFI_MASK 0x4
//
#define KMX62_INS1_AMI_MASK 0x2
//
#define KMX62_INS1_MMI_MASK 0x1
//None
#define KMX62_INS2_AXNI_MASK 0x20
//None
#define KMX62_INS2_AXPI_MASK 0x10
//None
#define KMX62_INS2_AYNI_MASK 0x8
//None
#define KMX62_INS2_AYPI_MASK 0x4
//None
#define KMX62_INS2_AZNI_MASK 0x2
//None
#define KMX62_INS2_AZPI_MASK 0x1
//None
#define KMX62_INS3_MXNI_MASK 0x20
//None
#define KMX62_INS3_MXPI_MASK 0x10
//None
#define KMX62_INS3_MYNI_MASK 0x8
//None
#define KMX62_INS3_MYPI_MASK 0x4
//None
#define KMX62_INS3_MZNI_MASK 0x2
//None
#define KMX62_INS3_MZPI_MASK 0x1
//None
#define KMX62_INC1_BFI1_MASK 0x40
//None
#define KMX62_INC1_WMI1_MASK 0x20
//None
#define KMX62_INC1_DRDY_A1_MASK 0x10
//None
#define KMX62_INC1_DRDY_M1_MASK 0x8
//None
#define KMX62_INC1_FFI1_MASK 0x4
//None
#define KMX62_INC1_AMI1_MASK 0x2
//None
#define KMX62_INC1_MMI1_MASK 0x1
//None
#define KMX62_INC2_BFI2_MASK 0x40
//None
#define KMX62_INC2_WMI2_MASK 0x20
//None
#define KMX62_INC2_DRDY_A2_MASK 0x10
//None
#define KMX62_INC2_DRDY_M2_MASK 0x8
//None
#define KMX62_INC2_FFI2_MASK 0x4
//None
#define KMX62_INC2_AMI2_MASK 0x2
//None
#define KMX62_INC2_MMI2_MASK 0x1
//Interrupt pin drive options for GPIO2
#define KMX62_INC3_IED2_MASK 0x80
//Interrupt active level control for interrupt GPIO2
#define KMX62_INC3_IEA2_MASK 0x40
//Interrupt latch control for interrupt GPIO2
#define KMX62_INC3_IEL2_MASK 0x30
//Interrupt pin drive options for GPIO1
#define KMX62_INC3_IED1_MASK 0x8
//Interrupt active level control for interrupt GPIO1
#define KMX62_INC3_IEA1_MASK 0x4
//Interrupt latch control for interrupt GPIO1
#define KMX62_INC3_IEL1_MASK 0x3
//
#define KMX62_ODCNTL_OSA_MASK 0xf
//
#define KMX62_ODCNTL_OSM_MASK 0xf0
//
#define KMX62_CNTL1_SRST_MASK 0x80
//None
#define KMX62_CNTL1_STEN_MASK 0x40
//Accelerometer and Magnetometer ST polarity. This bit is ignored when STNULL is set.
#define KMX62_CNTL1_STPOL_MASK 0x20
//enables the command test function
#define KMX62_CNTL1_COTC_MASK 0x8
//controls the operating mode of the ASIC_AOs temperature sensors. MAG_EN must also be enabled for temperature data to be converted. Output data rate is locked to the magnetometers OSM.
#define KMX62_CNTL2_TEMP_EN_MASK 0x40
//
#define KMX62_CNTL2_GSEL_MASK 0x30
//
#define KMX62_CNTL2_RES_MASK 0xc
//None
#define KMX62_CNTL2_MAG_EN_MASK 0x2
//None
#define KMX62_CNTL2_ACCEL_EN_MASK 0x1
//Sample Threshold - determines the number of data bytes that will trigger a watermark interrupt or will be saved prior to a trigger event.  The maximum number of data bytes is 384 (example - 32 samples of 3 axis of accl and 3 axis of mag by 2 bytes per axis).
#define KMX62_BUF_CTRL_1_SMT_TH_MASK 0x1ff
//
#define KMX62_BUF_CTRL_2_BUF_M_MASK 0x6
//8th bit of smt_th data
#define KMX62_BUF_CTRL_2_SMT_TH8_MASK 0x6
//None
#define KMX62_BUF_CTRL_3_BFI_EN_MASK 0x80
//None
#define KMX62_BUF_CTRL_3_BUF_AX_MASK 0x40
//None
#define KMX62_BUF_CTRL_3_BUF_AY_MASK 0x20
//None
#define KMX62_BUF_CTRL_3_BUF_AZ_MASK 0x10
//None
#define KMX62_BUF_CTRL_3_BUF_MX_MASK 0x8
//None
#define KMX62_BUF_CTRL_3_BUF_MY_MASK 0x4
//None
#define KMX62_BUF_CTRL_3_BUF_MZ_MASK 0x2
//None
#define KMX62_BUF_CTRL_3_BUF_TEMP_MASK 0x1
//"Sample Level; reports the number of data bytes that have been stored in the sample buffer.  If this register reads 0
#define KMX62_BUF_STATUS_1_SMP_LEV_MASK 0xff
//"Sample over flow; reports the number of data bytes that have been missed since the sample buffer was filled.  If this register reads 0
#define KMX62_BUF_STATUS_2_SMP_PAST_MASK 0xfffc
//"reports the status of the buffers trigger function if this mode has been selected.  When using trigger mode
#define KMX62_BUF_STATUS_2_BUF_TRIG_MASK 0x2
//
#define KMX62_BUF_STATUS_2_SMP_LEV_H_MASK 0x1
//"Sample over flow; reports the number of data bytes that have been missed since the sample buffer was filled.  If this register reads 0
#define KMX62_BUF_STATUS_3_SMP_PAST_H_MASK 0xff
