/*
 * kmx62: tri-axis accelerometer / tri-axis magnetometer
 *
 * Copyright (c) 2020 Rohm Semiconductor
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __KMX62_H__
#define __KMX62_H__

#define KMX62_G_RANGE_2G		(0x0 << 4)
#define KMX62_G_RANGE_4G		(0x1 << 4)
#define KMX62_G_RANGE_8G		(0x2 << 4)
#define KMX62_G_RANGE_16G		(0x3 << 4)
#define KMX62_G_RANGE_DONT_CARE		0xFF

#define KMX62_DEFAULT_G_RANGE		KMX62_G_RANGE_8G
#define KMX62_DEFAULT_POLL_DELAY_MS	100

#define KMX62_DEV_NAME "kmx62"

struct kmx62_platform_data {
	int (*init)(void);
	void (*release)(void);

	/*
	 * By default use, x is axis 0, y is axis 1, z is axis 2; these can be
	 * changed to account for sensor orientation within the host device.
	 */
	u8 x_map;
	u8 y_map;
	u8 z_map;

	/*
	 * Each axis can be negated to account for sensor orientation within
	 * the host device.
	 */
	u8 x_negate;
	u8 y_negate;
	u8 z_negate;

	/* Use this variable to set the default interval for data to
	 * be reported from the accelerometer and magnetometer. This
	 * value will be used during driver set up process, but can 
	 * be changed by the system during runtime via sysfs control.
	 */
	u8 poll_rate_ms;

	/* Use this variable to control the G range of
	 * the accelerometer output. Use the KMX62_G_RANGE_XX
	 * macro definition to select the desired G range.
	 */
	u8 g_range;

	int gpio_int1;
	int gpio_int2;

	/* Use this variable to choose whether or not to use
	 * DRDY hardware interrupt mode to trigger a data
	 * report event instead of using software polling. */
	bool use_drdy_int;
};

#endif
