/*
 * ROHM BH1749 sensor measurement readings and configuration setting.
 *
 * Copyright(C) 2018 Rohm Semiconductor
 * SPDX-License-Identifier: MIT
 */

#include <linux/input.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <inttypes.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#define __STDC_FORMAT_MACROS

#define MEAS_TYPE_RGB_RED MSC_SERIAL
#define MEAS_TYPE_RGB_GREEN MSC_PULSELED
#define MEAS_TYPE_RGB_BLUE MSC_GESTURE
#define MEAS_TYPE_RGB_IR MSC_SCAN
#define MEAS_TYPE_RGB_GREEN2 MSC_TIMESTAMP

#define INPUT_SYMLINK "/dev/input/bh1749"

#define SUCCESS 0
#define FAILURE -1
#define SAMPLES_NUMBER 10

int fd = -1;
char input_path[255];
int rgb_gain = -1, ir_gain = -1, threshold_h = -1,
		threshold_l = -1, meas_time = -1, int_judgement = -1;
char int_source = ' ';
bool discrete_mode = false;

int write_to_config(char *config_path, char *val)
{
	int fd_config;
	int ret;
	fd_config = open(config_path, O_WRONLY);
	if (fd_config < 0)
		return fd_config;

	ret = write(fd_config, val, strlen(val));
	close(fd_config);

	return ret;
}

int set_value(char *name, char *val)
{
	int write_config;
	char path[255];
	strcpy(path, input_path);
	strcat(path, "/");
	strcat(path, name);
	write_config = write_to_config(path, val);
	if(write_config < 0) {
		fprintf(stderr, "writing %s to %s failed\n", val, name);
		perror("set_value failed");
	}
	return write_config;
}

int disable_measurement()
{
	if(set_value("enable", "0") < 0)
		exit(FAILURE);
}

int enable_measurement()
{
	if(set_value("enable", "1") < 0)
		exit(FAILURE);
	printf("Starting measurement:\n");
}

int set_rgb_gain(int rgb_gain)
{
	char val[3];

	snprintf(val, 3, "%d", rgb_gain);
	return set_value("rgb_gain", val);
}

int set_ir_gain(int ir_gain)
{
	char val[3];

	snprintf(val, 3, "%d", ir_gain);
	return set_value("ir_gain", val);
}

void print_usage(char *program_name)
{
	printf("\nUsage for the BH1749 sensor configuration and data readings\n");
	printf("Please run as super user or using sudo command\n");
	printf("General usage: %s [options]\n", program_name);
	printf("Options:\n");
	printf("\t-m {measurement time}: sets measurement time, accepted values are 35, 120 or 240 ms\n");
	printf("\t-r {rbg gain}: sets RGB gain value, accepted values are 1 or 32\n");
	printf("\t-i {ir gain}: sets IR gain value, accepted values are 1 or 32\n");
	printf("\t-h {high threshold}: sets high value of threshold, values from 0-65535\n");
	printf("\t-l {low threshold}: sets low value of threshold, values from 0-65535\n");
	printf("\t-j {interrupt judgement}: sets interrupt judgement value, accepted values are 0, 1, 4 or 8\n");
	printf("\t-s {interrupt source}: sets the interrupt source, accepted values are r,g or b\n");
	printf("\t-P: prints current configuation details\n");
	printf("\t-D: discrete mode, prints the first 10 readings and exit\n");
	printf("\t-H: prints this usage only and exit\n\n");
}

int set_meas_time(int meas_time)
{
	char val[4];

	snprintf(val, 4, "%d", meas_time);
	return set_value("measurement_time", val);
}

int set_threshold_high(int threshold)
{
	char val[6];

	snprintf(val, 6, "%d", threshold);
	return set_value("int_threshold_high", val);
}

int set_threshold_low(int threshold)
{
	char val[6];

	snprintf(val, 6, "%d", threshold);
	return set_value("int_threshold_low", val);
}

int set_int_source(char source)
{
	char val[2];

	snprintf(val, 2, "%c", source);
	return set_value("int_source", val);
}

int set_int_judgement(int int_judgement)
{
	char val[2];

	snprintf(val, 2, "%d", int_judgement);
	return set_value("int_judgement", val);
}

void signal_handler(int signal)
{
	switch (signal) {
	case SIGINT:
		printf("\nReceived SIGINT, stopping measurement\n");
		disable_measurement();
		close(fd);
		exit(1);
		break;
	default:
	printf("received unknown signal");
	}
}

int resolve_paths(void)
{
	char *path = NULL;
	char *event = NULL;
	char *endptr = NULL;
	int ievent = 0;

	path = realpath(INPUT_SYMLINK, NULL);
	if (path == NULL) {
		perror("unable to resolve event device path\n");
		return -1;
	}

	event = strstr(path, "event");
	if (event == NULL) {
		fprintf(stderr, "couldn't find proper event path\n");
		return -1;
	}
	event += 5; /* jump over "event" */

	errno = 0;
	ievent = strtol(event, &endptr, 10);

	free(path);

	if ((errno == ERANGE && (ievent == LONG_MAX || ievent == LONG_MIN)) || (errno != 0)) {
		perror("strtol");
		return -1;
	}

	if (endptr == event)
		return -1;

	snprintf(input_path, 255, "/sys/class/input/event%d/device", ievent);

	return 0;
}

void assign_configs()
{
	if(rgb_gain != -1)
		set_rgb_gain(rgb_gain);
	if(ir_gain != -1)
		set_ir_gain(ir_gain);
	if(meas_time != -1)
		set_meas_time(meas_time);
	if(int_judgement != -1)
		set_int_judgement(int_judgement);
	if(threshold_h != -1)
		set_threshold_high(threshold_h);
	if(threshold_l != -1)
		set_threshold_low(threshold_l);
	if(int_source != ' ')
		set_int_source(int_source);
}

char *read_file(char *name)
{
	int fd;
	int n_bytes_read;
	static char buf[6];
	char path[255];

	strcpy(path, input_path);
	strcat(path, "/");
	strcat(path, name);

	fd = open(path, O_RDONLY);
	n_bytes_read = read(fd, buf, 6);
	buf[n_bytes_read - 1] = '\0';
	if(n_bytes_read < 0) {
		fprintf(stderr, "Reading from %s failed\n", path);
		perror("read_file failed");
	}

	close(fd);
	return buf;
}

void print_current_config()
{
	printf("Current configuration:\n");
	printf("enable -> %s\n", read_file("enable"));
	printf("measurement_time -> %s\n", read_file("measurement_time"));
	printf("int_judgement -> %s\n", read_file("int_judgement"));
	printf("rgb_gain -> %s\n", read_file("rgb_gain"));
	printf("ir_gain -> %s\n", read_file("ir_gain"));
	printf("int_source -> %s\n", read_file("int_source"));
	printf("int_threshold_high -> %s\n", read_file("int_threshold_high"));
	printf("int_threshold_low -> %s\n", read_file("int_threshold_low"));
}

int parse_configs(int argc, char **argv)
{
	int current_arg;

	while ((current_arg = getopt(argc, argv, "r:i:m:h:l:j:s:CHD")) != -1) {
		switch (current_arg) {
		case 'r':
			rgb_gain = (int)strtol(optarg, NULL, 0);
			if(rgb_gain != 1 && rgb_gain != 32) {
				printf("rgb_gain Wrong value\n");
				exit(FAILURE);
			}
		break;
		case 'i':
			ir_gain = (int)strtol(optarg, NULL, 0);
			if(ir_gain != 1 && ir_gain != 32) {
				printf("ir_gain Wrong value\n");
				exit(FAILURE);
			}
		break;
		case 'm':
			meas_time = (int)strtol(optarg, NULL, 0);
			if(meas_time != 35 && meas_time != 120 && meas_time != 240) {
				printf("meas_time Wrong value\n");
				exit(FAILURE);
			}
		break;
		case 'h':
			threshold_h = (int)strtol(optarg, NULL, 0);
			if(threshold_h < 0 || threshold_h > 65535) {
				printf("threshold_h Wrong value\n");
				exit(FAILURE);
			}

		break;
		case 'l':
			threshold_l = (int)strtol(optarg, NULL, 0);
			if(threshold_l < 0 || threshold_l > 65535) {
				printf("threshold_l Wrong value\n");
				exit(FAILURE);
			}

		break;
		case 'j':
			int_judgement = (int)strtol(optarg, NULL, 0);
			if(int_judgement != 0 && int_judgement != 1 && 
				int_judgement != 4 && int_judgement != 8) {
				printf("int_judgement Wrong value\n");
				exit(FAILURE);
			}
		break;
		case 's':
			int_source = optarg[0];
			if(int_source != 'r' && int_source != 'g' && int_source != 'b') {
				printf("int_source Wrong value\n");
				exit(FAILURE);
			}
		break;
		case 'C':
			print_current_config();
			exit(SUCCESS);
		break;
		case 'D':
			discrete_mode = true;
		break;
		case 'H':
			print_usage(argv[0]);
			exit(SUCCESS);
		break;
		}
	}
}

void read_and_print_data()
{
	int rd;
	struct input_event ev;
	uint16_t red = 0, green = 0, blue = 0, ir = 0, green2 = 0;
	int counter = SAMPLES_NUMBER;

	while (1) {
		memset((void *)&ev, 0, sizeof(ev));
		rd = read(fd, &ev, sizeof(struct input_event));
		if (rd <= 0)
			continue;

		if (ev.type == EV_SYN) {
			printf("r:%d, g:%d, b:%d, g2:%d, ir:%d\n",
				   red, green, blue, green2, ir);
			fflush(stdout);
			if(discrete_mode) {
				counter--;
				if(counter < 1) {
					disable_measurement();
					break;
				}
			}
		}

		if (ev.type == EV_MSC) {
			switch (ev.code) {
				case MEAS_TYPE_RGB_RED:
					red = ev.value;
					break;
				case MEAS_TYPE_RGB_GREEN:
					green = ev.value;
					break;
				case MEAS_TYPE_RGB_BLUE:
					blue = ev.value;
					break;
				case MEAS_TYPE_RGB_IR:
					ir = ev.value;
					break;
				case MEAS_TYPE_RGB_GREEN2:
					green2 = ev.value;
					break;
				default:
					fprintf(stderr, "unknown event type\n");
					break;
			}
		}
	}
}

int main(int argc, char **argv)
{
	if (argc < 2)
		print_usage(argv[0]);

	fd = open(INPUT_SYMLINK, O_RDONLY);
	if (fd < 0) {
		perror("Couldn't open event file");
		return fd;
	}

	if (resolve_paths() < 0)
		exit(EXIT_FAILURE);

	signal(SIGINT, signal_handler);
	parse_configs(argc, argv);
	disable_measurement();
	assign_configs();
	enable_measurement();
	read_and_print_data();
}
