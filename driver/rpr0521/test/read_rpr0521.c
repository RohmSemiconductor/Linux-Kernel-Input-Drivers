/*
 * ROHM RPR0521 sensor measurement readings and configuration setting.
 */

#include <linux/input.h>
#include <sys/types.h>
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
#include <pthread.h>
#define __STDC_FORMAT_MACROS

#define PROXIMITY_EVENT_TYPE ABS_DISTANCE
#define ALS_LUX_EVENT_TYPE ABS_MISC

#define INPUT_SYMLINK_PROXIMITY "/dev/input/rpr0521_proximity"
#define INPUT_SYMLINK_ALS "/dev/input/rpr0521_als"

#define SUCCESS 0
#define FAILURE -1
#define SAMPLES_NUMBER 10

int ps_fd = -1, als_fd = -1;
char input_path_ps[255];
char input_path_als[255];
int ps_delay = -1, als_delay = -1, ps_offset = -1;
bool discrete_mode = false, enable_ps = false, enable_als = false;
int ps_rd = 0, als_rd = 0;
struct input_event ps_ev, als_ev;
int proximity = 0, als_lux = 0;
bool stop_threads = false;
pthread_t ps_id, als_id;

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

int set_value(char *name, char *val, bool is_als)
{
	int write_config;
	char path[255];
	if(is_als)
		strcpy(path, input_path_als);
	else
		strcpy(path, input_path_ps);
	strcat(path, "/");
	strcat(path, name);
	
	write_config = write_to_config(path, val);
	if(write_config < 0) {
		fprintf(stderr, "writing %s to %s failed\n", val, path);
		perror("set_value failed");
	}

	return write_config;
}

int disable_measurement()
{
	if(set_value("enable", "0", true) < 0)
		exit(FAILURE);
	printf("Disabled ALS\n");

	if(set_value("enable", "0", false) < 0)
		exit(FAILURE);
	printf("Disabled proximity\n");
}

int enable_measurement_als()
{
	if(set_value("enable", "1", true) < 0)
		exit(FAILURE);
	printf("Enabled ALS\n");
}

int enable_measurement_proximity()
{
	if(set_value("enable", "1", false) < 0)
		exit(FAILURE);
	printf("Enabled proximity\n");
}

int set_ps_delay(int rgb_gain)
{
	char val[4];

	snprintf(val, 4, "%d", rgb_gain);
	return set_value("proximity_delay", val, false);
}

int set_als_delay(int ir_gain)
{
	char val[4];

	snprintf(val, 4, "%d", ir_gain);
	return set_value("als_delay", val, true);
}

int set_ps_offset(int ps_offset)
{
	char val[4];

	snprintf(val, 4, "%d", ps_offset);
	return set_value("proximity_offset", val, false);
}

void print_usage(char *program_name)
{
	printf("\nUsage for the RPR0521 sensor configuration and data readings\n");
	printf("Please run as super user or using sudo command\n");
	printf("General usage: %s [options]\n", program_name);
	printf("Options:\n");
	printf("\t-p: Enables proximity\n");
	printf("\t-a: Enables ALS\n");
	printf("\t-l {ALS delay}: sets ALS delay time\n");
	printf("\t-s {proximity delay}: sets proximity delay time\n");
	printf("\t-o {proximity offset}: sets proximity offset\n");
	printf("\t-P: prints current configuation details\n");
	printf("\t-D: discrete mode, prints the first 10 readings and exit\n");
	printf("\t-H: prints this usage only and exit\n\n");
}

int quit_program(int exit_code)
{
	stop_threads = true;
	pthread_cancel(ps_id);
	pthread_cancel(als_id);
	pthread_join(ps_id, NULL);
	pthread_join(als_id, NULL);
	close(als_fd);
	close(ps_fd);
	memset((void *)&als_ev, 0, sizeof(als_ev));
	memset((void *)&ps_ev, 0, sizeof(ps_ev));
	disable_measurement();
	exit(exit_code);
}

void signal_handler(int signal)
{
	switch (signal) {
	case SIGINT:
		printf("\nReceived SIGINT, stopping measurement\n");
		quit_program(-1);
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

	path = realpath(INPUT_SYMLINK_PROXIMITY, NULL);
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

	snprintf(input_path_ps, 255, "/sys/class/input/event%d/device", ievent);
	snprintf(input_path_als, 255, "/sys/class/input/event%d/device", ievent + 1);

	return 0;
}

int assign_configs()
{
	int err;

	if(als_delay != -1) {
		err = set_als_delay(als_delay);
		if(err < 0)
			return err;
	}
	if(ps_delay != -1) {
		err = set_ps_delay(ps_delay);
		if(err < 0)
			return err;
	}
	if(ps_offset != -1) {
		err = set_ps_offset(ps_offset);
		if(err < 0)
			return err;
	}

	return 0;
}

char *read_file(char *name, bool is_als)
{
	int fd;
	int n_bytes_read;
	static char buf[6];
	char path[255];
	
	if(is_als)
		strcpy(path, input_path_als);
	else
		strcpy(path, input_path_ps);
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
	printf("enable proximity-> %s\n", read_file("enable", false));
	printf("enable ALS-> %s\n", read_file("enable", true));
	printf("als_delay -> %s\n", read_file("als_delay", true));
	printf("proximity_delay -> %s\n", read_file("proximity_delay", false));
	printf("proximity_offset -> %s\n", read_file("proximity_offset", false));
}

int parse_configs(int argc, char **argv)
{
	int current_arg = 0;

	while ((current_arg = getopt(argc, argv, "apl:s:o:CHD")) != -1) {
		switch (current_arg) {
		case 'a':
			enable_als = true;
		break;
		case 'p':
			enable_ps = true;
		break;
		case 'l':
			als_delay = (int)strtol(optarg, NULL, 0);
		break;
		case 's':
			ps_delay = (int)strtol(optarg, NULL, 0);
		break;
		case 'o':
			ps_offset = (int)strtol(optarg, NULL, 0);
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

void *read_proximity(void *vargp)
{
	while(!stop_threads) {
		memset((void *)&ps_ev, 0, sizeof(ps_ev));
		ps_rd = read(ps_fd, &ps_ev, sizeof(struct input_event));
		if(ps_rd <= 0)
			continue;

		if (ps_ev.type == EV_ABS) {
			if (ps_ev.code == PROXIMITY_EVENT_TYPE)
				proximity = ps_ev.value;
			else
				fprintf(stderr, "unknown event type\n");
		}
	}
}

void *read_als(void *vargp)
{
	while(!stop_threads) {
		memset((void *)&als_ev, 0, sizeof(als_ev));
		als_rd = read(als_fd, &als_ev, sizeof(struct input_event));
		if( als_rd <= 0)
			continue;

		if (als_ev.type == EV_ABS) {
			if (als_ev.code == ALS_LUX_EVENT_TYPE)
				als_lux = als_ev.value;
			else
				fprintf(stderr, "unknown event type\n");
		}
	}
}

void read_and_print_data()
{
	int counter = SAMPLES_NUMBER;

	if(enable_ps)
		pthread_create(&ps_id, NULL, read_proximity, NULL);
	if (enable_als)
		pthread_create(&als_id, NULL, read_als, NULL);

	while (1) {
		// sleep time that is less than min measurement time of sensors of
		// 10ms anyways
		usleep(8000);
		if (ps_rd <= 0 && als_rd <= 0) {
			continue;
		}

		if (proximity || als_lux) {
			if(enable_als && !enable_ps)
				printf("Als_lux: 0x%x\n", als_lux);
			else if(!enable_als && enable_ps)
				printf("Proximity: 0x%x\n", proximity);
			else
				printf("proximity: 0x%x, als_lux: 0x%x\n", proximity, als_lux);

			fflush(stdout);
			proximity = 0;
			als_lux = 0;

			if(discrete_mode) {
				counter--;
				if(counter < 1) {
					quit_program(0);
				}
			}
		}
	}
}

int main(int argc, char **argv)
{
	if (argc < 2)
		print_usage(argv[0]);

	ps_fd = open(INPUT_SYMLINK_PROXIMITY, O_RDONLY);
	if (ps_fd < 0) {
		perror("Couldn't open proximity event file");
		return ps_fd;
	}

	als_fd = open(INPUT_SYMLINK_ALS, O_RDONLY);
	if (als_fd < 0) {
		perror("Couldn't open als event file");
		return als_fd;
	}

	if (resolve_paths() < 0)
		exit(EXIT_FAILURE);

	signal(SIGINT, signal_handler);
	parse_configs(argc, argv);
	disable_measurement();
	if(assign_configs())
		exit(-1);

	if(enable_als)
		if(enable_measurement_als())
			exit(-1);
	if(enable_ps)
		if(enable_measurement_proximity())
			exit(-1);

	if(!enable_als && !enable_ps) {
		printf("No sensor was selected, exiting\n");
		exit(0);
	}

	read_and_print_data();
	quit_program(-1);
}
