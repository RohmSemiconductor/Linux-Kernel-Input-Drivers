/* Simple example application on reading bh1792 output
 *
 * Modes are numbered:
 * 0 - syncronized measurement mode
 * 1 - non-syncronized measurement mode
 * 2 - single mode with green led
 * 3 - single mode with ir-led
 *
 */

#include <linux/input.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#define INPUT_SYMLINK "/dev/input/bh1792"

#define INPUT_EVENT_TYPE EV_MSC
#define INPUT_EVENT_LED MSC_SERIAL
#define INPUT_EVENT_TIME_MSB MSC_PULSELED
#define INPUT_EVENT_TIME_LSB MSC_SCAN

int fd = -1;
char input_path[255];
int operating_mode = -1;

int write_to_config(char *config_path, char *val)
{
	int fd_config;
	int ret;

	fd_config = open(config_path, O_WRONLY);
	if (fd_config < 0)
		return fd_config;

	ret = write(fd_config, val, strlen(val));
	if (ret < 0)
		perror("write failed");

	close(fd_config);

	if (ret != strlen(val))
		return -1;

	return 0;
}

int set_value(char *name, char *val)
{
	int write_config;
	char path[255];

	strcpy(path, input_path);
	strcat(path, "/");
	strcat(path, name);
	write_config = write_to_config(path, val);
	if (write_config) {
		fprintf(stderr, "writing %s to %s failed\n", val, name);
		perror("set_value failed");
	}
	return write_config;
}

int disable_measurement(void)
{
	return set_value("enable", "0");
}

int enable_measurement(void)
{
	return set_value("enable", "1");
}

int set_operating_mode(void)
{
	char val[2];

	snprintf(val, 2, "%d", operating_mode);
	return set_value("operating_mode", val);
}

int set_freq(int freq)
{
	char val[5];

	snprintf(val, 5, "%d", freq);
	return set_value("freq", val);
}

int set_led_current(int led, int current)
{
	char val[3];
	char ledname[13];

	snprintf(val, 3, "%d", current);
	snprintf(ledname, 13, "led%d_current", led);
	return set_value(ledname, val);
}

int set_threshold(int threshold)
{
	char val[6];

	snprintf(val, 6, "%d", threshold);
	return set_value("threshold", val);
}

void signal_handler(int signal)
{
	switch (signal) {
	case SIGALRM:
		printf("starting measurement\n");
		set_operating_mode();
		enable_measurement();
		break;
	case SIGINT:
		printf("received SIGINT, stopping measurement\n");
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

void usage(char *program_name)
{
	printf("\nusage: %s -m operation_mode [opts]\n", program_name);
	printf("usage: %s -h\tprint this help\n", program_name);
	printf("\n\tFollowing operation modes are supported:\n");
	printf("\t\t0 - syncronized measurement using Green led, frequency set by option -f (32, 64, 128, 256 or 1024)\n");
	printf("\t\t1 - non-synchronized measurement using IR led, measurement frequency 4 HZ, interrupt threshold set by option -t (0-0xffff)\n");
	printf("\t\t2 - single measurement mode using Green led, continuous operation by option -C\n");
	printf("\t\t3 - single measurement mode using IR led, continuous operation by option -C\n");
	printf("\n\tfollowing opts are supported in all modes:\n");
	printf("\t\t-c led current (0 - 63)\n");
	printf("\n\tExample, synchronized mode with 256 HZ: %s -m 0 -f 256\n", program_name);
}

int main(int argc, char **argv)
{
	int rd;
	struct input_event ev;
	int i = 0;
	uint32_t msb, lsb;
	uint64_t timestamp = 0;
	uint64_t last = 0;
	int f = -1;
	int t = -1;
	int c;
	int led_current = -1;
	int single_continuous = 0;

	if (argc < 2) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	fd = open(INPUT_SYMLINK, O_RDONLY);
	if (fd < 0) {
		perror("open");
		return fd;
	}

	if (resolve_paths() < 0)
		exit(EXIT_FAILURE);

	signal(SIGINT, signal_handler);

	while ((c = getopt(argc, argv, "m:f:t:c:Ch")) != -1) {
		switch (c) {
		case 'm':
			operating_mode = (int)strtol(optarg, NULL, 0);
		break;
		case 'f':
			f = (int)strtol(optarg, NULL, 0);
		break;
		case 't':
			t = (int)strtol(optarg, NULL, 0);
		break;
		case 'c':
			led_current = (int)strtol(optarg, NULL, 0);
		break;
		case 'C':
			single_continuous = 1;
		break;
		case 'h':
			usage(argv[0]);
			exit(EXIT_SUCCESS);
		break;
		}
	}

	if (disable_measurement())
		exit(EXIT_FAILURE);

	if (f != -1) {
		if (set_freq(f) < 0)
			exit(EXIT_FAILURE);
	}

	if (t != -1) {
		if (set_threshold(t) < 0)
			exit(EXIT_FAILURE);
	}


	if (led_current != -1) {
		if (operating_mode == 0 || operating_mode == 2)
			set_led_current(1, led_current);
		if (operating_mode == 1 || operating_mode == 3)
			set_led_current(2, led_current);
	}

	/* In case of synchronized and non-synchronized measurement modes start
	 * measurement right away. In single mode measurement is stated by timer
	 * to ensure reader to be ready when first sample is received
	 */

	switch (operating_mode) {
	case 0:
	case 1:
		set_operating_mode();
		enable_measurement();
		break;
	case 2:
	case 3:
		signal(SIGALRM, signal_handler);
		alarm(1);

	break;
	default:
		printf("illegal value for operating mode\n");
		usage(argv[0]);
		return -1;
	}

	while (1) {
		memset((void *)&ev, 0, sizeof(ev));
		rd = read(fd, &ev, sizeof(struct input_event));
		if (rd <= 0)
			continue;

		if (ev.type == EV_SYN && operating_mode == 0) {
			if (i < 32) {
				printf("lost %d packets\n", 32 - i);
				fflush(stdout);
			}
			i = 0;
			timestamp = ((uint64_t)msb << 32) | lsb;
			printf("timestamp %" PRIu64 " diff %" PRIu64 "\n", timestamp, timestamp - last);
			last = timestamp;
			fflush(stdout);
		}

		if (ev.type == INPUT_EVENT_TYPE) {
			if (ev.code == INPUT_EVENT_LED) {
				printf("%02d:\t%d,\t%d\n", i++, ev.value >> 16,
					   ev.value & 0xFFFF);
				fflush(stdout);

				if (operating_mode == 2 || operating_mode == 3) {
					if (single_continuous == 1)
						enable_measurement();
					else
						raise(SIGINT);
				}
			}

			if (ev.code == INPUT_EVENT_TIME_MSB)
				msb = ev.value;
			if (ev.code == INPUT_EVENT_TIME_LSB)
				lsb = ev.value;
		}
	}
}
