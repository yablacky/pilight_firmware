/**
 * Pulsi. Generate very accurate short time pulses on a Raspberry-Pi GPIO pin.
 * @author yablacky <schwarz.ware@gmx.de>
 * @date 2017-01-21
 * @abstract Intention was to generate pulse trains to test pilight
 * receiver and firmware filter. Pulses are specified like for
 * pilight-send for the "raw" protocol. pilight-send requires
 * the pilight-daemon to run while pulsi does not.
 */

#define VERSION			"0.3"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>		// basename()
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>

#ifndef countof
#define countof(a)		(sizeof(a) / sizeof(a[0]))
#endif

/*******************************************************
 * Raspberry Pi specific things.
 */

// Raspberry model and board

#define BCM2708_PERI_BASE	0x20000000
#define BCM2709_PERI_BASE	0x3F000000

static unsigned long		BCM27XX_PERI_BASE = BCM2709_PERI_BASE;
#define GPIO_BASE		(BCM27XX_PERI_BASE + 0x200000) /* GPIO controller */
#define GPIO_PINS_COUNT		26
#define PAGE_SIZE		(4*1024)
#define BLOCK_SIZE		(4*1024)

// Methods for GPIO pin access

static volatile unsigned long *gpio;

#define _GPIO_DIR	(gpio)		// 1 set output, 0 set input (almost, see macros)
#define _GPIO_SET	(gpio+7)	// Sets   bits which are 1 ignores bits which are 0
#define _GPIO_CLR	(gpio+10)	// Clears bits which are 1 ignores bits which are 0
#define _GPIO_GET	(gpio+13)	// Delivers bits which are 1
#define _GPIO_PULL	(gpio+37)	// Pull up/pull down
#define _GPIO_PULLCLK0	(gpio+38)	// Pull up/pull down clock

/**
 * Set direction of GPIO pin.
 * @param int p The GPIO number of the pin.
 * @param int as_output GPIO_DIR_INPUT or GPIO_DIR_OUTPUT (zero(false) for input, non-zero(true) for output).
 */
#define GPIO_DIR_INPUT	0
#define GPIO_DIR_OUTPUT 1
#define GPIO_DIR_SET(p, as_output) do { _GPIO_DIR[p / 10] &= ~(7<<((p % 10) * 3));	\
			 if (as_output) _GPIO_DIR[p / 10] |=  (1<<((p % 10) * 3)); } while (0)

/**
 * Set and query state of a GPIO pin.
 * @param int p The GPIO number of the pin.
 */
#define GPIO_SET(p)	(void)(   _GPIO_SET[0] = (1 << p)	)   // Set pin p to 1, H state.
#define GPIO_CLR(p)	(void)(   _GPIO_CLR[0] = (1 << p)	)   // Set pin p to 0, L state.
#define GPIO_GET(p)	      (  (_GPIO_GET[0] & (1 << p))!=0	)   // Ask pin p state, returns 0 or 1.

/**
 * Determine Raspberry model and board revision.
 * @param int* pModelNumber If not NULL set to 1,2,3 on return.
 * @param long* pBCMPeriBase Base address of board pheriphery.
 * @return int Board revision or -1 on error if no known raspi detected.
 */
int RaspiGetRevision(int *pModelNumber, unsigned long *pBCMPeriBase)
{
    FILE *fp;
    char line[120], revision[120], hardware[120];
    revision[0] = hardware[0] = 0;

    if ((fp = fopen("/proc/cpuinfo", "r")) == NULL) {
	perror("/proc/cpuinfo");
	return -1;
    }

    while(fgets(line, sizeof(line), fp) != NULL) {
	sscanf(line, "Hardware : %[a-zA-Z0-9 ./()]%*[\n\r]", hardware);
	sscanf(line, "Revision : %[a-zA-Z0-9 ./()]%*[\n\r]", revision);
    }

    fclose(fp);

    if (strstr(hardware, "BCM2708") != NULL) {
	if (pBCMPeriBase)
	    *pBCMPeriBase = BCM2708_PERI_BASE;	
	if (pModelNumber)
	    *pModelNumber = 1;
	if (strlen(revision) >= 4) {
	    char *rev = revision + strlen(revision) - 4;

	    if (strcmp(rev, "0002") == 0 || strcmp(rev, "0003") == 0)
		return 1;

	    for (; *rev; rev++)
		if (isdigit(*rev))
		    return 2;	// ok if digit seen.
	}
	fprintf(stderr, "cpuinfo: unknown 'Revision': '%s'", revision);
	return -1;
    }

    if (strstr(hardware, "BCM2709") != NULL) {
	if (pBCMPeriBase)
	    *pBCMPeriBase = BCM2709_PERI_BASE;	
	int len = strlen(revision);
	if (len < 4) {
	    fprintf(stderr, "cpuinfo: unknown 'Revision': '%s'", revision);
	    return -1;
	}
	if (pModelNumber) {
	    if (strcmp(revision + len - 4, "1041") == 0)
		*pModelNumber = 2; else
	    if (strcmp(revision + len - 4, "2082") == 0)
		*pModelNumber = 3;
	    else {
		fprintf(stderr, "cpuinfo: unknown 'Revision': '%s'", revision);
		return -1;
	    }
	}
	return 2;
    }

    fprintf(stderr, "cpuinfo: unknown 'Hardware': '%s'", hardware);
    return -1;
}


/**
 * Initialize accessing gpio pins. Setup global gpio pointer.
 * @return int Raspi model number (>=0) or -1 on error (message has been given).
 */
int init_gpio()
{
    int  mem_fd, raspi_model;

    if (sizeof(*gpio) != 4) {
	fprintf(stderr, "not a 32 bit type");
	return -1;
    }
    if (RaspiGetRevision(&raspi_model, &BCM27XX_PERI_BASE) < 0)
	return -1;

    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
	perror("can't open /dev/mem");
	return -1;
    }
    void *gpio_map = mmap(
	NULL,             //Any adddress in our space will do
	BLOCK_SIZE,       //Map length
	PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
	MAP_SHARED,       //Shared with other processes
	mem_fd,           //File to map
	GPIO_BASE         //Offset to GPIO peripheral
    );

    if (gpio_map == MAP_FAILED) {
	perror("mmap failed");

	close(mem_fd);	// dont close() before checking mmap() errno.
	return -1;
    }

    close(mem_fd); //No need to keep mem_fd open after mmap

    if (gpio)
	exit_gpio();

    gpio = gpio_map;
    return raspi_model > 0 ? raspi_model : 0;
}

/**
 * Shut down accessing of gpio pins. Uses and clears global gpio pointer.
 */
int exit_gpio()
{
    if (munmap((void*)gpio, BLOCK_SIZE) < 0)
	perror("munmap failed");
    gpio = NULL;
    return 0;
}

/*******************************************************
 * General purpose functions.
 */

/**
 * Parse a string as micro seconds value with optional unit.
 * @param char* str The string to parse.
 * @param char** pMore Optionally return unknown unit here.
 * @param int* pOverflow Optionally return info if value+unit is still a useconds_t value.
 * @return The time value in micro seconds.
 */
useconds_t strtousec(const char *str, char **pMore, int *pOverflow)
{
    char *more = "";
    if (pMore) *pMore = more;
    useconds_t f = 1, t, t1;
    t1 = t = strtoul(str, &more, 0);
    if (strcmp(more, "s") == 0)  f = 1000000; else 
    if (strcmp(more, "ms") == 0) f = 1000; else 
    if (strcmp(more, "us") == 0) f = 1; else 
    if (strcmp(more, "min") == 0) f = 60000000; else
    if (pMore) *pMore = more;
    t *= f;
    if (pOverflow)
	*pOverflow = t / f != t1;
    return t;
}

/**
 * Sleep like usleep() but accept sleep times above 1 second and
 * adjust the effective sleep duration by a given calibration value.
 * @param useconds_t duration_us Time to sleep in micro seconds.
 * @param inf calibration_us The effective sleep time is duration_us minus calibration_us.
 * @return int Success of sleep: 0=OK, -1: error, see errno.
 */
static inline int usleep_calibrated(useconds_t duration_us, int calibration_us)
{
    static const useconds_t one_second_us = 1000000;
    if ((useconds_t)calibration_us >= duration_us && calibration_us > 0)
	return 0;

    duration_us -= calibration_us;

    // usleep() may have limit of 1 second and will fail then without sleeping.
    if (duration_us < one_second_us)
	return usleep(duration_us);

    if (sleep(duration_us / one_second_us) < 0)
	return -1;
    return usleep(duration_us % one_second_us);
}

/*******************************************************
 * Pulsi specific code.
 */

const char *progname = "pulsi";
int verbose = 0;

/**
 * Sleep calibrated with "calibration" value of current scope.
 * @param useconds_t duration Time to sleep in micro seconds.
 * @return int Success of sleep: 0=OK, -1: error, see errno.
 */
#define pulsi_usleep_calibrated(duration) usleep_calibrated((duration), (calibration))

/**
 * Parse a pulse train specification, determine number of pulses and
 * optionally return the pulse durations as array of useconds_t.
 * @param int pulsec Number of values in pulsev.
 * @param char** pulsev Array of string with pulse specifications.
 * @param useconds_t** pUsecPulses If not NULL (and the function returns >=0),
 * it is set to a mallocated buffer of useconds_t values. Caller must free() it.
 * @return int Number of pulses in pulse spec. -1 on error (a message is print on error).
 */
int pulsi_parse_pulse_spec(int pulsec, char * const * pulsev, useconds_t **pUsecPulses)
{
    useconds_t *usecPulses = NULL;
    int ox; // output pulse count
    do {
	// 1st time to determine number of pulses to check them.
	// 2nd time if caller want the pulses.
	ox = 0;
	int ix; // input pulse index
	for (ix = 0; ix < pulsec; ix++) {
	    char *pulses = strdup(pulsev[ix]);	// Copy because strtok() modifies the string.
	    char *duration = strtok(pulses, " \t\n");
	    for (; duration != NULL; duration = strtok(NULL, " \t\n")) {
		char *more = ""; int overflow = 0;
		useconds_t t = strtousec(duration, &more, &overflow);
		if (more && *more) {
		    fprintf(stderr, "%s: pulse#%d: unit '%s' is not one of (us, ms, s, min)\n",
				progname, ix+1, more);
		    free(usecPulses);
		    return -1;
		}
		if (overflow) {
		    fprintf(stderr, "%s: pulse#%d: the duration '%s' is too long for this tool, sorry.\n",
				progname, ix+1, duration);
		    free(usecPulses);
		    return -1;
		}
		if (usecPulses)
		    usecPulses[ox] = t;
		ox++;
	    }
	    free(pulses);
	}
    } while (ox != 0	// have any pulses
	&& pUsecPulses	// caller want pulses
	&& !usecPulses	// no pulse buffer yet
	&& (usecPulses = malloc(ox * sizeof(*usecPulses)))  // could allocate pulse buffer
    );
    if (pUsecPulses)
	*pUsecPulses = usecPulses;
    return ox;
}


static char* predefined_pulse_train[] =
{
    "testsignal"
    ,	"Testsignal like pilight firmware filter method 2"
    ,	"2000 2000 2000 2000 "
	"1000 1000 1000 1000 "
	" 500  500  500  500 "
	" 250  250  250  250 "
	" 120  120  120  120 "
	"  60   60   60   60 "
	"6000 6000",

    "signature"
    ,	"A faked pilight firmware signature (version 9, lpf 10010, hpf 20020)"
    ,	// header
	"225  900 "

	// payload
#define BIT_0	"225 225 225 675 "
#define BIT_1	"225 675 225 255 "

	BIT_1 BIT_0 BIT_0 BIT_1	// 16 bits version
	BIT_0 BIT_0 BIT_0 BIT_0 // note lsb first!
	BIT_0 BIT_0 BIT_0 BIT_0 // Value here: 9
	BIT_0 BIT_0 BIT_0 BIT_0 // Shows as initial filter

	BIT_1 BIT_0 BIT_0 BIT_1	// 16 bits MIN_PULSELEN
	BIT_0 BIT_1 BIT_1 BIT_1 // note lsb first!
	BIT_1 BIT_1 BIT_0 BIT_0 // Value here: 1001.
	BIT_0 BIT_0 BIT_0 BIT_0 // Shows as lpf: 10010

	BIT_0 BIT_1 BIT_0 BIT_0	// 16 bits MAX_PULSELEN
	BIT_1 BIT_0 BIT_1 BIT_1 // note lsb first!
	BIT_1 BIT_1 BIT_1 BIT_0 // Value here: 2002
	BIT_0 BIT_0 BIT_0 BIT_0 // Shows as hpf: 20020

	BIT_1 BIT_0 BIT_1 BIT_0 // 4 bit checksum (5).

	// trailer
	"225 7650 225 7650 "
	,
#undef BIT_0
#undef BIT_1

// Add more predefined pulse trains before this commment.
    NULL
    , NULL
    , NULL
};

/**
 * Print the program help text.
 */
void print_help()
{
    printf(
	"usage: %s [OPTIONS] pulse_duration...\n"
	" Let a GPIO pin blink by driving that pin to H and L signals after\n"
	" individual periods of time; the so-called pulse durations.\n"
	" Typically this tool is used to generate very short pulse durations,\n"
	" down to about 60µs with suitable accuracy. The longest possible\n"
	" pulse is about 71 minutes. A series of pulses is called a pulse train.\n"
	"Possible options are:\n"
	"  --gpio-pin=number, -g\n"
	"       Generate H/L pulses on that GPIO pin (0 .. %d), default pin 9.\n"
	"  --prio-rise=amount, -p\n"
	"       Rise process priority (make faster) by amount (default 17)\n"
	"  --repeat=count, -r\n"
	"       Generate the pulse train count times.\n"
	"  --do-all-wait, -A\n"
	"       Do all waiting, even the very last one before exit.\n"
	"       By default the last wait is skipped if the GPIO pin\n"
	"       state will not be changed after waiting (e.g. is already L).\n"
	"  --simple-mode, -S\n"
	"       Use simple unchecked timing mode (what pilight-send does).\n"
	"  --calibrate=time, -C\n"
	"       When sleeping to generate a pulse, effectively sleep the given\n"
	"       time (in µs) shorter each time. Default 0. Applies to simple-\n"
	"       and non-simple-mode.\n"
	"  --verbose, -v\n"
	"       Tell what is going on. More -v tells more.\n"
	"       NOTE: verbose output degrades accucary of\n"
	"             generated pulse durations!\n"
	"  --help, -h, -?\n"
	"        Show this help.\n"
	"Pulse duration values are integers with optional unit:\n"
	"    us  Micro seconds (default unit)\n"
	"    ms  Milli seconds\n"
	"    s   Seconds\n"
	"    min Minutes\n"
	, progname, GPIO_PINS_COUNT - 1);

    if (predefined_pulse_train[0]) {
	printf(
	    "If only one pulse_duration is given, it can be a name of a\n"
	    "predefined pulse train:\n");
	int ii;
	for (ii = 0; predefined_pulse_train[ii]; ii += 3) {
	    printf("    %-16s - %s.\n", predefined_pulse_train[ii+0], predefined_pulse_train[ii+1]);
	}
    }

    printf(
	"Environment variables can be used to define different\n"
	"default values for some options:\n"
	"    PULSI_GPIO_PIN_NUMBER=number\n"
	"    PULSI_PRIO_RISE=prio_rise\n"
	"    PULSI_CALIBRATION=duration_in_microseconds\n"
	"This is version %s as of %s %s.\n"
	, VERSION, __DATE__, __TIME__);
}

/**
 * Guess what!
 */
int main(int argc, char **argv)
{
    progname = basename(argv[0]);
    int pin = 9, repeat = 1, prio_rise = 17, do_last_sleep = 0, simple_mode = 0;
    int calibration = 0;
    { const char *p = getenv("PULSI_GPIO_PIN_NUMBER"); if (p) pin = atoi(p); }
    { const char *p = getenv("PULSI_PRIO_RISE"); if (p) prio_rise = atoi(p); }
    { const char *p = getenv("PULSI_CALIBRATION"); if (p) calibration = atoi(p); }

    static struct option long_options[] = {
	{ "verbose",	no_argument,	    NULL,   'v'},
	{ "repeat",	required_argument,  NULL,   'r'},
	{ "gpio-pin",	required_argument,  NULL,   'g'},
	{ "prio-rise",	required_argument,  NULL,   'p'},
	{ "calibration",required_argument,  NULL,   'C'},
	{ "do-all-wait",no_argument,	    NULL,   'A'},
	{ "simple-mode",no_argument,	    NULL,   'S'},
	{ "help",	no_argument,	    NULL,   'h'},
	{ NULL,		0,		    NULL,   0  }
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "vhASC:r:g:p:", long_options, NULL)) != -1) {
	char *more = "";
	switch (opt) {
	case 'v': verbose++; break;
	case 'r': repeat      = strtol(optarg, &more, 0); break;
	case 'g': pin         = strtol(optarg, &more, 0); break;
	case 'p': prio_rise   = strtol(optarg, &more, 0); break;
	case 'C': calibration = strtol(optarg, &more, 0); break;
	case 'A': do_last_sleep = 1; break;
	case 'S': simple_mode=1; break;
	case 'h': print_help(progname);
	    // fall thru
	default:
	    exit(EXIT_FAILURE);
	}
	if (*more) {
		fprintf(stderr, "%s: -%c requires numeric argument; found %s\n", progname, opt, more);
		exit(EXIT_FAILURE);
	}
    }
    if (pin < 0 || pin >= GPIO_PINS_COUNT) {
	fprintf(stderr, "%s: pin number %d not in range 0..%d\n", progname, pin, GPIO_PINS_COUNT - 1);
	exit(EXIT_FAILURE);
    }

    // Handle process performance issues

    if (prio_rise) {
	errno = 0;
	int have_prio = getpriority(PRIO_PROCESS, 0);
	if (errno != 0) {
	    perror("Get process priority");
	    exit(EXIT_FAILURE);
	}
	int want_prio = have_prio - prio_rise;
	if (verbose) printf("Setting %s process priority (was %d, set %d).\n",
		    prio_rise > 0 ? "higher" : "lower", have_prio, want_prio);
	if (setpriority(PRIO_PROCESS, 0, want_prio) < 0) {
	    perror("Warning: Set process priority");
	}
    }
    if (mlockall(MCL_FUTURE) < 0) {
	perror("Warning: Keep all program code and data in memory");
    }

    // Initialize the GPIO interface.
    {
	int model_number = init_gpio();
	if (model_number < 0)
	    exit(EXIT_FAILURE);
	if (verbose) printf("Detected raspberry pi model %d.\n", model_number);
    }

    GPIO_DIR_SET(pin, GPIO_DIR_OUTPUT);

    // Get all the pulse durations as micro second values:
 
    int pulsec = -1;
    useconds_t *pulsev = NULL, pulse_duration_min, pulse_duration_max;
    if (argc - optind == 1 && !isdigit(argv[optind][0])) {
	int ii = 0;
	for (ii = 0; predefined_pulse_train[ii]; ii += 3)
	    if (strcasecmp(argv[optind], predefined_pulse_train[ii]) == 0)
		break;
	if (!predefined_pulse_train[ii]) {
	    fprintf(stderr, "%s: No such predefined pulse train: '%s'\n", argv[optind]);
	    pulsec = -1;
	} else {
	    if (verbose) printf("Using pulse train '%s' (%s)\n",
			    predefined_pulse_train[ii], predefined_pulse_train[ii+1]);
	    pulsec = pulsi_parse_pulse_spec(1, &predefined_pulse_train[ii+2], &pulsev);
	}
    } else {
	pulsec = pulsi_parse_pulse_spec(argc - optind, argv + optind, &pulsev);
    }

    if (pulsec < 0)
	exit(EXIT_FAILURE); // error message already printed.
    if (pulsec == 0) {
	fprintf(stderr, "%s: No pulses specified. Use -h for help.\n", progname);
	exit(EXIT_FAILURE);
    }
    if (pulsev == NULL) {
	fprintf(stderr, "%s: Out of memory. Too much pulses (%d) specified?\n", progname, pulsec);
	exit(EXIT_FAILURE);
    }
    // from here on, free(pulsev) must be called on return.

    // Determine shortest and longest pulse duration for logging purposes.
    {
	int ii = pulsec;
	pulse_duration_min = pulse_duration_max = pulsev[--ii];
	while (--ii >= 0) {
	    useconds_t t = pulsev[ii];
	    if (t < pulse_duration_min) pulse_duration_min = t;
	    if (t > pulse_duration_max) pulse_duration_max = t;
	}
    }

    if (verbose) printf("Pulses found: %d, shortest %u µs, longest %u µs. Repeat %d times.\n",
			pulsec, pulse_duration_min, pulse_duration_max, repeat);
    if (repeat > 1 && (pulsec & 1)) {
	printf("Warning: Number of pulses (%d) is odd; each repeat generates opposite L/H signals.\n", pulsec);
    }

    // Variables for automatic timing correction in non-simple mode.

    useconds_t mean_loop_duration = 30;	    // This will be adjusted. Found initial value by experiments.
    int        mean_loop_samples  = 100;    // Use last 100 samples for mean calculation (not exactly but almost).
    useconds_t wait_correction = 0;
    useconds_t cur_pulse_duration = 0;

    if (verbose) printf("Will generate pulses using %s mode on pin %d.\n"
			"Using sleep calibration of %u µs (sleep that shorter)\n",
			simple_mode ? "simple timing" : "accurate timing", pin, calibration);

    // Care for initial output pin state

    int pin_state = GPIO_GET(pin);
    if (pin_state) {
	printf("Warning: pin %d already H. 1st pulse will be a L pulse.\n", pin);
    }

    // Loop to actually generate pulses
    if (verbose) printf("Generating pulses...\n");

    int ox = 1; // output pulse index (1-based, total, including repeats)
    int rep;    // repeat index.
    for (rep = 1; rep <= repeat; rep++) {
	if (verbose && rep > 1) {
	    printf("Repeating %d/%d", rep, repeat);
	    if (simple_mode || verbose < 2) {
		putchar('\r'); fflush(stdout);
	    } else {
		putchar('\n');
	    }
	}
	int ix = 0; // input pulse index

	if (simple_mode) {
	    // Simply call usleep() with given duration of the current pulse.
	    // Assume the busy work we do in between pulses has no significant
	    // impact on the accuracy of generated pulse duration (which is
	    // wrong assumtion). This is similar to what pilight-send does.

	    if (pin_state && ix < pulsec) {
		pin_state = 0;
		pulsi_usleep_calibrated(cur_pulse_duration);
		GPIO_CLR(pin);
		cur_pulse_duration = pulsev[ix++];
	    }
	    pulsec--; // if loop entered, ensure there are at least 2 pulses
	    while (ix < pulsec) {
		pulsi_usleep_calibrated(cur_pulse_duration);
		GPIO_SET(pin); 
		cur_pulse_duration = pulsev[ix++];

		pulsi_usleep_calibrated(cur_pulse_duration);
		GPIO_CLR(pin);
		cur_pulse_duration = pulsev[ix++];
	    }
	    pulsec++;	// uncover above hidden pulse, if any.
	    if (ix < pulsec) {
		pulsi_usleep_calibrated(cur_pulse_duration);
		GPIO_SET(pin); 
		cur_pulse_duration = pulsev[ix++];
		pin_state = 1;
	    }
	    ox += pulsec;
	} else {
	    // Instead of just calling usleep() with the given duration of the
	    // current pulse, we take into account the additional time we need
	    // to do our busy work in this loop. As a result, the effective
	    // sleep time may be less than the given pulse duration.

	    useconds_t slept_duration = 0;
	    struct timeval lap_time;
	    gettimeofday(&lap_time, NULL);
	    for (; ix < pulsec; ix++, ox++) {
		// Measure how long the last loop takes and calculate the mean
		// duration of all loops so far:
		{
		    struct timeval last_lap = lap_time;
		    gettimeofday(&lap_time, NULL);
		    useconds_t loop_duration = (lap_time.tv_usec - last_lap.tv_usec)
				  +  1000000 * (lap_time.tv_sec  - last_lap.tv_sec);

		    // The loop duration includes the duration of the last
		    // sleep (except for 1st loop) and it is stange if the
		    // loop duration is shorter than slept:
		    if (loop_duration < slept_duration) {
			fprintf(stderr, "Seems that usleep(%u µs) returned %u µs too early (after %u µs)?!\n",
					slept_duration, slept_duration - loop_duration, loop_duration);
			// which value should we trust more in this case?
		    } else {
			loop_duration -= slept_duration;
		    }
		    // Calculate mean value of at most 100 loops (or whatever number is
		    // setup in mean_loop_samples). This (1) prevents numeric overflow
		    // and (2) considers timing changes caused due to process schedule.

		    // If the current duration differs from mean duration too much,
		    // then ignore current duration, do not pollute mean value.
		    useconds_t delta = mean_loop_duration > loop_duration
				     ? mean_loop_duration - loop_duration
				     : loop_duration - mean_loop_duration;
		    if (delta < 8 * mean_loop_duration) {
			mean_loop_duration = (mean_loop_duration * (mean_loop_samples - 1)
				    + loop_duration + (mean_loop_samples >> 1)) / mean_loop_samples;
		    }
		}
		wait_correction = mean_loop_duration;

		// Before changing the pin state, ensure the cur_pulse_duration is over.
		if (cur_pulse_duration > wait_correction)
		    pulsi_usleep_calibrated(slept_duration = cur_pulse_duration - wait_correction);
		else
		    slept_duration = 0;

		if (pin_state)
		    GPIO_CLR(pin);
		else
		    GPIO_SET(pin); 
		pin_state ^= 1;
		cur_pulse_duration = pulsev[ix];

		if (verbose > 1) printf(verbose > 1
		    ? "pulse %3d: %s (for %12u µs) [mean loop duration=%12u µs]\n"
		    : "pulse %3d: %s (for %12u µs)\n",
			ox, pin_state ? "H" : "L", cur_pulse_duration, mean_loop_duration);
	    }
	}
    }

    if (verbose == 1 && repeat > 1 && !simple_mode) printf("\n");

    // Ensure we leave with L output.
    if (simple_mode) {
	if (pin_state) {
	    pulsi_usleep_calibrated(cur_pulse_duration);
	    GPIO_CLR(pin);
	    pin_state = 0;
	    ox++;
	} else if (do_last_sleep) {
	    pulsi_usleep_calibrated(cur_pulse_duration);
	}
    } else {
	if (pin_state) {
	    if (cur_pulse_duration > wait_correction)
		pulsi_usleep_calibrated(cur_pulse_duration - wait_correction);
	    GPIO_CLR(pin);
	    pin_state = 0;
	    ox++;
	} else if (do_last_sleep) {
	    if (cur_pulse_duration > wait_correction)
		pulsi_usleep_calibrated(cur_pulse_duration - wait_correction);
	}
    }

    if (verbose > 1) printf("pulse end: L (until changed by someone else)\n");

    if (verbose) printf("Pulses in train: %d, total pulses (pin changes) generated: %d, "
			"shortest %u µs, longest %u µs.\n",
			pulsec, ox - 1, pulse_duration_min, pulse_duration_max);

    if (verbose && !simple_mode) printf("Mean work done per pulse: %u µs "
			"(e.g. can't generate shorter pulses).\n", mean_loop_duration);

    free(pulsev);
    pulsev = NULL;

    exit_gpio();

    exit(EXIT_SUCCESS);
}
