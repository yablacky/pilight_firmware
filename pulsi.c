/**
 * Pulsi. Generate pulses on a Raspberry-Pi GPIO pin.
 * @author yablacky <schwarz.ware@gmx.de>
 * @date 2017-01-019
 * @abstract Intention was to generate pulse trains to test pilight
 * receiver and firmware filter. Pulses are specified like for
 * pilight-send for the "raw" protocol. But pilight-send requires
 * the pilight-daemon to run while pulsi does not.
 */

#define VERSION			"0.1"

// For Raspberry Pi 2 and Pi 3, change BCM2708_PERI_BASE to 0x3F000000 for the code to work. 

//#define BCM2708_PERI_BASE	0x20000000
#define BCM2708_PERI_BASE	0x3F000000

#define GPIO_BASE		(BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define GPIO_PINS_COUNT		26

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

// I/O access
volatile unsigned long *gpio;

#define _GPIO_DIR	(gpio)		// 1 set output, 0 set input (almost, see macros)
#define _GPIO_SET	(gpio+7)	// Sets   bits which are 1 ignores bits which are 0
#define _GPIO_CLR	(gpio+10)	// Clears bits which are 1 ignores bits which are 0
#define _GPIO_GET	(gpio+13)	// Delivers bits which are 1
#define _GPIO_PULL	(gpio+37)	// Pull up/pull down
#define _GPIO_PULLCLK0	(gpio+38)	// Pull up/pull down clock

#define GPIO_DIR_INP(p) do {                 _GPIO_DIR[p / 10] &= ~(7<<((p % 10) * 3)); } while (0)
#define GPIO_DIR_OUT(p) do {GPIO_DIR_INP(p); _GPIO_DIR[p / 10] |=  (1<<((p % 10) * 3)); } while (0)

#define GPIO_SET(p)	(   _GPIO_SET[0] = (1 << p)		)
#define GPIO_CLR(p)	(   _GPIO_CLR[0] = (1 << p)		)
#define GPIO_GET(p)	(  (_GPIO_GET[0] & (1 << p))!=0		)

/**
 * Initialize accessing gpio pins. Setup global gpio pointer.
 */
int init_gpio()
{
    int  mem_fd;

    if (sizeof(*gpio) != 4) {
	fprintf(stderr, "not a 32 bit type");
	return -1;
    }

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
    return 0;
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

/**
 * Parse a string as micro seconds value with optional unit.
 * @param str The string to parse.
 * @param pmore Optinally return unknown unit here.
 * @return The time value in micro seconds.
 */
useconds_t strtousec(const char *str, char **pmore)
{
    char *more = "";
    if (pmore) *pmore = more;
    useconds_t t = strtoul(str, &more, 0);
    if (strcmp(more, "s") == 0)  t *= 1000000; else 
    if (strcmp(more, "ms") == 0) t *= 1000; else 
    if (strcmp(more, "us") == 0) ; else 
    if (strcmp(more, "min") == 0) t *= 60000000; else
    if (pmore) *pmore = more;
    return t;
}

const char *progname = "?";
int verbose = 0;

void print_help()
{
    fprintf(stderr,
	"usage: %s [-v] [-P GPIO_pin_number] [-r count] pulse_duration...\n"
	"  -P : Generate hi/lo pulses on the given GPIO pin (0 .. %d), default pin 9.\n"
	"  -r : Repeat: Send the pulse train count times.\n"
	"  -v : Tell what is going on. More -v tell more.\n"
	"  Pulse duration values are integers with optional unit:\n"
	"    us  Micro seconds (default unit)\n"
	"    ms  Milli seconds\n"
	"    s   Seconds\n"
	"    min Minutes\n"
	"  This is version %s as of %s %s.\n"
	, progname, GPIO_PINS_COUNT - 1, VERSION, __DATE__, __TIME__);
}

int main(int argc, char **argv)
{
    progname = argv[0];
    int pin = 9, repeat = 1;
    { const char *p = getenv("PULSI_GPIO_PIN_NUMBER"); if (p) pin = atoi(p); }

    int opt;
    while ((opt = getopt(argc, argv, "vr:P:")) != -1) {
	char *more = "";
	switch (opt) {
	case 'v': verbose++; break;
	case 'r': repeat = strtol(optarg, NULL, 0); break;
	case 'P': pin    = strtol(optarg, NULL, 0); break;
	default:
	    print_help(argv[0]);
	    exit(EXIT_FAILURE);
	}
    }
    if (pin < 0 || pin >= GPIO_PINS_COUNT) {
	fprintf(stderr, "%s: pin number %d not in range 0..%d\n", progname, pin, GPIO_PINS_COUNT - 1);
	exit(EXIT_FAILURE);
    }

    if (optind >= argc) {
	print_help(argv[0]);
	exit(EXIT_FAILURE);
    }

    if (init_gpio() < 0)
	exit(EXIT_FAILURE);

    // Set GPIO pin direction to output.
    GPIO_DIR_OUT(pin);

    useconds_t wait = 0;
    int state = GPIO_GET(pin);
    if (state) {
	printf("Warning: pin %d already H. 1st pulse will be a L pulse.\n", pin);
    }

    struct timeval pulse_start = { 0 };

    char **pulsev = argv + optind;
    int pulsec = argc - optind;
    int ox = 1; // output pulse index (total)
    int rep;    // repeat index.
    for (rep = 1; rep <= repeat; rep++) {
	if (rep > 1 && verbose) printf("Repeating %d/%d\n", rep, repeat);
	int ix = 0; // input pulse index
	for (; ix < pulsec; ix++) {
	    char *pulses = strdup(pulsev[ix]);	// Copy because strtok() modifies the string.
	    char *duration = strtok(pulses, " \t\n");
	    for (; duration != NULL; duration = strtok(NULL, " \t\n")) {
		char *more = "";
		useconds_t t = strtousec(duration, &more);
		if (more && *more) {
		    fprintf(stderr, "%s: pulse#%d: unit '%s' is not one of (us, ms, s, min)\n", progname, ox, more);
		    break;
		}
		if (wait) {
		    struct timeval spend;
		    gettimeofday(&spend);
		    spend.tv_sec -= pulse_start.tv_sec;
		    spend.tv_usec -= pulse_start.tv_usec;
		    spend.tv_usec += spend.tv_sec * 1000000U;
		    if (verbose > 1) printf("\t(Spend %u µs of %d µs to wait)\n", spend.tv_usec, wait);
		    if (spend.tv_usec < wait)
			usleep(wait - spend.tv_usec);
		}
		if (verbose) printf("pulse %3d: pin %d: %s (for %12u µs)\n", ox, pin, state ? "L" : "H", t);
		if (state)
		    GPIO_CLR(pin);
		else
		    GPIO_SET(pin); 
		gettimeofday(&pulse_start);
		state ^= 1;
		ox++;
		wait = t;
	    }
	    free(pulses);
	}
    }
    if (state) {
	if (wait) {
	    struct timeval spend;
	    gettimeofday(&spend);
	    spend.tv_sec -= pulse_start.tv_sec;
	    spend.tv_usec -= pulse_start.tv_usec;
	    spend.tv_usec += spend.tv_sec * 1000000U;
	    if (verbose > 1) printf("\t(Spend %u µs of %d µs to wait)\n", spend.tv_usec, wait);
	    if (spend.tv_usec < wait)
		usleep(wait - spend.tv_usec);
	}
	if (verbose) printf("pulse end: pin %d: L (until changed by someone else)\n", pin);
	GPIO_CLR(pin);
	gettimeofday(&pulse_start);
	state = 0;
    } else {
	if (verbose) printf("pulse end: pin %d: L (until changed by someone else)\n", pin);
    }

    exit_gpio();

    exit(EXIT_SUCCESS);
}
