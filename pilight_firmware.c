/* attiny45 - 433 MHz prefilter - V 4.0 - written by mercuri0 & CurlyMo
 * Completely re-written by yablacky.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>

/*********************************************************************
 * General definitions, not specific to this app.
 */
#define bool  					_Bool
#define true  					1
#define false  					0
#define countof(a)				(sizeof(a) / sizeof((a)[0]))
#define bitsof(a)				(sizeof(a)*8)

#define CAT(a, ...) 			PRIMITIVE_CAT(a, __VA_ARGS__)
#define PRIMITIVE_CAT(a, ...) 	a ## __VA_ARGS__

#define SET(a,b)				((a) |=  _BV(b))
#define CLEAR(a,b)				((a) &= ~_BV(b))
#define GET(a,b)				((a) & _BV(b))
#define TOGGLE(a,b)				((a) ^= _BV(b))

// Data-Direction port and pins
#define D_PORT 					DDRB
#define D_PIN(a) 				PRIMITIVE_CAT(DDB,a)

#define SET_OUTPUT(d_port, d_pin)		((d_port) |=  _BV(d_pin))
#define SET_INPUT(d_port, d_pin)		((d_port) &= ~_BV(d_pin))

// Output port and pins
#define O_PORT 					PORTB
#define O_PIN(a) 				PRIMITIVE_CAT(PORTB,a)

// Input port and pins
#define I_PORT 					PINB
#define I_PIN(a) 				PRIMITIVE_CAT(PINB,a)

/*********************************************************************
 * Special definitions, specific to this app.
 **
 * An early note to global and static variables:
 * The filter_method_v4() will define a static ring buffer that occupies
 * almost all of available RAM. The calculation of the that size takes into
 * account all the known global and static variables that are defined in the
 * source code. If you add global or static variables to the code then take
 * care to adjust the size calculation in filter_method_v4() accordingly.
 * To safe memory it might be possible in some cases to re-use global variables
 * rather then to define new ones. More information when and how to do this
 * see notes below at the comment "Variables used by signature sender".
 */
#define FW_CONTROL				0	// "Firmware control port" -> (DIP pin 5) -> PB0 --> PCINT0
#define REC_OUT 				4	// "Receivers output port" -> (DIP pin 3) -> PB4 --> PCINT4
#define PI_IN 					3	// Our output pulses -> PB3 -> (DIP pin 2) -> "PI's input port"

#define PULSE_DIV				34	// pulse divider (from pilight/inc/defines.h)
#define MIN_PULSELENGTH 			15	// unit 10us	//tested to work down to 20us pulsewidth (=2)
#define MAX_PULSELENGTH 			1600	// unit 10us
#define rdiv10(n)				(((n) + 5) / 10)	// convert value of unit 1us to value of unit 10us.
// values from pilight/protocols/core/pilight_firmware_v3.c:
#define PLSLEN 					225	// unit 1us; used when sending signature.
#define PULSE_MULTIPLIER			4

#define VERSION					4
#define SIGNATURES_TOSEND			3
#define SIGNATURE_EACH_US			60000000	// 1 minute.

#define FW_EEPROM_ADDR_CHECKSUM			1		// leaving EEPROM addr 0 untouched by design.
#define FW_EEPROM_ADDR_FILTER_METHOD		2		// filter index of execute_filter is stored here.

#define ctrl_ison()		(	GET(I_PORT, I_PIN(FW_CONTROL))!= 0	)
#define recv_ison()		(	GET(I_PORT, I_PIN(REC_OUT))!= 0		)
#define send_off()		(     CLEAR(O_PORT, O_PIN(PI_IN))		)
#define send_toggle()		(    TOGGLE(O_PORT, O_PIN(PI_IN))		)
#define send_on()		(	SET(O_PORT, O_PIN(PI_IN))		)
#define send_ison()		(	GET(O_PORT, O_PIN(PI_IN)) != 0		)

uint16_t			recving_since_10_us;	// Duration of pulse recently received.
uint16_t			sending_since_10_us;	// Actual duration of pulse currently sent.
uint16_t			send_duration_10_us;	// Desired duration of pulse currently sent.
int32_t				signature_in_10_us;	// Counter for SIGNATURE_EACH_US
uint8_t 			signatures_sent;	// Counter for SIGNATURES_TOSEND

/*********************************************************************
 * The signature this firmware sends is header + payload + footer like this:
 */
const uint16_t header[] = {		// Values: send durations in 10 us unit.
	rdiv10(PLSLEN*PULSE_DIV),	// L duration, simulate a footer.
	rdiv10(PLSLEN),			// H duration, real 1st header pulse.
	rdiv10(PLSLEN*PULSE_MULTIPLIER)	// L duration
};

uint16_t payload[] = {			// Values: pairs of (bit-count, value)
	16, VERSION,			// payload_version change at run time.
#define payload_version			payload[1]
	16, MIN_PULSELENGTH,
	16, MAX_PULSELENGTH,
	4, 0				// checksum filled in at run time.
#define payload_checksum		payload[countof(payload)-1]
};

const uint16_t footer[] = {		// Values: send durations in 10 us unit.
	rdiv10(PLSLEN),			// H duration
	rdiv10(PLSLEN*PULSE_DIV),	// L duration
	rdiv10(PLSLEN),			// H duration (finally toggles to L).
};

#define BIT_HEADER			((typeof(bit)) -countof(header))
#define BIT_PAYLOAD			0	// MUST BE zero.
#define BIT_FOOTER			countof(payload)

// Variables used by signature sender:
// They can be re-used by filters since filters are not executed while
// signatures are being sent. The only rule is that filter must not
// assume a particular value in this variables because filter may change
// and signatures may have been sent in between.
int8_t 					bit;		// does not mean "bit of a byte" but pulse-index or payload-index.
uint8_t 				lsb;		// payload-sub-bit-pulse-index.
uint8_t					payload_nbits;	// number of outstanding bits to send from payload_value.
uint16_t				payload_value;	// current payload value being sent (modified while sending).

/*********************************************************************
 * Signal filter methods.
 */
typedef void (*filter_method_t)(register uint8_t pin_change);

void filter_method_nop(register uint8_t pin_change) { /* NOP */ }	// only send signatures.
void filter_generate_test_signal(register uint8_t pin_change);
void filter_method_v3(register uint8_t pin_change);
void filter_method_v4(register uint8_t pin_change);

filter_method_t execute_filter = filter_method_v4;

/*********************************************************************
 * If just added this function to our code it disables the watchdog after a (soft) reset:
 * http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
 */
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void) {
	MCUSR = 0;
	wdt_disable();
}

uint8_t EEPROM_read_byte(register uint8_t address)
{
	while(EECR & (1<<EEPE))		// Wait for completion of previous write
		;
	EEAR = address;			// Set up address register
	EECR |= (1<<EERE);		// Start eeprom read by set "Read Enable"
	return EEDR;
}

void EEPROM_write_byte(register uint8_t address, register uint8_t data)
{
	while(EECR & (1<<EEPE))		// Wait for completion of previous write
		;
	EECR = (0<<EEPM1)|(0<<EEPM0);	// Set Programming mode "Atomic Byte Programming"
	EEAR = address;			// Set up address and data registers
	EEDR = data;
	uint8_t savedSREG = SREG;	// Save interrupt enable flag (and all others, too)
	cli();				// programming must not be interrupted!
	EECR |= (1<<EEMPE);		// "Master Programming Enable" must be separete instruction
	EECR |= (1<<EEPE);		// Start eeprom write by set "Programming Enable"
	SREG = savedSREG;		// Restore interrupt enable flag (and all others).
}

void calc_checksum(register uint8_t filter_idx) {

	// filter_idx will be zero for "builtin default" filter, not set
	// via info from EEPROM. Otherwise it is the index of the filter
	// method.
	payload_version = VERSION + filter_idx*100;

	int hpf = MAX_PULSELENGTH;
	int lpf = MIN_PULSELENGTH;
	int ver = payload_version;
	while(hpf > 10) hpf /= 10;
	while(lpf > 10) lpf /= 10;
	while(ver > 10) ver /= 10;
	payload_checksum = (ver + lpf + hpf) & 0xf;
}

static filter_method_t get_filter_method(register uint8_t filter_idx) {
	// The filter index of current execute_filter is store in EEPROM so that
	// it remains after power off.
	// The indexes are defined herein; do not change them without good reason.
	switch(filter_idx) {
	default: return 0;
	case 0: return filter_method_nop;
	case 1: return filter_generate_test_signal;
	case 2: return filter_method_v3;
	case 3: return filter_method_v4;
	}
}

void firmware_control(register uint8_t pin_change) {
	if(pin_change) {
		if(ctrl_ison()) {
			// Find current filter and, if found, switch to the next.
			uint8_t filter_idx = 0;
			filter_method_t fxn;
			while((fxn = get_filter_method(filter_idx++)) != 0) {
				if(fxn != execute_filter)
					continue;
				// Current filter method found, get next one:
				if((	fxn = get_filter_method(filter_idx)) == 0)
					fxn = get_filter_method(filter_idx = 0);
				break;
			}
			if(fxn != 0) {
				execute_filter = fxn;
				calc_checksum(filter_idx + 1);

				uint8_t cs = 0;
				EEPROM_write_byte(FW_EEPROM_ADDR_FILTER_METHOD, filter_idx);
				EEPROM_write_byte(FW_EEPROM_ADDR_CHECKSUM, (cs ^= filter_idx));
			}
		} else {
			signature_in_10_us = rdiv10(500);	// in 0.5 ms.
		}
	} else {
		// If a valid filter index is stored in EEPROM then establish that filter.
		uint8_t filter_idx, cs = 0;
		cs ^= (filter_idx = EEPROM_read_byte(FW_EEPROM_ADDR_FILTER_METHOD));
		if(cs == EEPROM_read_byte(FW_EEPROM_ADDR_CHECKSUM)) {
			filter_method_t fxn = get_filter_method(filter_idx);
			if(fxn != 0) {
				execute_filter = fxn;
				calc_checksum(filter_idx + 1);
			}
		}
	}
}

void init_system(void) {

	TCCR1 |= (1 << CTC1) | (1 << CS12);	// the next 2 in one op.
//	SET(TCCR1, CTC1);		// if(TIMER1==OCR1A) call ISR(TIMER1_COMPA_vect); if(TIMER1==OCR1C) TIMER1=0;
//	SET(TCCR1, CS12);		// set TIMER1 prescale: CK/8	(CK=16MHz)
	OCR1A = OCR1C = 20;		// CK/8/20 = CK/160 = 16Mhz / 160 = 100kHz --> interrupt each 10us
	SET(TIMSK, OCIE1A);

	PCMSK |= (1 << PCINT0) | (1 << PCINT4);	// the next 2 in one op.
//	SET(PCMSK, PCINT0);		// care for pin change on DIP pin 5 (firmware control)
//	SET(PCMSK, PCINT4);		// care for pin change on DIP pin 3 (received pulses)
	SET(MCUCR, ISC00);		// ISC00->1, ISC01->(hopefully is 0): interrupt on any logical change.
	SET(GIMSK, PCIE);		// enable pin change interrupts.

	SET_OUTPUT(D_PORT, D_PIN(PI_IN));
	send_off();

	power_adc_disable();
	power_usi_disable();
	power_timer0_disable();

	wdt_enable(WDTO_4S);

	calc_checksum(0);	// filter_idx == 0 means initial method; not set from EEPROM.

	firmware_control(0);

	// Start with normal filter operation:
	signatures_sent = SIGNATURES_TOSEND;
}

int main (void)
{
	cli();
	init_system();
	sei();

	while(1) {
		asm volatile("NOP");
	}
}

uint8_t send_single_pulse(register uint16_t duration_10_us) {

	if(sending_since_10_us < duration_10_us) {
		send_duration_10_us = duration_10_us;
		return 0;	// pulse need more sending time.
	}
	send_toggle();
	send_duration_10_us = sending_since_10_us = 0;
	return 1;		// pulse completely sent.
}

void prepare_signature() {
	send_off();
	send_duration_10_us = sending_since_10_us = 0;
	bit = BIT_HEADER;
	lsb = 0;
}

ISR(TIMER1_COMPA_vect) {
	wdt_reset();
	recving_since_10_us++;

	if(++sending_since_10_us < send_duration_10_us) {
		// Do nothing while sending pulse.

	} else if(signatures_sent >= SIGNATURES_TOSEND) {

		signature_in_10_us -= send_duration_10_us;
		// Set send_duration_10_us = 1 rather than 0 because this value is
		// used to subtract from signature_in_10_us above, in case it is not
		// changed by execute_filter().
		send_duration_10_us = 1;

		if(signature_in_10_us < 0) {
			signature_in_10_us = rdiv10(SIGNATURE_EACH_US);
			// Stop normal operation: Ignore interrupts from receiver (PCINT0).
			CLEAR(GIMSK, PCIE);		// disable pin change interrupts.
			// Prepare for sending signature(s).
			signatures_sent = 0;
			prepare_signature();
		} else {
			// Normal filter operation (indicate from timer, not from a pin_change).
			execute_filter(0);
		}

	// Send signature.

	} else if(bit < BIT_PAYLOAD) {
		// Send header
		bit += send_single_pulse(header[bit - BIT_HEADER]);

		if(bit == BIT_PAYLOAD) {
			payload_nbits = payload[bit++];
			payload_value = payload[bit];
		}

	} else if(bit < BIT_FOOTER) {
		// Send payload

		// A payload bit is coded in 4 pulses in a pattern like this:
		// lsb pulse-index: 0  1   2  3
		//         a 0 bit: P  P   P  PPP (short, short, short, long)
		//         a 1 bit: P  PPP P  P   (short, long, short, short)

		if(send_single_pulse(
				(lsb == ((payload_value & 1) ? 1 : 3)) ? rdiv10(PLSLEN*3) : rdiv10(PLSLEN)
			) && ++lsb == 4) {
			lsb = 0;
			payload_value >>= 1;
			if(--payload_nbits == 0) {
				if(++bit < countof(payload)) {
					payload_nbits = payload[bit++];
					payload_value = payload[bit];
				} else {
					bit = BIT_FOOTER;
				}
			}
		}
		
	} else if(bit < BIT_FOOTER + countof(footer)) {
		// Send footer
		bit += send_single_pulse(footer[bit - BIT_FOOTER]);

	} else {
		// Finished sending one signature.
		send_single_pulse(0);	// mark pulse end (toggles sender);

		if(++signatures_sent < SIGNATURES_TOSEND) {
			prepare_signature();
		} else {
			// Start normal operation: Care for interrupts from receiver (PCINT0).
			SET(GIMSK, PCIE);		// enable pin change interrupts.
		}
	}
}

void filter_method_v3(register uint8_t pin_change) {
	//static uint8_t valid_buffer;
	#define valid_buffer	lsb	// re-use global lsb

	if(pin_change) {
		valid_buffer <<= 1;
		if(recving_since_10_us >= MIN_PULSELENGTH)
		{
			if(recving_since_10_us <= MAX_PULSELENGTH)
			{
				valid_buffer |= 0x01;
				if(valid_buffer == 0xFF)
				{
					send_toggle();
				}
			}
		}
		recving_since_10_us = 0;
		TCNT1 = 0;
	} // else NOP
	#undef valid_buffer
}

void filter_method_v4(register uint8_t pin_change) {

	static const int8_t FILTER_V4_MIN_RAWLEN = 22;	// Must be less then least RAW_LENGTH of 433 protocols
							// (currently 41 for ninjablocks_weather)
	// Define ring buffer size as almost large as possibe.  This depends on
	// available RAM in the target MCU.  Not all of calculates "free" bytes
	// will be used, at least 8 bytes are left "ununsed" to be on the safe side.

	static uint16_t	pulse_ring[
#if TARGET_MCU_attiny85

	    200	// 512 SRAM - 32 STACK - (~~12*4) GLOBAL VARS = 432 BYTES --> 216 words -> use 200 for ring buffer.

#elif TARGET_MCU_attiny25 

	    16	// 128 SRAM - 32 STACK - (~~12*4) GLOBAL VARS = 48 BYTES --> 24 words -> use 16 for ring buffer.

#else /* assume TARGET_MCU_attiny45 */

	    80	// 256 SRAM - 32 STACK - (~~12*4) GLOBAL VARS = 176 BYTES --> 88 words -> use 80 for ring buffer.

#endif
	    ];
	static int8_t wr_idx, rd_idx, rawlen;	// Note: wr/rd_idx walk through the pulse_ring in reverse order
						// (from hi to lo values) so boundary check is at zero most of
						// the time.
	if(pin_change) {
		if(recving_since_10_us >= MIN_PULSELENGTH
		&& recving_since_10_us <= MAX_PULSELENGTH) {
			pulse_ring[wr_idx] = recving_since_10_us;
			if(--wr_idx < 0)
				wr_idx = countof(pulse_ring) - 1;

			// Here is the filter: if less than FILTER_V4_MIN_RAWLEN pulses are in
			// the ring (including current) and current is a footer pulse (>5100 us),
			// then ignore so far rawlen pulses in the ring (rewind).
			// This means: From the (FILTER_V4_MIN_RAWLEN-1)-th pulse that is not a
			// footer, start sending pulses from the ring.
			// Note that using higher MIN_RAWLEN require larger pulse_ring[] buffer
			// and this space is limited on attiny!

			if(++rawlen >= FILTER_V4_MIN_RAWLEN) {
				// If not yet started, start reading and sending pulses from ring:
				if(rd_idx < 0 && (rd_idx = wr_idx + rawlen) >= countof(pulse_ring))
					rd_idx -= countof(pulse_ring);

				if(recving_since_10_us > rdiv10(5100))
					rawlen = 0;
			} else {
				if(recving_since_10_us > rdiv10(5100)) {
					// Not enough pulses. Rewind wr_idx to stop sender as
					// early as possible (may already send some of the pulses).
					do if(++wr_idx >= countof(pulse_ring))
						wr_idx = 0;
					while(--rawlen > 0 && wr_idx != rd_idx);
					rawlen = 0;
				}
			}
		}
		recving_since_10_us = 0;
	} else if(rd_idx >= 0) {
		if(rd_idx == wr_idx) {
			send_single_pulse(0);	// mark pulse end (toggles sender)
			rd_idx = -1;
		} else {
			if(send_single_pulse(pulse_ring[rd_idx])) {
				if(--rd_idx < 0)
					rd_idx = countof(pulse_ring) - 1;
			}
		}
	}
}

void filter_generate_test_signal(register uint8_t pin_change) {

	if(! pin_change) {
		// The test signal are pulses of 2000, 1000, 500, 250, 120, 60 microseconds
		// each sent 4 times, plus one pulse of 5500 microseconds.
		// Filter is re-using global lsb and bit variables!
		uint16_t pulse = lsb;	// "lsb" * 10 us
		if(pulse == 0) {
			pulse = 550;
			bit = 1;	// this pulse only once.
		}
		if(send_single_pulse(pulse) && (--bit & 3) == 0) {
			// lsb of 0 is mapped to 5500 (is not storable in 8-bit lsb).
			lsb = lsb ? (lsb >> 1) < 6 ? 0 : (lsb >> 1) : 200;
		}
	} // else NOP
}

ISR(PCINT0_vect){
	static uint8_t pin_state;
	uint8_t pin_change = pin_state;
	pin_change ^= (pin_state = PINB);

	if(GET(pin_change, I_PIN(REC_OUT))) {
		execute_filter(pin_change);
	}

	if(GET(pin_change, I_PIN(FW_CONTROL))) {
		firmware_control(pin_change);
	}
}
