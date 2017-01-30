/* attiny45 - 433 MHz prefilter - V 6.0 - written by mercuri0 & CurlyMo
 * Completely re-written by yablacky, 2016-2017.
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

#define SETBIT(a,b)				((a) |=  _BV(b))
#define CLRBIT(a,b)				((a) &= ~_BV(b))
#define GETBIT(a,b)				((a) & _BV(b))
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

#ifndef USE_MANUALLY_OPTIMIZED_CODE
#define USE_MANUALLY_OPTIMIZED_CODE		1	// 1 or 0
#endif

#define FW_CONTROL				0	// "Firmware control port" -> (DIP pin 5) -> PB0 --> PCINT0
#define REC_OUT 				4	// "Receivers output port" -> (DIP pin 3) -> PB4 --> PCINT4
#define REC2_OUT 				1	// "2nd Receivers output port" -> (DIP pin 6) -> PB1 --> PCINT1
#define PI_IN 					3	// Our output pulses -> PB3 -> (DIP pin 2) -> "PI's input port"

#define PULSE_DIV				34	// pulse divider (from pilight/inc/defines.h)
#define MIN_PULSELENGTH 			15	// unit 10us	//tested to work down to 20us pulsewidth (=2)
#if ENABLE_HIGH_PASS_FILTER
	// Enabling the high pass filter (HPF) is not recommended because
	// 1) some protocols have valid long length footer pulses.
	// 2) the pause between pulse trains can be a long "pulse" that the HPF would
	//    filter away by in fact ignoring the first valid pulse after such a pause.
	// The HPF code is left here for reference.
#define MAX_PULSELENGTH 			1600	// unit 10us
#else
#define MAX_PULSELENGTH 			0x7fff	// Should be zero but daemon then does not update registry :-(
#endif
#define rdiv10(n)				(((n) + 5) / 10)	// convert value of unit 1us to value of unit 10us.
// values from pilight/protocols/core/pilight_firmware_v3.c:
#define PLSLEN 					225	// unit 1us; used when sending signature.
#define PULSE_MULTIPLIER			4

#define VERSION					6	// version 5 if pin_change_in_10_us < 0
#define SIGNATURES_TOSEND			3
#ifndef DEBUG_FILTER
#define SIGNATURE_EACH_US			60000000	// 1 minute.
#else
#define SIGNATURE_EACH_US			(60000000*15)	// 15 minutes.
#endif

#define FW_EEPROM_ADDR_CHECKSUM			1		// leaving EEPROM addr 0 untouched by design.
#define FW_EEPROM_ADDR_FILTER_METHOD		2		// filter index of execute_filter is stored here.
#define FW_EEPROM_ADDR_JACKET_LPF		3		// 0 or 1

uint8_t				receiver_pin_mask;		// PINB mask of selected receiver pin.

#define ctrl_ison()		(	GETBIT(I_PORT, I_PIN(FW_CONTROL))!= 0	)
#define recv_ison()		(	((I_PORT) & receiver_pin_mask)!= 0	)
#define send_off()		(	CLRBIT(O_PORT, O_PIN(PI_IN))		)
#if 0
    // This toggle code works but compiles as 3 assembler statements:
#define send_toggle()		(	TOGGLE(O_PORT, O_PIN(PI_IN))		)
#else
    // On attiny, writing 1 to IN toggles OUT; this compiles as 1 assembler statement:
#define send_toggle()           (	SETBIT(I_PORT, I_PIN(PI_IN))            )
#endif
#define send_on()		(	SETBIT(O_PORT, O_PIN(PI_IN))		)
#define send_ison()		(	GETBIT(O_PORT, O_PIN(PI_IN)) != 0	)

#define PIN_CHANGE_DELAY_US	MIN_PULSELENGTH
#if PIN_CHANGE_DELAY_US < 128
#define PIN_CHANGE_DELAY_T	int8_t
#else
#define PIN_CHANGE_DELAY_T	int16_t
#endif

PIN_CHANGE_DELAY_T		pin_change_in_10_us;	// Remaining time to delay pin change processing.
							// This enables the new additional LPF method.
							// Negative value means immediate processing
							// which is the old LPF method.

uint8_t				ten_us_clock;		// 255 -> 0 ok. Allows to measure 2.56 ms durations.
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
#define rec_pin_feedback(n)		(payload[5] = MAX_PULSELENGTH - (n))
	4, 0				// checksum filled in at run time.
#define payload_checksum		payload[countof(payload)-1]
};

const uint16_t footer[] = {		// Values: send durations in 10 us unit.
	rdiv10(PLSLEN),			// H duration
	rdiv10(PLSLEN*PULSE_DIV),	// L duration
	rdiv10(PLSLEN),			// H duration (finally toggles to L).
};

// Data index values for signture parts:
#define DIX_HEADER			((typeof(dix)) -countof(header))
#define DIX_PAYLOAD			0	// MUST BE zero.
#define DIX_FOOTER			countof(payload)

// Variables used by signature sender:
// They can be re-used by filters since filters are not executed while
// signatures are being sent. The only rule is that filter must not
// assume a particular value in this variables because filter may change
// and signatures may have been sent in between.
int8_t 					dix;		// current data index.
uint8_t 				lsb;		// payload-sub-pulse-index.
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
void filter_passthru(register uint8_t pin_change);

filter_method_t execute_filter = filter_method_v4;

/*********************************************************************
 * If just added this function to our code it disables the watchdog after a (soft) reset:
 * http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
 */
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
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

void calc_checksum(register uint8_t filter_idx)
{
	// filter_idx will be zero for "builtin default" filter, not set
	// via info from EEPROM. Otherwise it is the index of the filter
	// method.
	payload_version = VERSION;
	if(pin_change_in_10_us < 0)
	    payload_version = VERSION - 1;
	payload_version += filter_idx * 100;

	int hpf = MAX_PULSELENGTH;
	int lpf = MIN_PULSELENGTH;
	int ver = payload_version;
	while(hpf > 10) hpf /= 10;
	while(lpf > 10) lpf /= 10;
	while(ver > 10) ver /= 10;
	payload_checksum = (ver + lpf + hpf) & 0xf;
}

static filter_method_t get_filter_method(register uint8_t filter_idx)
{
	// The filter index of current execute_filter is store in EEPROM so that
	// it remains after power off.
	// The indexes are defined herein; do not change them without good reason.
	switch(filter_idx) {
	default: return 0;
	case 0: return filter_method_nop;
	case 1: return filter_generate_test_signal;
	case 2: return filter_method_v3;
	case 3: return filter_method_v4;
	case 4: return filter_passthru;
	}
}

void firmware_control(register uint8_t pin_change)
{
	// Firmware control checks for pin change pulses like this:
	// A HI pulse of 1s .. 2s duration finishes control sequence.
	// A HI pulse of more than 2s cancels control sequence.
	// A HI pulse less than 1s is a bit value and the duration is indicator of bit value.
	#define FW_CONTROL_TIMEBASE_10_US	rdiv10(SIGNATURE_EACH_US)
	#define FW_CONTROL_HIGH_BIT_10_US	rdiv10( 100000)	    // 100 ms; a 50ms pulse is 0, a 150ms pulse is 1.
	#define FW_CONTROL_FINISHED_10_US	rdiv10( 900000)	    // 900 ms; a bit less than 1 second.
	#define FW_CONTROL_TIME_OUT_10_US	rdiv10(2100000)	    // 2.1 seconds
	#define FW_CONTROL_STARTING_10_US	(FW_CONTROL_TIMEBASE_10_US + FW_CONTROL_TIME_OUT_10_US)
	#define	pin_change_count		payload_nbits
	#define control_value			payload_value
	if(!pin_change) {
		// If a valid filter index is stored in EEPROM then establish that filter.
		uint8_t filter_idx, cs = 0;
		cs ^= (filter_idx = EEPROM_read_byte(FW_EEPROM_ADDR_FILTER_METHOD));
		if(cs == EEPROM_read_byte(FW_EEPROM_ADDR_CHECKSUM)) {

			pin_change_in_10_us = EEPROM_read_byte(FW_EEPROM_ADDR_JACKET_LPF) ? 0 : -1;

			filter_method_t fxn = get_filter_method(filter_idx);
			if(fxn != 0) {
				execute_filter = fxn;
				calc_checksum(filter_idx + 1);
			}
		}
		return;
	}
	// Control pin changed.
	// Since we get here, the signature sender is not active (it would have
	// disabled the interrupt that lead us here). So we can re-use the signature
	// timer to measure timing of firmware control pin changes and thereby
	// making sure the signature sender stays inactive.
	if(signature_in_10_us <= FW_CONTROL_TIMEBASE_10_US) {
		// This is the first control call (since a long time).
		pin_change_count = 0;
		control_value = 0;
	}
	if(ctrl_ison()) {
		pin_change_count++;
		signature_in_10_us = FW_CONTROL_STARTING_10_US;
		return;
	}
	// Pin changed to off.
	if(pin_change_count == 0) {
		// Strange situation. Can't trust signature timer. Ignore.
		return;
	}

	int32_t duration_10_us = FW_CONTROL_STARTING_10_US - signature_in_10_us;
	if(duration_10_us < 0) {
		// Strange situation. Signature time is beyond our starting
		// time. This should not happen. Leave everything as is.
		return;
	}

	if(duration_10_us < FW_CONTROL_FINISHED_10_US) {
		// Received a bit. Duration is indicator of its value:
		control_value <<= 1;
		control_value |= (duration_10_us > FW_CONTROL_HIGH_BIT_10_US);
		// wait for another pin change
		signature_in_10_us = FW_CONTROL_STARTING_10_US;
		return;
	}

	if(pin_change_count > 16) {
		// Too much bits sent. Ignore control request.
		// no return here; we set the signature timer below.
	} else {
		// Control sequence finished.
		// Control_value bit layout is:
		// FEDCBA9876543210
		//             ffff = 0x0F; filter index 1 based, 0=next, 0xF = no change.
		//          vvv     = 0x70; version select; 0 = no change; undefined = no change.
		//        rr        = 0x18; receiver select; 0 = no change; undefined = no change.
		// ..........       = unused
		int8_t modified = 0;

		uint8_t filter_idx = control_value & 0x0F;

		// Version select is mainly to turn on or off the new LPF method
		// that is represented by version 6 and 5. The value coded in vvv
		// is the desired version minus 2 because version 1 and 2 are not
		// available and to have more room for future version numbers that
		// can be coded in 3 bits (up to version 9, which is 7 + 2)
		switch((uint8_t)((control_value & 0x70) >> 4)) {
		case 3-2:
			if(filter_idx == 0 || filter_idx == 0x0F)
				filter_idx = 3;	// 1 based index of filter_method_v3
			// fall thru
		case 4-2:
			if(filter_idx == 0 || filter_idx == 0x0F)
				filter_idx = 4;	// 1 based index of filter_method_v4
			// fall thru
		case 5-2:
			if(pin_change_in_10_us >= 0) modified = 1;
			pin_change_in_10_us = -1; // off
			break;
		case 6-2:
			if(pin_change_in_10_us < 0) modified = 1;
			pin_change_in_10_us = 0; // on
			break;
		}

		filter_method_t fxn;
		uint8_t find_next;

		if(filter_idx == 0) {
			// Below, find current filter and, if found, switch to the next.
			fxn = 0;
			find_next = 1;
		} else {
			// Switch to given filter (or keep unchanged and, below, find current).
			fxn = get_filter_method(--filter_idx);
			find_next = 0;
		}

		if(fxn == 0) {
			filter_idx = 0;
			while((fxn = get_filter_method(filter_idx++)) != 0) {
				if(fxn != execute_filter)
					continue;
				// Current filter method found
				if(!find_next)
					--filter_idx;
				else if((fxn = get_filter_method(filter_idx)) == 0)
					 fxn = get_filter_method(filter_idx = 0);
				break;
			}
			// if(fxn == 0) here, which should never happen, then
			// (at least) filter index is the first "undefined" number.
		}

		if(fxn != 0) {
			if(execute_filter != fxn) modified = 1;
			execute_filter = fxn;
		}

		switch((uint8_t)((control_value & 0x180) >> 7)) {
		case 1:
		    if(GETBIT(PCMSK, CAT(PCINT, REC2_OUT))) modified = 1;
		    CLRBIT(PCMSK, CAT(PCINT, REC2_OUT));
		    SETBIT(PCMSK, CAT(PCINT, REC_OUT));
		    receiver_pin_mask = (1 << I_PIN(REC_OUT));
		    rec_pin_feedback(1);
		    break;
		case 2:
		    if(GETBIT(PCMSK, CAT(PCINT, REC_OUT))) modified = 1;
		    CLRBIT(PCMSK, CAT(PCINT, REC_OUT));
		    SETBIT(PCMSK, CAT(PCINT, REC2_OUT));
		    receiver_pin_mask = (1 << I_PIN(REC2_OUT));
		    rec_pin_feedback(2);
		    break;
		}

		if(modified) {
			calc_checksum(filter_idx + 1);

			uint8_t cs = 0;
			EEPROM_write_byte(FW_EEPROM_ADDR_FILTER_METHOD, filter_idx);
			EEPROM_write_byte(FW_EEPROM_ADDR_JACKET_LPF, pin_change_in_10_us >= 0);
			EEPROM_write_byte(FW_EEPROM_ADDR_CHECKSUM, (cs ^= filter_idx));
		}
	}
	// Finally tell soon what we have now.
	signature_in_10_us = rdiv10(1000);	// in 1 ms.
}

void init_system(void)
{
	TCCR1 |= (1 << CTC1) | (1 << CS12);	// the next 2 in one op (they do not compile to sbi/cbi instructions).
//	SETBIT(TCCR1, CTC1);		// if(TIMER1==OCR1A) call ISR(TIMER1_COMPA_vect); if(TIMER1==OCR1C) TIMER1=0;
//	SETBIT(TCCR1, CS12);		// set TIMER1 prescale: CK/8	(CK=16MHz)
	OCR1A = OCR1C = 20;		// CK/8/20 = CK/160 = 16Mhz / 160 = 100kHz --> interrupt each 10us
	SETBIT(TIMSK, OCIE1A);

	rec_pin_feedback(0);	// 0 -> indicates that default receiver pin is used.
	receiver_pin_mask = (1 << I_PIN(REC_OUT));  // initially watch receiver at REC_OUT pin.
	SETBIT(PCMSK, CAT(PCINT, FW_CONTROL));	// care for pin change caused by firmware control
	SETBIT(PCMSK, CAT(PCINT, REC_OUT));	// care for pin change caused by receiver 1 output
//later	SETBIT(PCMSK, CAT(PCINT, REC2_OUT));	// care for pin change caused by receiver 2 output

	SETBIT(MCUCR, ISC00);		// ISC00->1, ISC01->(hopefully is 0): interrupt on any logical change.

	SET_INPUT(D_PORT, D_PIN(FW_CONTROL));
	SET_INPUT(D_PORT, D_PIN(REC_OUT));
	SET_INPUT(D_PORT, D_PIN(REC2_OUT));
	SET_OUTPUT(D_PORT,D_PIN(PI_IN));

	send_off();

	power_adc_disable();
	power_usi_disable();
	power_timer0_disable();

	wdt_enable(WDTO_4S);

	calc_checksum(0);	// filter_idx == 0 means initial method; not set from EEPROM.

	firmware_control(0);

	// Start with normal filter operation:
	signatures_sent = SIGNATURES_TOSEND;

	SETBIT(GIMSK, PCIE);		// enable pin change interrupts.
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

#define send_continue_pulse(duration) (void)(send_duration_10_us = (duration))
#define send_single_pulse(duration) (void)(send_toggle(), send_continue_pulse(duration))

#define delayed_pin_change_processing() do {		\
	PIN_CHANGE_DELAY_T t = pin_change_in_10_us;	\
	if(--t >= 0)					\
	    __delayed_pin_change_processing(t);		\
	} while(0)
void __delayed_pin_change_processing(PIN_CHANGE_DELAY_T t) {
	pin_change_in_10_us = t;
	if(t == 0) {
		execute_filter(1);
		recving_since_10_us = 0;
	}
}

void prepare_signature()
{
	dix = DIX_HEADER;
	lsb = 0;
}

void send_signatures()
{
	if(dix < DIX_PAYLOAD) {
		// Send header
		send_single_pulse(header[dix - DIX_HEADER]);

		if(++dix == DIX_PAYLOAD) {
			payload_nbits = payload[dix++];
			payload_value = payload[dix];
		}
		return;
	}

	if(dix < DIX_FOOTER) {
		// Send payload

		// A payload bit is coded in 4 pulses in a pattern like this:
		// lsb pulse-index: 0  1   2  3
		//         a 0 bit: P  P   P  PPP (short, short, short, long)
		//         a 1 bit: P  PPP P  P   (short, long, short, short)

		send_single_pulse(
			(lsb == ((payload_value & 1) ? 1 : 3)) ? rdiv10(PLSLEN*3) : rdiv10(PLSLEN)
		);
		if(++lsb == 4) {
			lsb = 0;
			payload_value >>= 1;
			if(--payload_nbits == 0) {
				if(++dix < countof(payload)) {
					payload_nbits = payload[dix++];
					payload_value = payload[dix];
				} else {
					dix = DIX_FOOTER;
				}
			}
		}
		return;
	}

	if(dix < DIX_FOOTER + countof(footer)) {
		// Send footer
		send_single_pulse(footer[dix - DIX_FOOTER]);
		dix++;
		return;
	}

	// Finished sending one signature.

	if(++signatures_sent < SIGNATURES_TOSEND) {
		prepare_signature();
		return;
	}

	// Finished sending all signatures.

	// Leave with sender off.
	if(send_ison())
		send_single_pulse(9);

	signature_in_10_us = rdiv10(SIGNATURE_EACH_US);

	// Start normal operation: Care for interrupts from receiver (PCINT0).
	SETBIT(GIMSK, PCIE);	// enable pin change interrupts.
}

ISR(TIMER1_COMPA_vect) {
	wdt_reset();

#if USE_MANUALLY_OPTIMIZED_CODE
	// Better asm code than the #else path.
	// (saves one superfluous compare op).
	register uint8_t t;
	if((t = ten_us_clock + 1) == 0)
		signature_in_10_us -= 256;
	ten_us_clock = t;
#else
	if(++ten_us_clock == 0)
		signature_in_10_us -= 256;
#endif

	if(++recving_since_10_us == 0) // overflow
		recving_since_10_us = ~0;
	
	// Note that sending_since_10_us can't overflow because it will
	// not even reach send_duration_10_us which can be 0xFFFF max.

	if(++sending_since_10_us < send_duration_10_us) {
		// Do nothing while sending pulse.
		// ... except:
		delayed_pin_change_processing();
		return;
	}
	sending_since_10_us = 0;

	// Duration of current pulse is over. To terminate the pulse,
	// the sender pin needs to be toggled. This is not done here,
	// because in some cases this must regularly not happen. So
	// this is left to be done by the remaining code.

	// If signature sender is active, continue sending signature pulses:
	if(signatures_sent < SIGNATURES_TOSEND) {
		send_signatures();
		return;
	}

	// Signature sender is not active.
#if USE_MANUALLY_OPTIMIZED_CODE
	// Better asm code than the #else path.
	// (does not load all 4 bytes in registers; only the most significant one which contains the sign)
	if(((int8_t*)&signature_in_10_us)[sizeof(signature_in_10_us)-1] < 0)
#else
	if(signature_in_10_us < 0)
#endif
	{
		// It is time to send signatures!

		// Stop normal operation: Ignore interrupts from receiver (PCINT0).
		CLRBIT(GIMSK, PCIE);		// disable pin change interrupts.

		// Turn off delayed pin change processing if it is enabled et al.
		if(pin_change_in_10_us >= 0)	// including 0 in test generates better asm.
			pin_change_in_10_us = 0;

		// By design, the 1st header pulse is a L pulse. To achieve
		// this, the sender must be H before.  The send state can
		// savely set here: It is was H already then it is no change
		// et all (except the side effect that the pulse is not
		// terminated at regular time).  If it is was L then it is
		// just a regular pulse termination:
		send_on();

		signatures_sent = 0;
		prepare_signature();
		return;
	}

	// Set a idefault duration that the last pulse lasts in case
	// execute_filter() does not call send_single_pulse().
	send_continue_pulse(rdiv10(500000));	// 500 ms.

	delayed_pin_change_processing();

	// Normal filter operation (indicate from timer, not from a pin_change).
	execute_filter(0);
}

void filter_method_v3(register uint8_t pin_change) {
	//static uint8_t valid_buffer;
	#define valid_buffer	lsb	// re-use global lsb

	if(pin_change) {
		valid_buffer <<= 1;
		if(recving_since_10_us >= MIN_PULSELENGTH)
		{
#if ENABLE_HIGH_PASS_FILTER
			if(recving_since_10_us <= MAX_PULSELENGTH)
#endif
			{
				valid_buffer |= 0x01;
				if(valid_buffer == 0xFF)
				{
					send_toggle();
				}
			}
		}
		TCNT1 = 0;
	} // else NOP
	#undef valid_buffer
}

void filter_passthru(register uint8_t pin_change) {

	if(pin_change) {
		if(recv_ison()) {
			send_on();
		} else {
			send_off();
		}
	} // else NOP
}

#ifndef FILTER_V4_DEBUG
#define FILTER_V4_DEBUG 0
#endif

void filter_method_v4(register uint8_t pin_change) {

#if FILTER_V4_DEBUG
	//----------------------------------------------------------------------------------
	// How to use debugger:
	// In the filter(V4)-code, set debug4_value[...] to values you want to examine.
	// The values are then output as pulse pattern before the sender goes to "sleep" mode.
	// Beware that the filter-code may run multiple times before debug4_values are output.
	// After debug4_values are output they are initialized to zero. So its simple to sum
	// up events, for example.
	static uint8_t debug4_value[4];	// size must be power of 2, max 16.
	#define debug4_output_done()	(void)0	    // Replace this by code to prepare next debug round.

	// Debug4 implementation:
	#define msb_mask(all_1_value)	((all_1_value) & ~((all_1_value) >> 1))
	#define debug4_send_bit_pulse(bit)	send_single_pulse((bit) ? rdiv10(1000) : rdiv10(100))
	#define debug4_idx_mask		0xF0	    // indicates that the mask (in lower 4 bits) is for debug4_idx.
	#define debug4_idx_start_mask	(debug4_idx_mask | msb_mask(countof(debug4_value) - 1))
	#define debug4_output_start()	(void)(debug4_mask = debug4_idx_start_mask)
	static uint8_t debug4_mask, debug4_idx;
#else
	#define debug4_output_start()	(void)0
	//----------------------------------------------------------------------------------
#endif

	// This filter is not (only) a low level noise filter. It filters the signals
	// on application level, namely the pilight application. The filter interprets
	// the pulse durations and number (pulse train length) like pilight and it
	// uses application knowledge of which pulse trains are worth to pass the
	// filter and which to suppress completely.
	// In order to suppress a pulse train completey, the filter does not send
	// received pulses immediately but stores their duration in a ring buffer,
	// analyses them and, if they should pass, sends them later or discards
	// them in the buffer without sending them. Discarded pulses do not cause
	// output pin changes which means that the pulse currently being sent lasts
	// for the duration of discarded pulses.
	// The decision of which pulse trains pass the filter is made on knowledge
	// of minimum pulse train length and how to detect a footer pulse which
	// marks the end of a pulse train:

#define FILTER_V4_MIN_RAWLEN	    22		// Must be less then least RAW_LENGTH of 433 protocols
						// (currently 41 for ninjablocks_weather)
#define IS_FOOTER_PULSE		    (recving_since_10_us > rdiv10(5100))


	// Define ring buffer size as almost large as possibe.  This depends on
	// available RAM in the target MCU.  Not all of calculated "free" bytes
	// will be used to be on the safe side. Further the size must be a power
	// of two for easy index-wrap-around.

	static uint16_t	pulse_ring[	// the size of the pulse_ring must be power of 2!
#if TARGET_MCU_attiny85

	    128	    // 512 SRAM - 32 STACK - (~~12*4) GLOBAL VARS = 432 BYTES --> 216 words -> use 128 for ring buffer.

#elif TARGET_MCU_attiny25 
#undef  FILTER_V4_MIN_RAWLEN			// Must not be larger then ring buffer size...
#define FILTER_V4_MIN_RAWLEN	    8		// Must be less then least RAW_LENGTH of 433 protocols

	    16	    // 128 SRAM - 32 STACK - (~~12*4) GLOBAL VARS = 48 BYTES --> 24 words -> use 16 for ring buffer.

#else /* assume TARGET_MCU_attiny45 */

	    64	    // 256 SRAM - 32 STACK - (~~12*4) GLOBAL VARS = 176 BYTES --> 88 words -> use 64 for ring buffer.

#endif
	    ];

#define	PULSE_RING_IDX		(countof(pulse_ring) - 1)	// Bitmask to calc ring buffer index modulo.
#define PULSE_RING_SLEEP	128	// Must be 1 bit and above max ring buffer index (127 on attiny 85).

	static uint8_t wr_idx,	// write index; where to store the next received pulse.
		       rd_idx,	// read index, which pulse to send next.
		       ts_idx,	// train start index, first pulse of currently receiving train.
		       rawlen;	// length (number of pulses) of currently receiving pulse train.

	if(pin_change) {
		if(recving_since_10_us < MIN_PULSELENGTH
#if ENABLE_HIGH_PASS_FILTER
		|| recving_since_10_us > MAX_PULSELENGTH
#endif
		)
			return;

		pulse_ring[wr_idx] = recving_since_10_us;
		wr_idx = (wr_idx + 1) & PULSE_RING_IDX;
		if(rawlen >= FILTER_V4_MIN_RAWLEN-1) {	// enough pulses received.
			// Send this pulse train.
			if(rd_idx & PULSE_RING_SLEEP) {
				// Sender "sleeps", wake it up.
				rd_idx = ts_idx;

				// Stop sending the current pulse asap:
				if(sending_since_10_us < ~0 -2)	// prevent overflow
					send_continue_pulse(sending_since_10_us + 2);
			}
#if USE_MANUALLY_OPTIMIZED_CODE
	// This variant merges the two "if(IS_FOOTER_PULSE)..." code parts (in
	// the #else path and behind #endif) into one, namely the later one.
	// This saves some jump ops and results in better asm code.
			ts_idx = wr_idx;
		} if (0) {
#else
			if(IS_FOOTER_PULSE) {
				ts_idx = wr_idx;    // pulse train finished, new train starts here.
				rawlen = 0; // pulse train finished, next pulse starts a new train.
			} else {
				ts_idx = wr_idx;    // drag the train start index or sender would stop.
				rawlen++;
			}
#endif
		} else if(IS_FOOTER_PULSE) {
			// Too short pulse train. Do not send.
			wr_idx = ts_idx;	// rewind to train start index.
			rawlen = 0; // discard train, next pulse starts a new train.
		} else {
			// Just remember and count that pulse.
			rawlen++;
		}

	} else {
		// The last pulse has been completely sent (or was prematurely
		// stopped when sender was "waked up" while sending a sleeping pulse).
		if(!(rd_idx & PULSE_RING_SLEEP)) {
			if(rd_idx != ts_idx) {
				// More pulses available.
				send_single_pulse(pulse_ring[rd_idx]);
				rd_idx = (rd_idx + 1) & PULSE_RING_IDX;
			} else {
				// No more pulses.
				send_single_pulse(rdiv10(500000));   // 0.5 sec
				rd_idx = PULSE_RING_SLEEP;  // no need to OR it in.
				debug4_output_start();
			}
		} else {
#if FILTER_V4_DEBUG
	//----------------------------------------------------------------------------------
	// Debug4 implementation:
	// Sleep mode is entered now: Send the values in debug4_value[] coded
	// as following pulse patterns:
	// 2 (or 3 or 4) index-bits + 8 value-bits + footer
	// where a 1-bit is 1000 us, a 0-bit 100 us.
	// Each sent debug4_value[] is reset to zero.
	// If all debug4_value are sent, debug4_output_done() is called.

	if((debug4_mask & debug4_idx_mask) == debug4_idx_mask) {
		// Send the bits of debug4_idx value.
		debug4_mask &= ~debug4_idx_mask;
		debug4_send_bit_pulse( debug4_idx & debug4_mask );
		if((debug4_mask >>= 1) != 0) {
			// More index bits.
			debug4_mask |= debug4_idx_mask;
		} else {
			// All index bits sent. Send debug4_value[debug4_idx].
			debug4_mask = 1 << 7;
		}
	} else if(debug4_mask == 3) {
		// Send train-footer and go for the next debug4_idx.
		send_single_pulse(rdiv10(5500));
		if(++debug4_idx < countof(debug4_value)) {
			debug4_mask = debug4_idx_start_mask;
		} else {
			// All debug4_value sent. Just terminate our
			// above sent pulse:
			debug4_idx = 0;
			debug4_mask = 5;
		}
	} else if(debug4_mask == 5) {
		send_single_pulse(rdiv10(5500));
		debug4_mask = 0;
		debug4_output_done();
	} else if(debug4_mask) {
		debug4_send_bit_pulse( debug4_value[debug4_idx] & debug4_mask );
		if((debug4_mask >>= 1) == 0) {
			debug4_value[debug4_idx] = 0;
			debug4_mask = 3;
		}
	} else
	//----------------------------------------------------------------------------------
#endif
			send_continue_pulse(rdiv10(500000));
		}
	}
}

void filter_generate_test_signal(register uint8_t pin_change) {

	if(! pin_change) {
		// The test signal are pulses of 2000, 1000, 500, 250, 120, 60 microseconds
		// each sent 4 times, plus 2 pulses of 6000 microseconds.
		// Filter is re-using global lsb and dix variables!

		// Note: In previous version 4 the terminating pulse was 5500 microseconds
		// and it was only 1 pulse (not 2). But 5500 us were bad to measure with
		// (at least my) oscilloscope and just 1 pulse makes the test signal an
		// odd number of pulses. Since each pulse toggles, an odd number of pulses
		// has the poor effect that every 2nd pulse sequence has opposite hi/lo
		// values wich is not (or very hard) to trigger with (my) oscilloscope.
		// So it will be an even number now and each test pulse sequence
		// definitively starts with a hi pulse that easily can be trigged on.
		// BTW: This way I found out that firmware generates pulses very
		// accurate // and that jitter in pulse durations, that are measured
		// by raspi's pilight (and can be displayed by pilight-raw), is very
		// likely a problem of pilight's pulse measuring implementation.

		uint16_t pulse = lsb;	// "lsb" * 10 us
		if(pulse == 0) {
			pulse = 600;
		}
		send_single_pulse(pulse);
		if((--dix & 3) == 0) {
			// lsb of 0 is mapped to 600 (is not storable in 8-bit lsb).
			lsb = lsb ? (lsb >> 1) < 6 ? 0 : (lsb >> 1) : 200;
			if(lsb == 0) {
				send_on();
				dix = 2;
			} else {
				dix = 4;
			}
		}

	} // else NOP
}

ISR(PCINT0_vect){
	static uint8_t pin_state;
	uint8_t pin_change = pin_state;
	pin_change ^= (pin_state = PINB);

	if(pin_change & receiver_pin_mask) {
		if(pin_change_in_10_us < 0) {
			// Old LPF method: immediate pin change processing.
			execute_filter(pin_change);
			recving_since_10_us = 0;
		} else
			// New LPF method has a different implementation of the
			// low pass filter. It is able to completely filter
			// away a pulse spike and works transparently to and
			// for all filter methods. The high pass filtering is
			// stil  done within filter methods.
		if(pin_change_in_10_us == 0) {
			// The previous pin change (if any) was processed.
			// Delay processing of the current pin change a litte
			// so we can check if this is the start of a spike and
			// should be ignored.
			pin_change_in_10_us = PIN_CHANGE_DELAY_US;
		} else {
			// Pin change too early! The previous pin change was not
			// yet processed. Ignore the previous and the current
			// pin change (by doing as if both are processed) because
			// both are part of a spike.
			pin_change_in_10_us = 0;
		}
	}

	if(GETBIT(pin_change, I_PIN(FW_CONTROL))) {
		firmware_control(pin_change);
	}
}
