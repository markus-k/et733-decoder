#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

/************************************************/
/* UART for debugging                           */

void uart_setup() {
  // setup uart, 250k baud@4Mhz
  DDRD |= (1 << PD1);

  // double speed
  //UCSRA |= (1 << U2X);

  UCSRB = (1 << TXEN);
  UCSRC = (1 << UCSZ0) | (1 << UCSZ1);
  UBRRH = 0;
  UBRRL = 0;
}

void uart_putc(uint8_t v) {
  while (!(UCSRA & (1 << UDRE))) {}
  UDR = v;
}

/************************************************/
/* timer utils                                  */

#define CYCLES_US(t)    ((F_CPU / 1000000UL) * (t))

#define RISE 1
#define FALL 0

void set_ic_edge(uint8_t rising) {
  if (rising) {
    TCCR1B |= (1 << ICES1);
  } else {
    TCCR1B &= ~(1 << ICES1);
  }
}

void toggle_ic_edge() {
  TCCR1B ^= (1 << ICES1);
}

void timer_setup() {
  // setup timer1, 122Hz
  TCCR1B |= (1 << WGM12) | (1 << CS10);
  uint16_t ocr1 = 0x8000;
  OCR1H = ocr1 >> 8;
  OCR1AL = ocr1;

  TIMSK |= (1 << ICIE1) | (1 << OCIE1A);
}

void timer_restart() {
  TCNT1H = 0;
  TCNT1L = 0;
}

/************************************************/
/* program                                      */

#define MANCHESTER_DEBUG

#define STATE_WAITING                   0
#define STATE_PREAMBLE_START            1
#define STATE_PREAMBLE_END              2
#define STATE_MANCH_DECODING            4
#define STATE_MANCH_SAME_CONFIRM        5

// last input capture time
volatile uint16_t t_ic = 0;

// +/- pulse width
#define T_LONG                          (CYCLES_US(500))
#define T_SHORT                         (CYCLES_US(250))

// last received bit
volatile uint8_t last_bit = 0;

// rx buffer
#define RXBUF_SIZE                      13
volatile uint8_t buf[RXBUF_SIZE];
// current bit in buffer
volatile uint8_t buf_i = 0;

// flag for isr
volatile uint8_t had_edge = 0;

// state of the state machine
volatile uint8_t state = 0;
// preamble cycle counter
volatile uint8_t preamble_count = 0;

void reset_state() {
  state  = STATE_WAITING;
  had_edge = 0;
  preamble_count = 0;
  buf_i = 0;
  last_bit = 1;
  set_ic_edge(RISE);
}

ISR(TIMER1_CAPT_vect) {
  uint16_t l = ICR1L;
  uint16_t h = ICR1H;
  t_ic = (h << 8) | l;
  had_edge = 1;

  timer_restart();
}

ISR(TIMER1_COMPA_vect) {
  // timeout, reset
  reset_state();
}

#ifdef MANCHESTER_DEBUG
#define manch_dbg(a) uart_putc(a)
#else
#define manch_dbg(a)
#endif

// roughly equal function
// a > 0.7b && a < 1.3b
uint8_t t_eq(uint16_t a, uint16_t b) {
  uint16_t margin = (b >> 2) + (b >> 3);
  return a > (b - margin) && a < (b + margin);
}

void store_bit() {
  uint8_t i = buf_i >> 3; // div 8
  uint8_t bit = 7 - (buf_i - (i << 3));

  if (i > RXBUF_SIZE) {
    // buffer overflow, stop
    return;
  }

  if (last_bit) {
    buf[i] |= (1 << bit);
  } else {
    buf[i] &= ~(1 << bit);
  }

  manch_dbg((last_bit << 4) | bit);
  manch_dbg(i);

  buf_i++;
}


uint8_t decode_to_quatenary(uint8_t quat) {
  switch (quat) {
  case 0x5:
    return 0;
  case 0x6:
    return 1;
  case 0x9:
    return 2;
  case 0xA:
    return 3;
  default:
    return 0;
  }
}

uint16_t decode_10bit_value_lower(uint8_t *ptr) {
  uint16_t res = 0;

  res |= (uint16_t)decode_to_quatenary(*ptr >> 4) << 8;
  res |= (uint16_t)decode_to_quatenary((*ptr++) & 0x0F) << 6;
  res |= (uint16_t)decode_to_quatenary(*ptr >> 4) << 4;
  res |= (uint16_t)decode_to_quatenary((*ptr++) & 0x0F) << 2;
  res |= (uint16_t)decode_to_quatenary(*ptr >> 4);

  return res;
}

uint16_t decode_10bit_value_upper(uint8_t *ptr) {
  uint16_t res = 0;

  res |= (uint16_t)decode_to_quatenary((*ptr++) & 0x0F) << 8;
  res |= (uint16_t)decode_to_quatenary(*ptr >> 4) << 6;
  res |= (uint16_t)decode_to_quatenary((*ptr++) & 0x0F) << 4;
  res |= (uint16_t)decode_to_quatenary(*ptr >> 4) << 2;
  res |= (uint16_t)decode_to_quatenary(*ptr & 0x0F);

  return res;
}

void decode_et733() {
  uint8_t *ptr = (uint8_t *)buf;
  uint8_t starting = 0;

  if (!(*ptr++ == 0xAA && *ptr++ == 0x99 && *ptr++ == 0x95)) {
    // invalid header
    return;
  }

  starting = *ptr++ == 0x6A;

  uint16_t temp1 = decode_10bit_value_lower(ptr);
  ptr += 2;
  uint16_t temp2 = decode_10bit_value_upper(ptr);

  uart_putc(temp1 >> 8);
  uart_putc(temp1 & 0xFF);

  uart_putc(temp2 >> 8);
  uart_putc(temp2 & 0xFF);

  uart_putc(starting);
}

void decode_manchester() {
  /*manch_dbg(t_ic >> 8);
    manch_dbg(t_ic & 0xFF);*/

  switch (state) {
  case STATE_WAITING:
    // we have a rising edge, let's see what we get next
    state = STATE_PREAMBLE_START;
    set_ic_edge(FALL);
    break;

  case STATE_PREAMBLE_START:
    // we had a positive edge, now a falling edge
    if (t_ic > CYCLES_US(200) && t_ic < CYCLES_US(1200)) {
      // first positive pulse was within expected width
      state = STATE_PREAMBLE_END;
      set_ic_edge(RISE);
    } else {
      // thats a weird one, restart
      reset_state();
    }
    break;

  case STATE_PREAMBLE_END:
    if (t_ic > CYCLES_US(4500) && t_ic < CYCLES_US(5500)) {
      // looks like we have a preamble pulse
      if (preamble_count >= 7) {
	state = STATE_MANCH_DECODING;
	store_bit();
      } else {
	state = STATE_PREAMBLE_START;
	preamble_count++;
      }

      set_ic_edge(FALL);
    } else {
      // not within expected pulse width
      reset_state();
    }
    break;

  case STATE_MANCH_DECODING:
    if (t_eq(t_ic, T_SHORT)) {
      // short pulse, confirm next is short too
      state = STATE_MANCH_SAME_CONFIRM;
      toggle_ic_edge();
    } else if (t_eq(t_ic, T_LONG)) {
      // long pulse, store inverse of last bit
      last_bit = !last_bit;
      store_bit();
      toggle_ic_edge();
    } else {
      // this is wrong
      reset_state();
    }
    break;

  case STATE_MANCH_SAME_CONFIRM:
    if (t_eq(t_ic, T_SHORT)) {
      // pulse was short too, store bit
      store_bit();
      state = STATE_MANCH_DECODING;
      toggle_ic_edge();
    } else {
      // error, pulse was long or broken
      reset_state();
    }
    break;
  }

  manch_dbg(preamble_count);
  manch_dbg(state);

  if (buf_i >= 104) {
    manch_dbg(0xEE);
    for (uint8_t i = 0; i < RXBUF_SIZE; i++) {
      uart_putc(buf[i]);
    }

    decode_et733();

    reset_state();
  }
}

int main() {
  uart_setup();

  timer_setup();

  reset_state();

  sei();

  while (1) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();

    if (had_edge) {
      decode_manchester();

      had_edge = 0;
    }
  }

  return 0;
}
