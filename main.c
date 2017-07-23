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
/* spi slave                                    */

volatile struct spi_transfer {
  uint8_t *buf;
  uint8_t len;
  uint8_t *cur;
} spi_transfer;

void spi_setup() {
  // three wire mode, external positive edge clock
  USICR |= (1 << USIWM0) | (1 << USICS1);

  DDRB |= (1 << PB6); // DO as output
  DDRB &= ~(1 << PB5); // DI as input
  DDRB &= ~(1 << PB7); // USCK as input

  DDRD &= ~(1 << PD2); // CS as input
  DDRD |= (1 << PD4);

  // enable INT0 on any edge
  MCUCR |= (1 << ISC00);
  GIMSK |= (1 << INT0);
}

#define spi_write_next() do { USIDR = *spi_transfer.cur++; } while (0)

ISR(INT0_vect) {
  if (PIND & (1 << PD2)) {
    // end of transmission
    // turn off ovf interrupt
    USICR &= ~(1 << USIOIE);
  } else {
    // start of a transmission
    // turn on ovf interrupt
    USICR |= (1 << USIOIE);
    USISR |= (1 << USIOIF); // clear ovf flag

    PORTD &= ~(1 << PD4);

    // set the 4 bit counter to 0
    USISR &= ~0x0F;

    spi_transfer.cur = spi_transfer.buf;
    spi_write_next();
  }
}

ISR(USI_OVERFLOW_vect) {
  PORTD ^= (1 << PD4);
  if (spi_transfer.cur < spi_transfer.buf + spi_transfer.len) {
    spi_write_next();
  } else {
    USIDR = 0;
  }

  USISR |= (1 << USIOIF); // clear ovf flag
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

#define BBQTEMP_FLAG_STARTING           0

struct bbqtemp_packet {
  uint16_t magic;
  uint16_t temp1;
  uint16_t temp2;
  uint16_t sender_id;
  uint8_t flags;
};

// last input capture time
volatile uint16_t t_ic = 0;

// +/- pulse width
#define T_LONG                          (CYCLES_US(500))
#define T_SHORT                         (CYCLES_US(250))

// last received bit
volatile uint8_t last_bit = 0;

// rx buffer
#define RXBUF_SIZE                      13
volatile struct {
  uint8_t buf[RXBUF_SIZE];

  // current bit in buffer
  uint8_t buf_i;
} rf_receive;

// flag for isr
volatile uint8_t had_edge = 0;

// state of the state machine
volatile uint8_t state = 0;
// preamble cycle counter
volatile uint8_t preamble_count = 0;

// packet for sending out
volatile struct bbqtemp_packet last_packet;

void reset_state() {
  // reset everything for the decoder
  state  = STATE_WAITING;
  had_edge = 0;
  preamble_count = 0;
  rf_receive.buf_i = 0;
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
// a > 0.75b && a < 1.25b
uint8_t t_eq(uint16_t a, uint16_t b) {
  uint16_t margin = (b >> 2) + (b >> 3);
  return a > (b - margin) && a < (b + margin);
}

void store_bit() {
  uint8_t i = rf_receive.buf_i >> 3; // div 8
  uint8_t bit = 7 - (rf_receive.buf_i - (i << 3));

  if (i > RXBUF_SIZE) {
    // buffer overflow, stop
    return;
  }

  if (last_bit) {
    rf_receive.buf[i] |= (1 << bit);
  } else {
    rf_receive.buf[i] &= ~(1 << bit);
  }

  manch_dbg((last_bit << 4) | bit);
  manch_dbg(i);

  rf_receive.buf_i++;
}

uint8_t decode_quatenary_value(uint8_t quat) {
  switch (quat) {
  default: // fallthrough
  case 0x5:
    return 0;
  case 0x6:
    return 1;
  case 0x9:
    return 2;
  case 0xA:
    return 3;
  }
}

void decode_quatenary_coding(uint8_t *out) {
  uint8_t i;

  for (i = 0; i < RXBUF_SIZE; i++) {
    uint8_t val = rf_receive.buf[i];
    uint8_t nibble = (decode_quatenary_value(val >> 4) << 2) |
      decode_quatenary_value(val & 0x0F);

    if ((i & 1) == 0) {
      // upper nibble
      nibble <<= 4;
    }

    out[i >> 1] |= nibble;
  }
}

// checksum calculation based on Nibbler's (Sebastians) work
// see https://forums.adafruit.com/viewtopic.php?f=8&t=25414&sid=e1775df908194d56692c6ad9650fdfb2&start=15#p321384
uint16_t calculate_checksum(uint8_t *buf) {
  // initial value of linear feedback shift register
  uint16_t mask = 0x3331;
  uint16_t cksum = 0x0;
  uint8_t i;

  uint32_t data = ((uint32_t)buf[0] & 0xF) << 20 |
    (uint32_t)buf[1] << 12 |
    (uint32_t)buf[2] << 4 |
    ((uint32_t)buf[3] & 0xF0) >> 4;

  for (i = 0; i < 24; i++) {
    if ((data >> i) & 0x01) {
      // data bit at current position is "1"
      // do XOR with mask
      cksum ^= mask;
    }

    uint8_t msb = (mask >> 15) & 0x1;
    mask <<= 1;
    if (msb == 1) {
      // msb is set, toggle pattern for feedback bits
      mask ^= 0x1021;
    }
  }

  return cksum;
}

void decode_et733() {
  uint8_t buf[RXBUF_SIZE / 2 + 1];
  uint8_t i;

  i = sizeof(buf);
  while (i--) {
    buf[i] = 0;
  }
  decode_quatenary_coding(buf);

  for (i = 0; i < sizeof(buf); i++) {
    uart_putc(buf[i]);
  }

  if (!(buf[0] == 0xFA && buf[1] >> 4 == 0x8)) {
    // invalid header
    return;
  }

  uint8_t starting = (buf[1] & 0xF) == 0x7;

  uint16_t temp1 = (((uint16_t)buf[2] << 8) | buf[3]) >> 6;
  uint16_t temp2 = (((uint16_t)buf[3] & 0x3F) << 4) | (buf[4] >> 4);

  uint16_t orig_cksum = (uint16_t)(buf[4] & 0xF) << 12 | (uint16_t)buf[5] << 4 | buf[6] >> 4;

  uint16_t cksum = calculate_checksum(&buf[1]);
  uint16_t sender_id = orig_cksum ^ cksum;

  last_packet.temp1 = temp1;
  last_packet.temp2 = temp2;
  last_packet.sender_id = sender_id;
  last_packet.flags = (starting ? (1 << BBQTEMP_FLAG_STARTING) : 0);

  uart_putc(temp1 >> 8);
  uart_putc(temp1 & 0xFF);

  uart_putc(temp2 >> 8);
  uart_putc(temp2 & 0xFF);

  uart_putc(sender_id >> 8);
  uart_putc(sender_id & 0xFF);

  uart_putc(cksum >> 8);
  uart_putc(cksum & 0xFF);

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

  if (rf_receive.buf_i >= 104) {
    manch_dbg(0xEE); // some byte for easier triggering
    for (uint8_t i = 0; i < RXBUF_SIZE; i++) {
      manch_dbg(rf_receive.buf[i]);
    }

    decode_et733();

    reset_state();
  }
}

int main() {
  last_packet.magic = 0xE733;

  uart_setup();
  spi_setup();

  spi_transfer.buf = (uint8_t *)&last_packet;
  spi_transfer.len = sizeof(struct bbqtemp_packet);

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
