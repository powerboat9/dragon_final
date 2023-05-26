//*****************************************************************************
//*****************************    C Source Code    ***************************
//*****************************************************************************
//
//  DESIGNER NAME:  Owen Avery
//
//   PROJECT NAME:  Final
//
//      FILE NAME:  main.c
//
//-----------------------------------------------------------------------------
//
// DESCRIPTION:
//    This code emulates a microwave on the Dragon12-Plus Light board. The
//    emulated microwave has a built in cook timer, power level adjustment,
//    door latch, night light, turntable, distance sensor, keypad, clock,
//    and serial connection. Supplimental documentation is included alongside
//    the program source code. Global state usage is minimized and modularized.
//
//*****************************************************************************
//*****************************************************************************
#include <hidef.h>      /* common defines and macros */
#include <mc9s12dg256.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12dg256b"

#include "main_asm.h" /* interface to the assembly module */

// some general timing constants
#define BUS_FREQUENCY 24000000
#define RTI_PERIOD_MS 10.24
#define RTI_PER_SECOND ((int) (1000.0 / RTI_PERIOD_MS + 0.5))
#define RTI_PER_MINUTE ((int) (60000.0 / RTI_PERIOD_MS + 0.5))

// less general timing constants
// but they are heavily dependent on each other
// and the general timing constants
#define TIMER_FREQUENCY 1500000
// used to set timer frequency relative to bus clock
#define TIMER_PRESCALE 0x04

// declare some functions here
// so we can send status messages throughout the code
char serial_transmit(char *str);
// and pause the clock in tmod_* functions
void clock_timer_set_state(char state);

// required for clock_timer_set_state() usage
// drawing clock
#define CLOCK_TIMER_STATE_CLOCK 0
// not drawing, timer/clock setting code has taken over
#define CLOCK_TIMER_STATE_NO_DRAW 1
// "INVALID INPUT"
#define CLOCK_TIMER_STATE_INVALID 2
// timer running
#define CLOCK_TIMER_STATE_TIMING 3
// timer paused
#define CLOCK_TIMER_STATE_PAUSED 4
// "ENJOY"
#define CLOCK_TIMER_STATE_ENJOY 5

/* LCD BUFFERING */

// addresses used for writing to lcd
#define LCD_ADDR_TOP 0x00
#define LCD_ADDR_BOTTOM 0x40

#define LCD_ROW_WIDTH 16

// used to buffer lcd writes
struct lcd_buffer {
  char buf[LCD_ROW_WIDTH];
  char dirty_min_pos; // inclusive
  char dirty_max_pos; // exclusive
  char addr; // LCD_ADDR_TOP/BOTTOM
};

// the two lcd buffers, top and bottom rows
static struct lcd_buffer lcd_buf_top_row = {
  {
  ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' '
  },
  0,
  0,
  LCD_ADDR_TOP
};
static struct lcd_buffer lcd_buf_bottom_row = {
  {
  ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' '
  },
  0,
  0,
  LCD_ADDR_BOTTOM
};

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are enabled and initializes
//    the lcd for buffering.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void lcd_buf_init() {
  lcd_init();
  clear_lcd();
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and sets a
//    character in the given lcd buffer. In doing so, it makes sure to mark
//    that section of the buffer dirty.
//
// INPUT PARAMETERS:
//    buf - a pointer to the lcd buffer
//    val - the value of the character to set
//    pos - the position to set it at
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void lcd_buf_putc(struct lcd_buffer *buf, char val, char pos) {
  if (buf->buf[pos] != val) {
    // we are making a change
    buf->buf[pos] = val;
    
    if (buf->dirty_min_pos == buf->dirty_max_pos) {
      // no dirty bytes - we have the only dirty byte
      buf->dirty_min_pos = pos;
      buf->dirty_max_pos = pos + 1;
    } else {
      if (buf->dirty_min_pos > pos) {
        // grow dirty range down
        buf->dirty_min_pos = pos;
      } else if (buf->dirty_max_pos <= pos) {
        // grow dirty range up
        buf->dirty_max_pos = pos + 1;
      }
    }
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and clears the
//    rightmost section of an lcd buffer.
//
// INPUT PARAMETERS:
//    buf - a pointer to the lcd buffer
//    pos - the starting position in the buffer, inclusive, from which
//          the lcd buffer should be cleared.
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void lcd_buf_clear_right(struct lcd_buffer *buf, char pos) {
  for (; pos < LCD_ROW_WIDTH; pos++) lcd_buf_putc(buf, ' ', pos);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are enabled and attempts to
//    flush a character in the given lcd buffer to the lcd display.
//
// INPUT PARAMETERS:
//    buf - a pointer to the lcd buffer
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    0 if the lcd display was not written to, 1 if it was
//
//-----------------------------------------------------------------------------
char lcd_buf_flush_step_row(struct lcd_buffer *buf) {
  // where the lcd cursor is
  // used to determine set_lcd_addr() calls
  static char lcd_pos = LCD_ADDR_TOP;

  char write;

  // disable interrupts while checking global state
  asm sei;
  if (buf->dirty_min_pos == buf->dirty_max_pos) {
    // reenable interrupts, return early, we have nothing to do
    asm cli;
    return 0;
  }
  
  // verify lcd cursor correctly positioned
  while ((buf->addr + buf->dirty_min_pos) != lcd_pos) {
    lcd_pos = buf->addr + buf->dirty_min_pos;
    asm cli;
    // enable interrupts while sending lcd instruction
    set_lcd_addr(lcd_pos);
    asm sei;
    // loop again, to verify that nothing has
    // changed because of interrupts
  }

  // adjust state for after next write
  write = buf->buf[buf->dirty_min_pos++];
  lcd_pos++;
  // reenable interrupts and perform write
  asm cli;
  data8(write);
  
  return 1;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are enabled and attempts to
//    flush a character in the global lcd buffers to the lcd display.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    0 if the lcd display was not written to, 1 if it was
//
//-----------------------------------------------------------------------------
char lcd_buf_flush_step() {
  return
    lcd_buf_flush_step_row(&lcd_buf_top_row) ||
    lcd_buf_flush_step_row(&lcd_buf_bottom_row);
}

/* DISPLAY FUNCTIONS */

// used to format time
#define DISPLAY_TIME_DIGIT_CNT 4
#define DISPLAY_TIME_SIZE 5
#define DISPLAY_TIME_COLON_POS 2

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and writes a time
//    aligned to the left in the top row lcd buffer. The time is given as
//    (hh * 60 + ll) and displayed as "hh:ll".
//
//    EX: 59 gives "00:59"
//       160 gives "02:40"
//
// INPUT PARAMETERS:
//    time - a time in the above described format
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void display_time(unsigned int time) {
  // used to calculate and position time digits
  // most significant digits handled first
  const static int div_tbl[] = {600, 60, 10, 1};
  const static char pos_tbl[] = {0, 1, 3, 4};

  char pos;

  for (pos = 0; pos < DISPLAY_TIME_DIGIT_CNT; pos++) {
    char digit;

    // calculate digit
    digit = (char) (time / div_tbl[pos]);
    // reduce time
    time -= digit * div_tbl[pos];
    // write to lcd buffer
    lcd_buf_putc(&lcd_buf_top_row, digit + '0', pos_tbl[pos]);
  }
  
  // insert colon and clear anything to the right
  lcd_buf_putc(&lcd_buf_top_row, ':', DISPLAY_TIME_COLON_POS);
  lcd_buf_clear_right(&lcd_buf_top_row, DISPLAY_TIME_SIZE);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and writes a
//    message aligned to the left in the top row lcd buffer.
//
// INPUT PARAMETERS:
//    msg - a message to display
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void display_msg(char *msg) {
  char pos;

  // write message
  for (pos = 0; msg[pos]; pos++) lcd_buf_putc(&lcd_buf_top_row, msg[pos], pos);
  // clear to the right
  lcd_buf_clear_right(&lcd_buf_top_row, pos);
}

// used to format temperature
#define DISPLAY_TEMP_DIGITS 5
#define DISPLAY_TEMP_MAX_SIZE (DISPLAY_TEMP_DIGITS + 2)

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and writes a
//    temperature aligned to the left in the bottom row lcd buffer.
//
// INPUT PARAMETERS:
//    temp - a temperature in fahrenheit to display
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void display_temp(unsigned int temp) {
  const static int div_tbl[] = {10000, 1000, 100, 10, 1};
  // seperate positions, since the temperature could
  // be a variable number of digits
  char pos = 0;
  char draw_pos = 0;
  
  // skip extra leading zeros
  while ((temp < div_tbl[pos]) & (pos != (DISPLAY_TEMP_DIGITS - 1))) {
    pos++;
  }

  for (;pos < DISPLAY_TEMP_DIGITS; pos++) {
    char digit;

    // display digits
    digit = (char) (temp / div_tbl[pos]);
    temp -= digit * div_tbl[pos];
    lcd_buf_putc(&lcd_buf_bottom_row, digit + '0', draw_pos++);
  }
  
  // display units
  lcd_buf_putc(&lcd_buf_bottom_row, ' ', draw_pos++);
  lcd_buf_putc(&lcd_buf_bottom_row, 'F', draw_pos++);
  
  // clear to the right
  // no power values were harmed in the writing
  // of this display buffer
  while (draw_pos < DISPLAY_TEMP_MAX_SIZE) {
    lcd_buf_putc(&lcd_buf_bottom_row, ' ', draw_pos++);
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and writes a
//    power value aligned to the right in the bottom row lcd buffer. Power is
//    formated as tens of percents.
//
//    EX: 1 gives  "10%"
//        5 gives  "50%"
//       10 gives "100%"
//
// INPUT PARAMETERS:
//    power - a power value in the above described format
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void display_power(unsigned char power) {
  char pos = LCD_ROW_WIDTH - 1;

  lcd_buf_putc(&lcd_buf_bottom_row, '%', pos--);
  if (power == 10) {
    // add extra zero
    lcd_buf_putc(&lcd_buf_bottom_row, '0', pos--);
    // then divide power by 10
    power = 1;
  }
  lcd_buf_putc(&lcd_buf_bottom_row, '0', pos--);
  lcd_buf_putc(&lcd_buf_bottom_row, power + '0', pos--);
  // clear any extra '1' from displaying "100%"
  lcd_buf_putc(&lcd_buf_bottom_row, ' ', pos);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and clears the
//    lower lcd buffer.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void display_lower_clear() {
  lcd_buf_clear_right(&lcd_buf_bottom_row, 0);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and draws an input
//    prompt on the top row lcd buffer.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void display_input_start() {
  char pos;

  pos = 0;
  lcd_buf_putc(&lcd_buf_top_row, '>', pos++);
  lcd_buf_clear_right(&lcd_buf_top_row, pos);
}

// used to draw inputted digits
#define DISPLAY_INPUT_POS_TRIGGER_COLON 2
#define DISPLAY_INPUT_COLON_POS 4

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and echos digits
//    inputted to the lcd buffer.
//
// INPUT PARAMETERS:
//    digit - the digit, 0-9 (not ascii), to write
//    pos - the position, 0-3 (most to least significant) to write to
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void display_input_pos(char digit, char pos) {
  // lcd buffer position lookup table
  const static char pos_tbl[] = {2, 3, 5, 6};

  if (pos == DISPLAY_INPUT_POS_TRIGGER_COLON) {
    // it is time to draw the colon
    lcd_buf_putc(&lcd_buf_top_row, ':', DISPLAY_INPUT_COLON_POS);
  }
  
  lcd_buf_putc(&lcd_buf_top_row, digit + '0', pos_tbl[pos]);
}     

/* ABORT BUTTON/LAMP */

#define ABORT_MASK 0x20
#define LAMP_MASK 0x10

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function initializes the abort button and night light/lamp.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void abort_lamp_init() {
  DDRM = LAMP_MASK;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function checks if the abort button is currently pressed. It should
//    be polled only occasionally (ex: during an RTI) to automatically
//    debounce.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    1 if the button is pressed, 0 otherwise
//
//-----------------------------------------------------------------------------
char abort_is_pressed() {
  return !(PTM & ABORT_MASK);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function sets the night light/lamp on.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void lamp_set_on() {
  PTM |= LAMP_MASK;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function sets the night light/lamp off.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void lamp_set_off() {
  PTM &= ~LAMP_MASK;
}

/* MOTOR PWM (TURNTABLE, DOOR LATCH) */

// turntable is on motor0
#define TURNTABLE_VAL_OFF 0
#define TURNTABLE_VAL_ON 150
#define TURNTABLE_MOTOR 0x01

#define TURNTABLE_PORT_B_OUT 0xff
#define TURNTABLE_PORT_B_VAL 0x01

#define LATCH_VAL_OPEN 3300
#define LATCH_VAL_LOCK 6500

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function initializes the turntable and door latch.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void turntable_latch_init() {
  // enable motors in default state
  // turntable
  motor0_init();
  motor0(TURNTABLE_VAL_OFF);
  // latch
  servo76_init();
  set_servo76(LATCH_VAL_OPEN);
  // enable h-bridge for motor0 (turntable)
  DDRB = TURNTABLE_PORT_B_OUT;
  PORTB = TURNTABLE_PORT_B_VAL;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function sets the door latch and turntable for microwave
//    activation.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void turntable_latch_on() {
  motor0(TURNTABLE_VAL_ON);
  set_servo76(LATCH_VAL_LOCK);
  // log microwave activation
  serial_transmit("Activating Microwave\r\n");
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function sets the door latch and turntable for microwave
//    deactivation.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void turntable_latch_off() {
  motor0(TURNTABLE_VAL_OFF);
  set_servo76(LATCH_VAL_OPEN);
  // log microwave deactivation
  serial_transmit("Deactivating Microwave\r\n");
}

/* HANDLES TIMER MODULE (distance sensor, speaker) */

// not making noise
#define TMOD_STATE_OFF 0
// not making noise, monitoring distance
#define TMOD_STATE_MONITOR 1
// distance warning noise, monitoring distance
#define TMOD_STATE_WARN 2
// done, noise on
#define TMOD_STATE_DONE_ON 3
// done, noise off
#define TMOD_STATE_DONE_OFF 4

// current state
static char tmod_state = TMOD_STATE_OFF;

// main timer used for sound (CH5)
#define TMOD_CH_SOUND1 0x20
// secondary timer used for sound (CH7)
#define TMOD_CH_SOUND2 0x80
// timer used for distance sensor echo (CH3)
#define TMOD_CH_ECHO 0x08
// timer used for distance sensor trigger (CH4)
#define TMOD_CH_TRIGGER 0x10

#define TMOD_ENABLE 0x80

// used to setup output modes
#define TMOD_EDGE_TRIGGER_MODE_SET 0x02
#define TMOD_EDGE_TRIGGER_LEVEL 0x01

// used to setup input modes
#define TMOD_EDGE_ECHO_ANY 0xC0

// small delay before timer trigger
// used to enable and almost immediately
// trigger timer interrupt
#define TMOD_START_DELAY 2

// duration of trigger in timer ticks
// ~10 microseconds
#define TMOD_TRIGGER_TIME 15

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function initializes the timer module and
//    associated speaker/distance sensor. 
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void tmod_init() {
  // set output compare on channels
  // for sound signals and TRIGGER
  TIOS = TMOD_CH_SOUND1 | TMOD_CH_SOUND2 | TMOD_CH_TRIGGER;
  // setup timer prescale
  TSCR2 = TIMER_PRESCALE;
  // ch7 (sound2) affects sound1 pin (ch5)
  // sets low by default
  OC7M = TMOD_CH_SOUND1;
  // setup interrupts
  TIE = TMOD_CH_ECHO | TMOD_CH_SOUND1;
  TCTL4 = TMOD_EDGE_ECHO_ANY;
  // enable timer
  TSCR1 = TMOD_ENABLE;
}

// Hz
#define TMOD_FREQ_DIST 2500
#define TMOD_FREQ_DONE 1250
// timer ticks/half period
#define TMOD_TONE_DIST \
  (TIMER_FREQUENCY / TMOD_FREQ_DIST / 2)
#define TMOD_TONE_DONE \
  (TIMER_FREQUENCY / TMOD_FREQ_DONE / 2)

// 0.5 seconds
#define TMOD_DONE_TOGGLE (TMOD_FREQ_DONE >> 1)

// used to produce pulsing done tone
static int tmod_toggle_cnt;

// milimeters per second
#define TMOD_SPEED_SOUND 340000
// milimeters
#define TMOD_SAFE_DIST 250
// threshold for safe distance sensor trigger
#define TMOD_SAFE_THRESH (\
  ((float) TMOD_SAFE_DIST) / TMOD_SPEED_SOUND\
                           * TIMER_FREQUENCY\
                           * 2\
                         )

// current noise pitch
static int tmod_pitch;

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This interrupt handler triggers on a sound1 compare, and sets up the
//    next tone generation period.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void interrupt 13 tmod_handle_sound_int() {
  switch (tmod_state) {
    case TMOD_STATE_DONE_ON:
    case TMOD_STATE_DONE_OFF:
      if (!--tmod_toggle_cnt) {
        // toggle sound2 setting
        OC7D ^= TMOD_CH_SOUND1;
        // reset toggle counter
        tmod_toggle_cnt = TMOD_DONE_TOGGLE;
      }
  }
  
  tone(tmod_pitch);
  
  TFLG1 = TMOD_CH_SOUND1;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function turns on the speaker with a given pitch.
//
// INPUT PARAMETERS:
//    pitch - the given pitch, in timer ticks per half period.
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void tmod_enable_sound(int pitch) {
  int tcnt;

  tmod_pitch = pitch;
  // setup toggling
  // does nothing if not in TMOD_STATE_DONE_*
  tmod_toggle_cnt = TMOD_DONE_TOGGLE;
  // schedule compares
  tcnt = TCNT;
  TC5 = tcnt;
  TC7 = tcnt;
  // sound2 compare sets sound1 output high
  OC7D = TMOD_CH_SOUND1;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function sets the timer module state, updating the current playing
//    tone as necessary.
//
// INPUT PARAMETERS:
//    new_state - the TMOD_STATE_* to set the current state to
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void tmod_set_state(char new_state) {
  if (tmod_state == new_state) return;

  switch (tmod_state = new_state) {
    case TMOD_STATE_OFF:
    case TMOD_STATE_MONITOR:
    case TMOD_STATE_DONE_OFF:
      // sound2 compare sets sound1 output low
      // this keeps the speaker low
      OC7D = 0;
      break;
    case TMOD_STATE_WARN:
      // enable distance alarm
      tmod_enable_sound(TMOD_TONE_DIST);
      break;
    case TMOD_STATE_DONE_ON:
      tmod_enable_sound(TMOD_TONE_DONE);
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This interrupt handles rising/falling edges on the "EDGE" output of the
//    distance sensor, keeping track of (and updating state based on) the
//    calculated distance.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void interrupt 11 tmod_handle_echo() {
  static unsigned int last_val;
  static char has_last;
  
  TFLG1 = TMOD_CH_ECHO;
  
  switch (tmod_state) {
    case TMOD_STATE_MONITOR:
    case TMOD_STATE_WARN:
      // distance sensor is on
      // continue
      break;
    default:
      // distance sensor is off
      // return early
      has_last = !has_last;
      return;
  }

  if (has_last) {
    unsigned int dif = TC3 - last_val;

    if (dif < TMOD_SAFE_THRESH) {
      // warn the user and pause timer
      tmod_set_state(TMOD_STATE_WARN);
      clock_timer_set_state(CLOCK_TIMER_STATE_PAUSED);
    } else {
      // make sure timer is running
      tmod_set_state(TMOD_STATE_MONITOR);
      clock_timer_set_state(CLOCK_TIMER_STATE_TIMING);
    }
  } else {
    last_val = TC3;
  }

  has_last = !has_last;
}

/*
 * "we suggest to use over 60ms measurement
 * cycle, in order to prevent trigger signal
 * to the echo signal" - author of HC-SR04 datasheet
 */
#define TMOD_RTI_PER_DIST_MEASURE 8

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called every RTI and is responsible for sending pulses
//    on the distance sensor "TRIGGER" input.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void tmod_rti_tick() {
  static char dist_cycle_counter;
  
  // early return if distance sensor not running
  switch (tmod_state) {
    case TMOD_STATE_MONITOR:
    case TMOD_STATE_WARN:
      break;
    default:
      return;
  }

  if (!dist_cycle_counter) {
    // send a pulse on TRIGGER
    
    // pull TRIGGER high
    TCTL1 |= TMOD_EDGE_TRIGGER_MODE_SET | TMOD_EDGE_TRIGGER_LEVEL;
    CFORC = TMOD_CH_TRIGGER;
    // pull TRIGGER low after delay
    TC4 = TCNT + TMOD_TRIGGER_TIME;
    TCTL1 &= ~TMOD_EDGE_TRIGGER_LEVEL;
  }
  if (++dist_cycle_counter == TMOD_RTI_PER_DIST_MEASURE) {
    dist_cycle_counter = 0;
  }
}

/* HANDLES TEMPERATURE/POTENTIOMETER/LIGHT SENSOR */

// adc channels for each sensor
#define AD0_TEMPERATURE 5
#define AD0_POTENTIOMETER 7
#define AD0_LIGHT 4

#define ADC_RANGE 1024
#define POWER_RANGE 10
#define ADC_POWER_BIN_SIZE 103

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called while interrupts are disabled and writes current
//    temperature and power level readings to the lcd buffer.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void temp_power_redraw() {
  // Celsius: cel = val >> 1
  // Fahrenheit: fah = cel * 9/5 + 32
  display_temp(ad0conv(AD0_TEMPERATURE) * 9 / 10 + 32);
  // 0-1023 -> 1-10
  display_power(
    ad0conv(AD0_POTENTIOMETER) / ADC_POWER_BIN_SIZE + 1
  );
}

#define ADC_DARK_THRESHOLD 100

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called every RTI and is responsible for controlling the
//    night light/lamp based on photosensor readings.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void photo_rti() {
  if (ad0conv(AD0_LIGHT) <= ADC_DARK_THRESHOLD) {
    lamp_set_on();
  } else {
    lamp_set_off();
  }
}

/* HANDLES CLOCK/TIMER */

// 24:00 - clock overflows to 00:00
#define CLOCK_OVERFLOW_VALUE (24 * 60)

// used to schedule clock/timer increments
static unsigned int clock_sched = RTI_PER_MINUTE;
static unsigned char timer_sched;
// used to store clock value in minutes
// note: static variables are initialized to 0
static unsigned int clock_val;
// used to store timer value in seconds
static unsigned int timer_val;

// 2 seconds
#define CLOCK_SCHED_REDRAW_TIME (RTI_PER_SECOND << 1)

// used to schedule clock redraw after "INVALID INPUT" display
static unsigned int clock_sched_redraw;

// state macros defined towards beginning of code
// so that tmod_* functions can use them
static char clock_timer_state = CLOCK_TIMER_STATE_CLOCK;

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function sets the clock/timer state, writing to the lcd buffer as
//    necessary.
//
// INPUT PARAMETERS:
//    state - the CLOCK_TIMER_STATE_* to set the current state to
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void clock_timer_set_state(char state) {
  // early return on noop
  if (clock_timer_state == state) return;
  
  switch (clock_timer_state = state) {
    case CLOCK_TIMER_STATE_CLOCK:
      display_time(clock_val);
      display_lower_clear();
      break;
    case CLOCK_TIMER_STATE_NO_DRAW:
      break;
    case CLOCK_TIMER_STATE_INVALID:
      clock_sched_redraw = CLOCK_SCHED_REDRAW_TIME;
      display_msg("INVALID INPUT");
      display_lower_clear();
      break;
    case CLOCK_TIMER_STATE_TIMING:
      display_time(timer_val);
      temp_power_redraw();
      break;
    case CLOCK_TIMER_STATE_PAUSED:
      display_msg("BACK AWAY");
      display_lower_clear();
      break;
    case CLOCK_TIMER_STATE_ENJOY:
      display_msg("ENJOY");
      display_lower_clear();
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function redraws the clock if the current state requires a clock
//    display.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void clock_register_update() {
  if (clock_timer_state == CLOCK_TIMER_STATE_CLOCK) {
    display_time(clock_val);
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function redraws the timer if the current state requires a timer
//    display.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void timer_register_update() {
  if (clock_timer_state == CLOCK_TIMER_STATE_TIMING) {
    display_time(timer_val);
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called every RTI and handles clock/timer change/redraw
//    scheduling.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void clock_timer_rti() {
  // clock
  if (!--clock_sched) {
    clock_sched = RTI_PER_MINUTE;
    if (++clock_val == CLOCK_OVERFLOW_VALUE) {
      clock_val = 0;
    }
    clock_register_update();
  }
  
  // handle timed/abort based state transitions
  switch (clock_timer_state) {
    case CLOCK_TIMER_STATE_INVALID:
      // wait for timeout on "INVALID INPUT" display
      if (!--clock_sched_redraw) {
        clock_timer_set_state(CLOCK_TIMER_STATE_CLOCK);
      }
      break;
    case CLOCK_TIMER_STATE_TIMING:
      // handle abort
      if (abort_is_pressed()) {
        clock_timer_set_state(CLOCK_TIMER_STATE_CLOCK);
        turntable_latch_off();
        tmod_set_state(TMOD_STATE_OFF);
        break;
      }
      // redraw temperature and power
      temp_power_redraw();
      // handle timer
      if (!--timer_sched) {
        timer_sched = RTI_PER_SECOND;
        if (--timer_val) {
          display_time(timer_val);
        } else {
          clock_timer_set_state(CLOCK_TIMER_STATE_ENJOY);
          turntable_latch_off();
          tmod_set_state(TMOD_STATE_DONE_ON);
        }
      }
      break;
    case CLOCK_TIMER_STATE_ENJOY:
      // handle abort
      if (abort_is_pressed()) {
        clock_timer_set_state(CLOCK_TIMER_STATE_CLOCK);
        tmod_set_state(TMOD_STATE_OFF);
      }
      break;
    case CLOCK_TIMER_STATE_PAUSED:
      // handle abort
      if (abort_is_pressed()) {
        clock_timer_set_state(CLOCK_TIMER_STATE_CLOCK);
        turntable_latch_off();
        tmod_set_state(TMOD_STATE_OFF);
      }
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function sets the current clock time.
//
// INPUT PARAMETERS:
//    time - time value, formated as described in display_time()
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void clock_set(unsigned int time) {
  clock_sched = RTI_PER_MINUTE;
  clock_val = time;
  clock_register_update();
  // do some logging
  serial_transmit("Clock Set\r\n");
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function starts the microwave and sets the cook timer.
//
// INPUT PARAMETERS:
//    time - time value, formated as described in display_time()
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void timer_start(unsigned int time) {
  timer_sched = RTI_PER_SECOND;
  timer_val = time;
  clock_timer_set_state(CLOCK_TIMER_STATE_TIMING);
  serial_transmit("Timer Started\r\n");
  turntable_latch_on();
  tmod_set_state(TMOD_STATE_MONITOR);
}

/* HANDLES SETTING CLOCK/TIMER */

// number which the keypad interprets a '*' as
#define KEYPAD_STAR 0xE
// number which the keypad interprets no button press as
#define KEYPAD_NONE 0x10

#define SET_MODE_NONE 0
#define SET_MODE_CLOCK 1
#define SET_MODE_TIMER 2
static char set_mode = SET_MODE_NONE;

#define SET_BUFFER_SIZE 4
static char set_buffer[SET_BUFFER_SIZE];
static char set_pos;

// maximum seconds/minutes/hours
#define SET_SEC_MAX 59
#define SET_MIN_MAX 59
#define SET_HR_MAX 23

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function checks if the time value currently being inputted is a
//    valid clock setting.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    0 if the time is invalid, 1 if it's valid
//
//-----------------------------------------------------------------------------
char set_is_ok_clock() {
  if ((set_buffer[0] * 10 + set_buffer[1]) > SET_HR_MAX) {
    return 0;
  }
  if ((set_buffer[2] * 10 + set_buffer[3]) > SET_MIN_MAX) {
    return 0;
  }
  return 1;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function checks if the time value currently being inputted is a
//    valid timer setting.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    0 if the time is invalid, 1 if it's valid
//
//-----------------------------------------------------------------------------
char set_is_ok_timer() {
  if ((set_buffer[0] * 10 + set_buffer[1]) > SET_MIN_MAX) {
    return 0;
  }
  if ((set_buffer[2] * 10 + set_buffer[3]) > SET_SEC_MAX) {
    return 0;
  }
  // also check that timer is not 00:00
  {
    char pos;
    for (pos = 0; pos < SET_BUFFER_SIZE; pos++) {
      if (set_buffer[pos] != '0') return 1;
    }
  }
  // timer is 00:00
  return 0;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function converts the time value currently being inputted to the
//    format used by display_time().
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    the time being set in the described format
//
//-----------------------------------------------------------------------------
unsigned int set_buffer_into_time() {
  unsigned int acc;

  acc = set_buffer[0];
  acc = acc * 10 + set_buffer[1];
  acc = acc * 6 + set_buffer[2];
  acc = acc * 10 + set_buffer[3];

  return acc;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called on keypad press events, and handles the setting
//    of clock/timer values.
//
// INPUT PARAMETERS:
//    k - the key that has been pressed
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void set_accept_key(unsigned char k) {
  // make use of early returns to avoid
  // heavily nested statements

  // no input while timer active
  if (clock_timer_state == CLOCK_TIMER_STATE_TIMING) return;
  if (clock_timer_state == CLOCK_TIMER_STATE_PAUSED) return;

  // handle no setting in progress
  if (set_mode == SET_MODE_NONE) switch (k) {
    case KEYPAD_STAR:
      // begin setting clock
      if (clock_timer_state == CLOCK_TIMER_STATE_ENJOY) {
        tmod_set_state(TMOD_STATE_MONITOR);
      }
      set_mode = SET_MODE_CLOCK;
      set_pos = 0;
      clock_timer_set_state(CLOCK_TIMER_STATE_NO_DRAW);
      display_input_start();
      serial_transmit("Setting Clock...\r\n");
      return;
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
      // begin setting timer
      if (clock_timer_state == CLOCK_TIMER_STATE_ENJOY) {
        tmod_set_state(TMOD_STATE_MONITOR);
      }
      set_mode = SET_MODE_TIMER;
      set_buffer[0] = k;
      set_pos = 1;
      clock_timer_set_state(CLOCK_TIMER_STATE_NO_DRAW);
      display_input_start();
      display_input_pos(k, 0);
      serial_transmit("Setting Timer...\r\n");
      return;
    default:
      // invalid input
      return;
  }

  // filter out all invalid input
  if (k > 9) return;

  // store digit
  set_buffer[set_pos] = k;
  display_input_pos(k, set_pos++);
  if (set_pos != SET_BUFFER_SIZE) {
    // not done inputting yet, early return
    return;
  }

  if (set_mode == SET_MODE_CLOCK) {
    // setting clock
    set_mode = SET_MODE_NONE;
    if (set_is_ok_clock()) {
      clock_set(set_buffer_into_time());
      clock_timer_set_state(CLOCK_TIMER_STATE_CLOCK);
      // success, early return
      return;
    }
  } else {
    // setting timer
    set_mode = SET_MODE_NONE;
    if (set_is_ok_timer()) {
      timer_start(set_buffer_into_time());
      // success, early return
      return;
    }
  }

  // failed to set
  serial_transmit("Invalid Input\r\n");
  clock_timer_set_state(CLOCK_TIMER_STATE_INVALID);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function is called every RTI and polls the keypad, calling
//    set_accept_key() on detected keypress events.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void set_rti_tick() {
  {
    // check keypad
    static char last_keypress = KEYPAD_NONE;
    char keypress;

    keypress = (char) keyscan();
    if (keypress != last_keypress) {
      if (last_keypress != KEYPAD_NONE) {
        set_accept_key(last_keypress);
      }
    }
    last_keypress = keypress;
  }
}

/* HANDLES SERIAL IO */

// SCI0
#define SERIAL__REG(reg) SCI0##reg
#define SERIAL_INT 20

// use macros so that switching to SCI1
// would be more convienient
#define SERIAL_CR2 SERIAL__REG(CR2)
#define SERIAL_BDL SERIAL__REG(BDL)
#define SERIAL_CR1 SERIAL__REG(CR1)
#define SERIAL_SR1 SERIAL__REG(SR1)
#define SERIAL_DRL SERIAL__REG(DRL)

// used to setup serial
#define SERIAL_TRANSMIT_ENABLE 0x08
#define SERIAL_RECIEVE_ENABLE 0x04
#define SERIAL_INTERRUPT_TRANSMIT 0x80
#define SERIAL_INTERRUPT_RECIEVE 0x20
#define SERIAL_BAUD 9600

// mark readiness to transmit/recieve
#define SERIAL_FLAG_TRANSMIT 0x80
#define SERIAL_FLAG_RECIEVE 0x20

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function initializes the serial port
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void serial_init() {
  // set baud rate
  // since baud rate > ~5860, we only use BDL
  SERIAL_BDL = (BUS_FREQUENCY / 16) / SERIAL_BAUD;
  // 8-bit, no parity
  SERIAL_CR1 = 0;
  // enable
  SERIAL_CR2 = SERIAL_TRANSMIT_ENABLE |\
            SERIAL_RECIEVE_ENABLE |\
            SERIAL_INTERRUPT_RECIEVE;
  // clear screen
  asm sei;
  serial_transmit("\x1b[2J\x1b[;H");
  asm cli;
}

#define SERIAL_RING_SIZE 64
static char serial_ring[SERIAL_RING_SIZE];
static char serial_ring_pos;
static char serial_ring_len;

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function queues a message to transmit on the serial connection.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    1 on success, 0 on failure
//
//-----------------------------------------------------------------------------
char serial_transmit(char *str) {
  char new_len, pos;

  new_len = serial_ring_len;
  pos = serial_ring_pos + new_len;
  pos &= SERIAL_RING_SIZE - 1;
  
  while (*str) {
    if (new_len == SERIAL_RING_SIZE) {
      // cannot transsmit
      // fail safe, discard transmission
      return 0;
    }
    
    serial_ring[pos++] = *(str++);
    pos &= SERIAL_RING_SIZE - 1;
    new_len++;
  }
  
  // transmission possible
  // complete
  serial_ring_len = new_len;

  // enable transmission interrupts
  SERIAL_CR2 |= SERIAL_INTERRUPT_TRANSMIT;
  
  return 1;
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This interrupt handler performs direct reception and transmission on
//    the serial connection.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void interrupt SERIAL_INT serial_handle_int() {
  char flags;

  flags = SERIAL_SR1;

  if (flags & SERIAL_FLAG_TRANSMIT) {
    if (serial_ring_len) {
      serial_ring_len--;
      // pop from ring and transmit
      SERIAL_DRL = serial_ring[serial_ring_pos++];
      serial_ring_pos &= SERIAL_RING_SIZE - 1;
      // reread flags
      flags = SERIAL_SR1;
    } else {
      // turn off transmission interrupts
      SERIAL_CR2 &= ~SERIAL_INTERRUPT_TRANSMIT;
    }
  }
  
  if (flags & SERIAL_FLAG_RECIEVE) {
    static char transmit_mini_buf[] = {
      0, '\r', '\n', 0
    };
    char read_char, emu_key;
  
    read_char = SERIAL_DRL;
    
    // map from ascii to keypress code
    // while filtering out unexpected input
    if ((read_char >= '0') && (read_char <= '9')) {
      emu_key = read_char - '0';
    } else if (read_char == '*') {
      emu_key = KEYPAD_STAR;
    } else {
      // early return
      return;
    }
    
    // echo and emulate keypress
    // only emulate if echo will succeed
    *transmit_mini_buf = read_char;
    if (serial_transmit(transmit_mini_buf)) {
      set_accept_key(emu_key);
    }
  }
}

/* RTI */

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This interrupt handler calls all of the dedicated RTI event handlers.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void interrupt 7 RTI0_handler() {
  set_rti_tick();
  clock_timer_rti();
  tmod_rti_tick();
  photo_rti();
  
  clear_RTI_flag();
}

/* MAIN */

//-----------------------------------------------------------------------------
// DESCRIPTION
//    This function initializes the microwave, then handles blocking lcd
//    redraw in a loop.
//
// INPUT PARAMETERS:
//    NONE
//
// OUTPUT PARAMETERS:
//    NONE
//
// RETURN:
//    NONE
//
//-----------------------------------------------------------------------------
void main(void) {
  PLL_init();
  
  seg7_disable();
  led_disable();
  keypad_enable();
  ad0_enable();
  lcd_buf_init();
  serial_init();
  turntable_latch_init();
  abort_lamp_init();
  tmod_init();
  clock_register_update();

  RTI_init(); // also globally enables interrupts

  for (;;) {
    while (lcd_buf_flush_step()) {}
    /*asm wai;*/ // commented out to remove small race condition
  }
}
