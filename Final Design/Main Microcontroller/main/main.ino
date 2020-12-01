#include <Wire.h>
#include <DS3231.h>
#include <Threads.h>
#include <math.h>
#include <stdlib.h>
#include <LM92.h>
#include <DHT.h>

/************************************/
/****** GLOBAL CONSTANTS ************/
/************************************/
// # of buttons we read @ once
const int MAX_PRESSES = 1;

// Rebound time in ms
const unsigned long DEBOUNCE = 10;

// I2C addresses
const int LED_I2C_ADDR = 0x05;
const int LCD_I2C_ADDR = 0x06;
const int LM92_I2C_ADDR = 0x48; // A0, A1 grounded

// Protothread delays
const unsigned long SERIAL_THREAD_DELAY = 250; // ms
const unsigned long TIMER_THREAD_DELAY = 100; // ms
const unsigned long TEMP_THREAD_DELAY = 600; // ms
const unsigned long KEYPAD_THREAD_DELAY = 100; // ms
const unsigned long LCD_THREAD_DELAY = 1000; // ms

// How many temps to read, then averaged
const byte T_AVG_READ = 10;

// Time between temp reads
const byte T_READ_DELAY = 10;

const char STATE_OFF[4] =   {'O', 'F', 'F', ' '};
const char STATE_HIGH[4] =  {'H', 'E', 'A', 'T'};
const char STATE_LOW[4] =   {'C', 'O', 'O', 'L'};
const char UNIT_C =   'C';
const char UNIT_K =   'K';
const char UNIT_F =   'F';
const char UNIT_H =   '%';
const char SENSOR0[3] = {'T', '9', '2'};
const char SENSOR1[3] = {'T', '1', '9'};
const char SENSOR2[3] = {'D', 'H', 'T'};

#define DHTTYPE DHT22
/************************************/
/************************************/
/************************************/


/************************************/
/********* KEYPAD STUFF *************/
/************************************/
const int row_1 = P5_3; // 40
const int row_2 = P5_4; // 39
const int row_3 = P3_4; // 38
const int row_4 = P3_5; // 37
const int col_1 = P3_6; // 36
const int col_2 = P3_7; // 35
const int col_3 = P1_4; // 34
const int col_4 = P1_5; // 33
const int row_interrupt = P5_2; // 41

const byte row_pins[4][2] = {
  {5, 3},
  {5, 4},
  {3, 4},
  {3, 5}
};

const byte col_pins[4][2] {
  {3, 6},
  {3, 7},
  {1, 4},
  {1, 5}
};

const byte int_pin[2] = {5, 2};

// "hash table" lookup for pins
const char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

int num_pressed = 0;
byte pressed[MAX_PRESSES];
/************************************/
/************************************/
/************************************/


/************************************/
/*********** TEST_LEDS **************/
/************************************/
const int test_led_1 = P6_0; // 22
const int test_led_2 = P4_3; // 23
const int test_led_3 = P4_2; // 24

const byte test_leds[3][2] = {
  {6, 0},
  {4, 3},
  {4, 2}
};
/************************************/
/************************************/
/************************************/


/************************************/
/********* SERIAL STUFF *************/
/************************************/
const int tx = P1_7; // 31
const int rx = P1_6; // 32
byte read_char;
/************************************/
/************************************/
/************************************/


/************************************/
/********* DATA CONTAINERS **********/
/************************************/
byte RX_data[32];
byte *RX_str = RX_data;
byte datetime_idx = 0;
byte serial_idx = 0;
DS3232RTC RTC(false);
LM92 lm92;

char time_str_arr[3];
char temp_str_arr[13][4];
/************************************/
/************************************/
/************************************/


/************************************/
/*********** TEMPERATURES ***********/
/************************************/
const int lm19_pin = A1; // 2
const int dht_pin = P5_0; // 43
DHT dht(dht_pin, DHTTYPE);

float humidity = 0.0;


byte temp_buff[2];
float lm92_tempC = 0.0;
float lm92_tempK = 0.0;
float lm92_tempF = 0.0;
float lm19_tempC = 0.0;
float lm19_tempK = 0.0;
float lm19_tempF = 0.0;
float dht_tempC = 0.0;
float dht_tempK = 0.0;
float dht_tempF = 0.0;
float dht_indexC = 0.0;
float dht_indexK = 0.0;
float dht_indexF = 0.0;

float temp_arr[13];

// Heater state: 0 = OFF, 1 = HEAT, 2 = COOL
byte heater_state = 0;

// Temp sensor: 0 = LM92, 1 = LM19, 2 = DHT
byte temp_sensor = 0;

// Temperature units: 0 = K, 1 = C, 2 = F, 3 = Hum
byte temp_units = 0;

// Time passed in current heater mode
unsigned long time_passed = 0;

// DHT Mode: 0 = temp, 1 = temp index
byte dht_mode = 0;

// Constants for parabolic temp calc from lm19
float t_A = -1481.96;
float t_B = 2.1962E6;
float t_C = 1.8639;
float t_D = 3.88E-6;

/************************************/
/************************************/
/************************************/


/************************************/
/********** LED BAR RESET ***********/
/************************************/
const int led_bar_rst = P2_4; // 11
const byte led_rst[2] = {2, 4};
/************************************/
/************************************/
/************************************/


/************************************/
/****** Function Declarations *******/
/************************************/
static void set_datetime();
static inline void pin_write(byte, byte);
static inline void pin_clear(byte, byte);
static inline byte pin_read(byte, byte);
static byte d_read(byte, byte);
static int millidelay(unsigned long);
static void microdelay(unsigned long);
static void scan();
static void add_button_press(int, int);
static void reset_keypad();
static void send_lcd_data();
static float read_lm19();
static void get_lm19_temp();
static void day_of_week(byte, char*);
void serial_run();
void time_run();
void temp_run();
void keypad_run();
void lcd_run();

static void write_lcd(char *vals);

/************************************/
/************************************/
/************************************/


/************************************/
/****** PROTOTHREADING STUFF ********/
/************************************/
// Thread serial(serial_run, SERIAL_THREAD_DELAY);
Thread temp(temp_run, TEMP_THREAD_DELAY);
Thread keypad(keypad_run, KEYPAD_THREAD_DELAY);
Thread lcd(lcd_run, LCD_THREAD_DELAY);

Thread thread_array[3] = {
  // serial,
  temp,
  keypad,
  lcd
};
/************************************/
/************************************/
/************************************/


/************************************/
/********** DHT SENSOR INIT *********/
/************************************/

/************************************/
/************************************/
/************************************/

void setup()
{
  // WDTCTL = WDTPW + WDTHOLD;
  // PM5CTL0 &= ~LOCKLPM5;
  Serial1.begin(9600);
  Wire.begin();
  Wire.setClock(10000);
  setSyncProvider(RTC.get);
  lm92.enableFaultQueue(true);
  lm92.ResultInCelsius = true;
  lm92.setTLow(0.0);
  lm92.setTHigh(100.0);

  dht.begin();


  // Set pins as input / output
  pinMode(row_1, INPUT_PULLDOWN);
  pinMode(row_2, INPUT_PULLDOWN);
  pinMode(row_3, INPUT_PULLDOWN);
  pinMode(row_4, INPUT_PULLDOWN);
  pinMode(col_1, OUTPUT);
  pinMode(col_2, OUTPUT);
  pinMode(col_3, OUTPUT);
  pinMode(col_4, OUTPUT);
  pinMode(row_interrupt, INPUT_PULLUP);
  pinMode(test_led_1, OUTPUT);
  pinMode(test_led_2, OUTPUT);
  pinMode(test_led_3, OUTPUT);
  pinMode(led_bar_rst, OUTPUT);
  pinMode(dht_pin, INPUT);

  // Set all keypad outputs high
  pin_write(col_pins[0][0], col_pins[0][1]);
  pin_write(col_pins[1][0], col_pins[1][1]);
  pin_write(col_pins[2][0], col_pins[2][1]);
  pin_write(col_pins[3][0], col_pins[3][1]);
  pin_write(led_rst[0], led_rst[1]);

  // Clear LED bar w/ a reset
  pin_clear(led_rst[0], led_rst[1]);
  millidelay(100);
  pin_write(led_rst[0], led_rst[1]);
  delay(100);

  pin_clear(test_leds[0][0], test_leds[0][1]);
  pin_clear(test_leds[1][0], test_leds[1][1]);
  pin_clear(test_leds[2][0], test_leds[2][1]);

  // Check if DS3231 is synced
  if (timeStatus() != timeSet) {
    // if it isn't, choose a default
    init_default_time();
  }
  time_passed = millis();
}


void loop()
{
  // Check protothreads
  thread_check();
}


void serial_run()
{
  if (Serial1.available() > 0) {
    while(Serial1.available() > 0) {
      read_char = Serial1.read();
      if (read_char != 10 && read_char != 13) {
        if (read_char == '@') {
          datetime_idx = 0;
          set_datetime();
        }
      }
    }
  }
}

static void init_default_time()
{
  // April 13th, 2020
  // 6:30:00 am
  time_t t;
  tmElements_t tm;
  tm.Year = CalendarYrToTm(2020);
  tm.Month = 4;
  tm.Day = 13;
  tm.Hour = 6;
  tm.Minute = 30;
  tm.Second = 0;
  t = makeTime(tm);
  RTC.set(t);
  setTime(t);
}

static void set_datetime()
{
  time_t t;
  tmElements_t tm;
  if (Serial1.available() > 0) {
    int y = Serial1.parseInt();
    tm.Year = CalendarYrToTm(y);
    tm.Month = Serial1.parseInt();
    tm.Day = Serial1.parseInt();
    tm.Hour = Serial1.parseInt();
    tm.Minute = Serial1.parseInt();
    tm.Second = Serial1.parseInt();
    t = makeTime(tm);
    RTC.set(t);
    setTime(t);
  }
  while (Serial1.available() > 0) Serial1.read();
}


void temp_run()
{
  float t = 0.0;
  for (int i = 0; i < T_AVG_READ; i++) {
    t += lm92.readTemperature();
    delay_ms(T_READ_DELAY);
  }
  lm92_tempC = t / T_AVG_READ;
  lm92_tempK = lm92_tempC + 273.15;
  lm92_tempF = (lm92_tempC * (9.0/5.0)) + 32.0;

  // Generate lm19 values
  get_dht_data();
  get_lm19_temp();

  temp_arr[0] = lm92_tempK;
  temp_arr[1] = lm92_tempC;
  temp_arr[2] = lm92_tempF;
  temp_arr[3] = lm19_tempK;
  temp_arr[4] = lm19_tempC;
  temp_arr[5] = lm19_tempF;
  temp_arr[6] = dht_tempK;
  temp_arr[7] = dht_tempC;
  temp_arr[8] = dht_tempF;
  temp_arr[9] = dht_indexK;
  temp_arr[10] = dht_indexC;
  temp_arr[11] = dht_indexF;
  temp_arr[12] = humidity;
}


void keypad_run()
{
  if(pin_read(int_pin[0], int_pin[1])) {
        scan();
  }
}


static void data_to_str()
{
  // Create arrays holding float values
  for (int i = 0; i < 13; i++) {
    float t = temp_arr[i];
  	int t_ints = (int)t;
  	float t_decimal = t - t_ints;

    int t_hundreds = (int)(t_ints % 1000 / 100);
  	int t_tens = (int)(t_ints % 100 / 10);
  	int t_ones = (int)(t_ints % 10);
  	int t_tenths = (int)((int)(t_decimal * 10) % 10);

    if (t > 100.0) {
      temp_str_arr[i][0] = (char)(t_hundreds + '0');
    	temp_str_arr[i][1] = (char)(t_tens + '0');
    	temp_str_arr[i][2] = (char)(t_ones + '0');
    	temp_str_arr[i][3] = ' ';
    } else {
    	temp_str_arr[i][0] = (char)(t_tens + '0');
    	temp_str_arr[i][1] = (char)(t_ones + '0');
    	temp_str_arr[i][2] = '.';
    	temp_str_arr[i][3] = (char)(t_tenths + '0');
    }
  }

  // Array for time value
  unsigned long t_sec = (char)((millis() - time_passed) / 1000);

  char t = ((t_sec / 100) % 10);
  if (t == 0) {
    time_str_arr[0] = ' ';
  } else if ( t < 0 || t > 9) {
    time_str_arr[0] = '?';
  } else {
    time_str_arr[0] = t + '0';
  }

  t = ((t_sec / 10) % 10);
  if (t == 0 && time_str_arr[0] == ' ') {
    time_str_arr[1] = ' ';
  } else {
    time_str_arr[1] = t + '0';
  }

  t = (t_sec % 10);
  time_str_arr[2] = t + '0';

}


void lcd_run()
{
  /****** COUNT BYTES WE SEND: *****/
  // Turn all data into c_str
  data_to_str();

  /******* TEMP: send 4  (4 tot) ******/
  if (temp_sensor == 0) {
    if (temp_units == 0) {
       write_lcd(temp_str_arr[0][0]);
       write_lcd(temp_str_arr[0][1]);
       write_lcd(temp_str_arr[0][2]);
       write_lcd(temp_str_arr[0][3]);
    } else if (temp_units == 1) {
       write_lcd(temp_str_arr[1][0]);
       write_lcd(temp_str_arr[1][1]);
       write_lcd(temp_str_arr[1][2]);
       write_lcd(temp_str_arr[1][3]);
    } else {
       write_lcd(temp_str_arr[2][0]);
       write_lcd(temp_str_arr[2][1]);
       write_lcd(temp_str_arr[2][2]);
       write_lcd(temp_str_arr[2][3]);
    }
  } else if (temp_sensor == 1) {
    if (temp_units == 0) {
       write_lcd(temp_str_arr[3][0]);
       write_lcd(temp_str_arr[3][1]);
       write_lcd(temp_str_arr[3][2]);
       write_lcd(temp_str_arr[3][3]);
    } else if (temp_units == 1) {
       write_lcd(temp_str_arr[4][0]);
       write_lcd(temp_str_arr[4][1]);
       write_lcd(temp_str_arr[4][2]);
       write_lcd(temp_str_arr[4][3]);
    } else {
       write_lcd(temp_str_arr[5][0]);
       write_lcd(temp_str_arr[5][1]);
       write_lcd(temp_str_arr[5][2]);
       write_lcd(temp_str_arr[5][3]);
    }
  } else {
    if (temp_units == 0) {
       write_lcd(temp_str_arr[6][0]);
       write_lcd(temp_str_arr[6][1]);
       write_lcd(temp_str_arr[6][2]);
       write_lcd(temp_str_arr[6][3]);
    } else if (temp_units == 1) {
       write_lcd(temp_str_arr[7][0]);
       write_lcd(temp_str_arr[7][1]);
       write_lcd(temp_str_arr[7][2]);
       write_lcd(temp_str_arr[7][3]);
    } else if (temp_units == 2){
       write_lcd(temp_str_arr[8][0]);
       write_lcd(temp_str_arr[8][1]);
       write_lcd(temp_str_arr[8][2]);
       write_lcd(temp_str_arr[8][3]);
    } else {
       write_lcd(temp_str_arr[12][0]);
       write_lcd(temp_str_arr[12][1]);
       write_lcd(temp_str_arr[12][2]);
       write_lcd(temp_str_arr[12][3]);
    }
  }

  /****** TIME: send 3 (7 tot) *****/
  write_lcd(time_str_arr[0]);
  write_lcd(time_str_arr[1]);
  write_lcd(time_str_arr[2]);


  /****** STATE: send 4 (11 tot) *****/
  if (heater_state == 0) {
    write_lcd(STATE_OFF[0]);
    write_lcd(STATE_OFF[1]);
    write_lcd(STATE_OFF[2]);
    write_lcd(STATE_OFF[3]);
  } else if (heater_state == 1) {
    write_lcd(STATE_HIGH[0]);
    write_lcd(STATE_HIGH[1]);
    write_lcd(STATE_HIGH[2]);
    write_lcd(STATE_HIGH[3]);
  } else if (heater_state == 2) {
    write_lcd(STATE_LOW[0]);
    write_lcd(STATE_LOW[1]);
    write_lcd(STATE_LOW[2]);
    write_lcd(STATE_LOW[3]);
  }

  /****** UNITS: send 1 (12 tot) ******/
  if (temp_units == 0) {
    write_lcd(UNIT_K);
  } else if (temp_units == 1) {
    write_lcd(UNIT_C);
  } else if (temp_units == 2) {
    write_lcd(UNIT_F);
  } else if (temp_units == 3) {
    write_lcd(UNIT_H);
  }

  /********* SENSOR: send 3 (15 tot) *****/
  if (temp_sensor == 0) {
    write_lcd(SENSOR0[0]);
    write_lcd(SENSOR0[1]);
    write_lcd(SENSOR0[2]);
  } else if (temp_sensor == 1) {
    write_lcd(SENSOR1[0]);
    write_lcd(SENSOR1[1]);
    write_lcd(SENSOR1[2]);
  } else {
    write_lcd(SENSOR2[0]);
    write_lcd(SENSOR2[1]);
    write_lcd(SENSOR2[2]);
  }

  write_lcd('!');
}


static inline void pin_write(byte port, byte pin)
{
  if(port == 1) {
    P1OUT |= (1 << pin);
  } else if(port == 2) {
    P2OUT |= (1 << pin);
  } else if(port == 3) {
    P3OUT |= (1 << pin);
  } else if(port == 4) {
    P4OUT |= (1 << pin);
  } else if(port == 5) {
    P5OUT |= (1 << pin);
  } else if(port == 6) {
    P6OUT |= (1 << pin);
  }
}


static inline void pin_clear(byte port, byte pin)
{
  if(port == 1) {
    P1OUT &= ~(1 << pin);
  } else if(port == 2) {
    P2OUT &= ~(1 << pin);
  } else if(port == 3) {
    P3OUT &= ~(1 << pin);
  } else if(port == 4) {
    P4OUT &= ~(1 << pin);
  } else if(port == 5) {
    P5OUT &= ~(1 << pin);
  } else if(port == 6) {
    P6OUT &= ~(1 << pin);
  }
}


static inline byte pin_read(byte port, byte pin)
{
  if(port == 1) {
    return P1IN & (1 << pin);
  } else if(port == 2) {
    return P2IN & (1 << pin);
  } else if(port == 3) {
    return P3IN & (1 << pin);
  } else if(port == 4) {
    return P4IN & (1 << pin);
  } else if(port == 5) {
    return P5IN & (1 << pin);
  } else if(port == 6) {
    return P6IN & (1 << pin);
  }
}


static byte d_read(byte port, byte pin)
{
  millidelay(DEBOUNCE);
  return pin_read(port, pin);
}

static inline void delay_ms(unsigned long m)
{
  unsigned long current = millis();
  while((millis() - current) <= m) {}
}

static int millidelay(unsigned long m)
{
  unsigned long current = millis();
  while(millis() - current <= m) {
    if(pin_read(int_pin[0], int_pin[1])) {
      // An LED interrupt was detected, break loop
      return 1;
    }
  }
  return 0;
}


static void microdelay(unsigned long m)
{
  unsigned long current = micros();
  while(micros() - current <= m){}
}


static void scan()
{
begin:
  for(int i = 0; i < 4; i++) {
    pin_clear(col_pins[i][0], col_pins[i][1]);
  }

  // Turn on each column, once at a time, and
  // read each row
  for(int i = 0; i < 4; i++) {
    pin_write(col_pins[i][0], col_pins[i][1]);
    for(int j = 0; j < 4; j++) {
      if(d_read(row_pins[j][0], row_pins[j][1])) {
        // We have a button press!
        add_button_press(j, i);
      }
    }
    pin_clear(col_pins[i][0], col_pins[i][1]);
  }

  if (pressed[0] == '0') {
    if (heater_state != 0) {
      time_passed = millis();
    }
    heater_state = 0;
    clear_leds();
  } else if (pressed[0] == '1') {
    if (heater_state != 1) {
      time_passed = millis();
    }
    heater_state = 1;
    clear_leds();
    red_led(1);
  } else if (pressed[0] == '2') {
    if (heater_state != 2) {
      time_passed = millis();
    }
    heater_state = 2;
    clear_leds();
    aqua_led(1);
  } else if (pressed[0] == '4') {
    if (temp_units == 3) {
      temp_units = 0;
    }
    temp_sensor = 0;
  } else if (pressed[0] == '5') {
    if (temp_units == 3) {
      temp_units = 0;
    }
    temp_sensor = 1;
  } else if (pressed[0] == '6'){
    temp_sensor = 2;
  } else if (pressed[0] == 'A') {
    temp_units = 0;
  } else if (pressed[0] == 'B') {
    temp_units = 1;
  } else if (pressed[0] == '#') {
    temp_units = 2;
  } else if (pressed[0] == 'D') {
    if (temp_sensor == 2) {
      temp_units = 3;
    }
  }

  byte press = pressed[0];
  if (press != 0 && (press == '0' || press == '1' || press == '2')) {
    // Send data to LED

    pin_write(led_rst[0], led_rst[1]);
    delay_ms(100);

    pin_clear(led_rst[0], led_rst[1]);
    delay_ms(100);

    Wire.beginTransmission(LED_I2C_ADDR);
    Wire.write(press);
    Wire.endTransmission();
  }

  reset_keypad();
}


static void add_button_press(int row, int col)
{
  if(num_pressed == MAX_PRESSES) {
    // We're done reading buttons
    return;
  }
  // Add button pressed by "hash"
  pressed[num_pressed] = keys[row][col];
  num_pressed++;
}


static void reset_keypad()
{
  num_pressed = 0;
  for(int i = 0; i < 4; i++) {
    pin_write(col_pins[i][0], col_pins[i][1]);
  }
  memset(pressed, 0, sizeof(pressed));
}

static void get_lm19_temp()
{
  float vin;
  float t = 0.0;
  for (int i = 0; i < T_AVG_READ; i++) {
    vin = 3.3 * analogRead(lm19_pin) / 1024.0;
    t += t_A + sqrt(t_B + ((t_C - vin) / t_D)) + 3.0;
    delay_ms(T_READ_DELAY);
  }

  t /= T_AVG_READ;

  lm19_tempC = t;
  lm19_tempK = lm19_tempC + 273.15;
  lm19_tempF = (lm19_tempC * (9.0 / 5.0)) + 32.0;
}


static void get_dht_data()
{
  float h = dht.readHumidity();
  if ((int)h != -1) {
    humidity = h;
  }

  float t = dht.readTemperature();
  if ((int)t != -1) {
    dht_tempC = t;
  }

  dht_tempK = dht_tempC + 273.15;
  dht_tempF = (dht_tempC * (9.0 / 5.0)) + 32.0;

  dht_indexC = dht.computeHeatIndex(dht_tempC, humidity, false);
  dht_indexK = dht_indexC + 273.15;
  dht_indexF = (dht_indexC * (9.0 / 5.0)) + 32.0;
}

static void toggle_led(int num)
{
  if (pin_read(test_leds[num - 1][0], test_leds[num - 1][1]) == 1) {
    pin_clear(test_leds[num - 1][0], test_leds[num - 1][1]);
  } else {
    pin_write(test_leds[num - 1][0], test_leds[num - 1][1]);
  }
}

static void red_led(int logic)
{
  if (logic == 1) {
    pin_write(test_leds[0][0], test_leds[0][1]);
  } else if (logic == 0) {
    pin_clear(test_leds[0][0], test_leds[0][1]);
  }
}

static void green_led(int logic)
{
  if (logic == 1) {
    pin_write(test_leds[1][0], test_leds[1][1]);
  } else if (logic == 0) {
    pin_clear(test_leds[1][0], test_leds[1][1]);
  }
}

static void aqua_led(int logic)
{
  if (logic == 1) {
    pin_write(test_leds[2][0], test_leds[2][1]);
  } else if (logic == 0) {
    pin_clear(test_leds[2][0], test_leds[2][1]);
  }
}


static void clear_leds()
{
  pin_clear(test_leds[0][0], test_leds[0][1]);
  pin_clear(test_leds[2][0], test_leds[2][1]);
  pin_clear(test_leds[1][0], test_leds[1][1]);
}

static void thread_check()
{
  for (int i = 0; i < 4; i++) {
    thread_array[i].check();
  }
}

static void write_lcd(char *vals)
{
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(vals);
  Wire.endTransmission();
}

static void write_lcd(const char *vals)
{
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(vals);
  Wire.endTransmission();
}


static void write_lcd(char vals)
{
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(vals);
  Wire.endTransmission();
}
