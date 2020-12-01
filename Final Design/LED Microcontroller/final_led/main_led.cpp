#include "msp430fr2310.h"
#include <stdint.h>
#include "Board.h"
#include "./driverlib/MSP430FR2xx_4xx/driverlib.h"

#define SLAVE_ADDRESS 0x05

// LED patterns
#define MODEA         0
#define MODEB         1
#define MODEC         2
#define MODED         3

// Right and left circular bit shift
#define rright( x, n, b )  ( (x) >> (n) ) | ( (x) << ( (b) - (n) ))
#define rleft( x, n, b )  ( (x) << (n) ) | ( (x) >> ( (b) - (n) ))

// Replace occurences of "byte" -> "uint8_t"
typedef uint8_t byte;

// Arrays for pins: {port, pin}
const byte LED_CLK[2]   = {1, 1};
const byte LED_PINS[8][2] = {
  {1, 5}, // #1
  {1, 6}, // #2
  {1, 7}, // #3
  {2, 0}, // #4
  {1, 4}, // #5
  {1, 0}, // #6
  {2, 6}, // #7
  {2, 7}, // #8
};


// Declare functions
static inline void led_write(byte port, byte pin);
static inline void led_clear(byte port, byte pin);
static void write_led(const byte pin[2], byte logic);
static void write_leds(byte mask);
static void handle_mode(byte mode);
static void mode_A();
static void mode_B();
static void mode_C();
static void mode_D();
static void reset_FF();
static void I2Creceive();
static void loop();


volatile byte RX_data;
volatile bool i2c_received = false;

int main(void)
{
    /***********************************************
    **************** SETUP *************************
    ************************************************/
    WDTCTL = WDTPW + WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;
    // P1SEL0 |= (BIT2 + BIT3); // configure I2C pins
    // P1SEL1 |= (BIT2 + BIT3);

    // Set up I2C
    // UCB0CTL1 |= UCSWRST; // eUSCI_Bin reset state
    // UCB0CTL0 |= UCMODE_3; // I2C slave mode
    // UCB0CTL1 = UCSSEL_2 + UCSWRST;
    // UCB0I2CSA = SLAVE_ADDRESS; // Own Address
    //
    // UCB0CTL1 &= ~UCSWRST; // eUSCI_B_in operational state
    // UCB0IE |= UCTXIE + UCRXIE;// enable TX&RX-interrupt


    // // Configure Pins for I2C
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN2 + GPIO_PIN3,
        GPIO_PRIMARY_MODULE_FUNCTION
    );
    // GPIO_setAsPeripheralModuleFunctionInputPin(
    //     GPIO_PORT_UCB0SDA,
    //     GPIO_PIN_UCB0SDA,
    //     GPIO_FUNCTION_UCB0SDA
    // );
    //
    //PMM_unlockLPM5();
    //
    // eUSCI configuration
    EUSCI_B_I2C_initSlaveParam param = {0};
    param.slaveAddress = SLAVE_ADDRESS;
    param.slaveAddressOffset = EUSCI_B_I2C_OWN_ADDRESS_OFFSET0;
    param.slaveOwnAddressEnable = EUSCI_B_I2C_OWN_ADDRESS_ENABLE;
    EUSCI_B_I2C_initSlave(EUSCI_B0_BASE, &param);
    //
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
    //
    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0
            );
    //
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0
            );
    UCB0CTL1 &= ~UCSWRST; // eUSCI_B_in operational state
    __bis_SR_register(GIE);        // Enter active mode w/ interrupts

    //Set output directions
    P1DIR |= (BIT0 + BIT1 + BIT4 + BIT5 + BIT6 + BIT7);
    P2DIR |= (BIT0 + BIT6 + BIT7);
    //
    // P1REN |= (BIT0 + BIT1 + BIT4 + BIT5 + BIT6 + BIT7);
    // P2REN |= (BIT0 + BIT6 + BIT7);
    // Set all pins low (don't touch I2C)
    P1OUT &= (~BIT0 + ~BIT1 + ~BIT4 + ~BIT5 + ~BIT6 + ~BIT7);
    P2OUT &= (~BIT0 + ~BIT6 + ~BIT7);

    loop();
}

static void loop()
{
  while(1) {
        if(i2c_received) {
            handle_mode(RX_data);
        }
    }
}

static inline void led_write(byte port, byte pin)
{
  if(port == 1) {
    P1OUT |= (1 << pin);
  } else if(port == 2) {
    P2OUT |= (1 << pin);
  }
}

static inline void led_clear(byte port, byte pin)
{
  if(port == 1) {
    P1OUT &= ~(1 << pin);
  } else if(port == 2) {
    P2OUT &= ~(1 << pin);
  }
}

/*
 *  Write to F-F as block, instead of individual pins.
 *  "Mask" is a byte whose bits tell us whether the ith pin
 *  is on or off
 */
static void write_leds(byte mask)
{
  for(int i = 0; i < 8; i++) {
    if(((mask >> (7 - i)) & 0x01)) {
      led_write(LED_PINS[i][0], LED_PINS[i][1]);
    } else {
      led_clear(LED_PINS[i][0], LED_PINS[i][1]);
    }
  }
  // Toggle clock on F-F
  P1OUT &= ~BIT1;
  __delay_cycles(10);
  P1OUT |= BIT1;
}

// Handle received mode from master
static void handle_mode(byte mode)
{
    reset_FF();
    i2c_received = false;
    RX_data = 0;
    switch(mode) {
        case '0':
            write_leds(0x00 << 4);
            break;
        case '1':
            write_leds(0x01 << 4);
            break;
        case '2':
            write_leds(0x02 << 4);
            break;
        case '3':
            write_leds(0x03 << 4);
            break;
        case '4':
            write_leds(0x04 << 4);
            break;
        case '5':
            write_leds(0x05 << 4);
            break;
        case '6':
            write_leds(0x06 << 4);
            break;
        case '7':
            write_leds(0x07 << 4);
            break;
        case '8':
            write_leds(0x08 << 4);
            break;
        case '9':
            write_leds(0x09 << 4);
            break;
        case 'A':
            write_leds(0x0A << 4);
            break;
        case 'B':
            write_leds(0x0B << 4);
            break;
        case 'C':
            write_leds(0x0C << 4);
            break;
        case 'D':
            write_leds(0x0D << 4);
            break;
        case 'E':
            write_leds(0x0E << 4);
            break;
        case 'F':
            write_leds(0x0F << 4);
            break;
        default:
            break;
    }
}

/*
 *  Static mode:
 *
 *  XOXOXOXO
 *
 *  (X: on, O: off)
 */
static void mode_A()
{
  write_leds(0b10101010);
}


/*
 *  Rotation mode:
 *
 *  (TODO: COPY HERE LATER)
 */
static void mode_B()
{
  byte mask = 0b01111111;
  while(1) {
    write_leds(mask);
    __delay_cycles(1000000);
    mask = rright(mask, 1, 8);
  }
}


/*
 *  "Middle Oscillator" mode:
 *
 *  (TODO: COPY HERE LATER)
 */
static void mode_C()
{
  // Init byte is 0b00011000, but we can
  // split this into 2 chunks that are both
  // independently bit shifted
    while(1) {
    byte leftbits = 0b0001;
    byte rightbits = 0b1000;

    // Left shifts left,
    for(int i = 0; i < 4; i++) {
      byte mask = ((leftbits << 4) | rightbits);
      write_leds(mask);
      leftbits = leftbits << 1;
      rightbits = rightbits >> 1;
      if(i < 3) {
          __delay_cycles(1000000);
      }
    }

    leftbits = leftbits >> 1;
    rightbits = 1;
    byte mask = ((leftbits << 4) | rightbits);
    write_leds(mask);
    __delay_cycles(1000000);

    for(int i = 0; i < 2; i++) {
      leftbits = leftbits >> 1;
      rightbits = rightbits << 1;
      byte mask = ((leftbits << 4) | rightbits);
      write_leds(mask);
      __delay_cycles(1000000);
    }
  }
}



/*
 *  "Wave" mode
 *
 *  (TODO: COPY HERE LATER)
 */
static void mode_D()
{
  byte mask = 0b00111100;
  while(1) {
    for(int i = 0; i < 6; i++) {
      write_leds(mask);
      mask = (mask >> 1);
      if(i < 5) {
          __delay_cycles(1000000);
      }
    }

    for(int i = 0; i < 4; i++) {
      mask = ((mask << 1) |0b01);
      write_leds(mask);
      __delay_cycles(1000000);
    }

    for(int i = 0; i < 2; i++) {
      mask = (mask << 1);
      write_leds(mask);
      if(i < 1) {
          __delay_cycles(1000000);
      }
    }
  }
}

static void reset_FF()
{
  byte mask = 0b00000000;
  write_leds(mask);
}

// // // #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt
// // //#elif defined(__GNUC__)
// // //__attribute__((interrupt(USCI_B0_VECTOR)))
// // //#endif
void USCIB0_ISR(void)
{
    //i2c_received = true;
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:             // No interrupts break;
            break;
        case USCI_I2C_UCALIFG:      // Arbitration lost
            break;
        case USCI_I2C_UCNACKIFG:    // NAK received (master only)
            break;
        case USCI_I2C_UCSTTIFG:     // START condition detected with own address (slave mode only)
            break;
        case USCI_I2C_UCSTPIFG:     // STOP condition detected (master & slave mode)
            break;
        case USCI_I2C_UCRXIFG3:     // RXIFG3
            break;
        case USCI_I2C_UCTXIFG3:     // TXIFG3
            break;
        case USCI_I2C_UCRXIFG2:     // RXIFG2
            break;
        case USCI_I2C_UCTXIFG2:     // TXIFG2
            break;
        case USCI_I2C_UCRXIFG1:     // RXIFG1
            break;
        case USCI_I2C_UCTXIFG1:     // TXIFG1
            break;
        case USCI_I2C_UCRXIFG0:     // RXIFG0
          i2c_received = true;
          RX_data = EUSCI_B_I2C_slaveGetData(EUSCI_B0_BASE);
            break;
        case USCI_I2C_UCTXIFG0:     // TXIFG0
            break;
        case USCI_I2C_UCBCNTIFG:    // Byte count limit reached (UCBxTBCNT)
            break;
        case USCI_I2C_UCCLTOIFG:    // Clock low timeout - clock held low too long
            break;
        case USCI_I2C_UCBIT9IFG:    // Generated on 9th bit of a transmit (for debugging)
            break;
        default:
            break;
    }
}
