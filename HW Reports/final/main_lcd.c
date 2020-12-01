#include <msp430.h>
#include <inttypes.h>
#include "Board.h"
#include "./driverlib/MSP430FR2xx_4xx/driverlib.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00
/**************************************************/


#define SLAVE_ADDRESS 	0x06
#define EXPECTED_BYTES	15

#define byte char


// Right and left circular bit shift
#define rright( x, n, b )  ( (x) >> (n) ) | ( (x) << ( (b) - (n) ))
#define rleft( x, n, b )  ( (x) << (n) ) | ( (x) >> ( (b) - (n) ))

// LCD variables
byte displayfunction = 0x00;
byte displaycontrol = 0x00;
byte displaymode = 0x00;
const byte numlines = 2;
const byte row_offsets[4] = {0x00, 0x40, 0x10, 0x50};

// I2C variables
byte temp_RX_data = 0;
bool i2c_received = false;
char num_chars = 0;
int data_index = 0;
int datapoints_read = 0;

bool temp_data = false;
bool time_data = false;
bool state_data = false;
bool unit_data = false;


// Declare functions
static void handle_data(byte data);
static void loop();
static void update_screen();
static void lcd_clear();
static void lcd_set_cursor(byte col, byte row);
static void lcd_write(byte);
static void lcd_command(byte);
static void lcd_print(byte *string);
static void lcd_send(byte value, byte mode);
static inline void lcd_write8bits(byte value);


 char data_arr[15];
 char *dataptr = data_arr;

 int data_idx = 0;

 bool new_temp = false;
 bool new_time = false;
 bool new_state = false;
 bool new_units = false;

// Did we finish updating anything?
 bool finished = false;

int main(void)
{
    /***********************************************
    **************** SETUP *************************
    ************************************************/
    WDTCTL = WDTPW + WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    // // Configure Pins for I2C
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN2 + GPIO_PIN3,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

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

	/*
	 *  Pins:
	 *  FF_CLK = 	1.5
	 *  RS = 		1.1
	 *  RW =		1.0
	 *  EN = 		1.4
	 *  D0 =		2.6 (FF)
	 *  D1 = 		2.0 (FF)
	 *  D2 = 		1.7 (FF)
	 *  D3 = 		1.6 (FF)
	 *  D4 = 		2.6
	 *  D5 =		2.0
	 *  D6 =		1.7
	 *  D7 = 		1.6
	 */


    //Set output directions
	P1DIR |= (BIT0 + BIT1 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7);
	P2DIR |= (BIT0 + BIT6);

	// P1REN |= (BIT0 + BIT4 + BIT5 + BIT6 + BIT7 + BIT1);
	// P2REN |= (BIT0 + BIT6);
	P1OUT &= ~(BIT5 + BIT6 + BIT0 + BIT7 + BIT4);
	P2OUT &= ~(BIT0 + BIT6);


	/************** INIT LCD **********************/
	_delay_cycles(100000);
	lcd_command(0b00111100); // 2-line, disp on
	__delay_cycles(7000);
	lcd_command(0b00001100); // disp on, Blink on
	__delay_cycles(100);
	lcd_command(0b00001100); // disp on, Blink on
	__delay_cycles(100);
	lcd_command(0b00001100); // disp on, Blink on
	__delay_cycles(7000);
	lcd_command(0b00000001); // Clear
	__delay_cycles(7000);
	lcd_command(0b00000110); // Increment, no shift
	__delay_cycles(7000);
	lcd_command(0b10000000); // cursor at 0,0
	__delay_cycles(7000);

	/**********************************************/

	// memset(data_arr, 0, sizeof(data_arr));

	lcd_clear();
	__delay_cycles(100);
	lcd_set_cursor(0, 0);
	lcd_print(" TEC CONTROLLER ");
	lcd_set_cursor(0, 1);
	lcd_print(" BY RYAN FRENCH ");
	__delay_cycles(3000000);
	lcd_clear();

	lcd_set_cursor(0, 0);
	lcd_print("TEC STATE: ");

	lcd_set_cursor(3, 1);
	lcd_print(":");

	lcd_set_cursor(9, 1);
	lcd_print("@t=");

	lcd_set_cursor(15, 1);
	lcd_print("s");

    loop();
}

 void loop()
{
    while(1) {
		if (i2c_received) {
			handle_data(temp_RX_data);
		}
    }
}


// Handle received data from master
void handle_data(byte d)
{
    i2c_received = false;

	if (d == '!') {
		// data received
		data_idx = 0;
		update_screen();
	} else {
		data_arr[data_idx] = d;
		data_idx++;
	}
}


static void update_screen()
{
	lcd_set_cursor(11, 0);
	lcd_write(data_arr[7]);
	lcd_write(data_arr[8]);
	lcd_write(data_arr[9]);
	lcd_write(data_arr[10]);

	lcd_set_cursor(0, 1);
	lcd_write(data_arr[12]);
	lcd_write(data_arr[13]);
	lcd_write(data_arr[14]);

	lcd_set_cursor(4, 1);
	lcd_write(data_arr[0]);
	lcd_write(data_arr[1]);
	lcd_write(data_arr[2]);
	lcd_write(data_arr[3]);
	lcd_write(data_arr[11]);

	lcd_set_cursor(12, 1);
	lcd_write(data_arr[4]);
	lcd_write(data_arr[5]);
	lcd_write(data_arr[6]);
}


/************* LCD FUNCTIONS *******************/
 static void lcd_clear()
{
    lcd_command(LCD_CLEARDISPLAY);    // clearparams->display, set cursor position to zero
    __delay_cycles(2000);    // this command takes a long time!
}

 static void lcd_set_cursor(byte col, byte row)
{
    const byte max_lines = sizeof(row_offsets) / sizeof(*(row_offsets));
    if ( row >= max_lines ) {
        row = max_lines - 1;        // we count rows starting w/0
    }
    if ( row >= numlines ) {
        row = numlines - 1;        // we count rows starting w/0
    }

    lcd_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
	__delay_cycles(500);
}

 static void lcd_print(byte *string)
{
	byte *str_ptr = string;
	while (*str_ptr != '\0') {
		lcd_write(*str_ptr);
		str_ptr++;
		__delay_cycles(50);
	}
	__delay_cycles(100);
}

 static void lcd_command(byte value) {
    lcd_send(value, 0);
}

 static void lcd_write(byte value) {
	 // num_chars++;
	 // if (num_chars == 17) {
		//  lcd_set_cursor(0, 1);
	 // } else if (num_chars == 33) {
		//  lcd_clear();
		//  lcd_set_cursor(0, 0);
		//  num_chars = 0;
	 // }
	 lcd_send(value, 1);
}

 static void lcd_send(byte value, byte mode) {
	if (mode == 0) {
		P1OUT &= ~BIT1;
	} else if (mode == 1) {
		P1OUT |= BIT1;
	}
	P1OUT &= ~BIT0;
    lcd_write8bits(value);
}


static inline void lcd_write8bits(byte value) {
	byte val = value;

	P1OUT |= BIT4;
	__delay_cycles(10);
	if ((val >> 0) & 0x01) {
		P2OUT |= BIT6;
	} else {
		P2OUT &= ~BIT6;
	}
	if ((val >> 1) & 0x01) {
		P2OUT |= BIT0;
	} else {
		P2OUT &= ~BIT0;
	}
	if ((val >> 2) & 0x01) {
		P1OUT |= BIT7;
	} else {
		P1OUT &= ~BIT7;
	}
	if ((val >> 3) & 0x01) {
		P1OUT |= BIT6;
	} else {
		P1OUT &= ~BIT6;
	}

	// Toggle F-F CLK
	P1OUT |= BIT5;
	__delay_cycles(100);
	P1OUT &= ~BIT5;
	__delay_cycles(100);

	if ((val >> 4) & 0x01) {
		P2OUT |= BIT6;
	} else {
		P2OUT &= ~BIT6;
	}
	if ((val >> 5) & 0x01) {
		P2OUT |= BIT0;
	} else {
		P2OUT &= ~BIT0;
	}
	if ((val >> 6) & 0x01) {
		P1OUT |= BIT7;
	} else {
		P1OUT &= ~BIT7;
	}
	if ((val >> 7) & 0x01) {
		P1OUT |= BIT6;
	} else {
		P1OUT &= ~BIT6;
	}

	P1OUT &= ~BIT4;
	__delay_cycles(10000);
}

/********************************************************/


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
            temp_RX_data = EUSCI_B_I2C_slaveGetData(EUSCI_B0_BASE);
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
