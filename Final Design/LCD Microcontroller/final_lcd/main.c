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


#define SLAVE_ADDRESS 0x06

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
volatile byte temp_RX_data = 0;
volatile byte RX_data[32];
volatile byte *RX_str = RX_data;
volatile bool i2c_received = false;
volatile char num_chars = 0;
volatile int data_index = 0;


// Declare functions
 void delay_us(uint32_t s);
 void write_data(byte rs, byte rw, byte data);
 void handle_data(byte data);
 void I2Creceive();
 void loop();

 void init_lcd();
 void lcd_clear();
 void lcd_home();
 void lcd_no_display();
 void lcd_display();
 void lcd_no_blink();
 void lcd_blink();
 void lcd_no_cursor();
 void lcd_cursor();
 void lcd_scroll_left();
 void lcd_scroll_right();
 void lcd_left_to_right();
 void lcd_right_to_left();
 void lcd_autoscroll();
 void lcd_no_autoscroll();
// void create_char(byte, byte[]);
 void lcd_set_cursor(byte col, byte row);
 void lcd_write(byte);
 void lcd_command(byte);
 void lcd_print(byte *string);
 void lcd_send(byte value, byte mode);
 void lcd_write4bits(byte value);
 void lcd_write8bits(byte value);
 void lcd_pulse();

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

	lcd_print("Hello World!");
	__delay_cycles(1000000);
	lcd_clear();
	num_chars = 0;
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



 void delay_us(unsigned long s)
{
    unsigned long i;
	for (i = 0; i < s; i++) {
		__delay_cycles(1);
	}
}

 byte reverse_bits(byte b) {
   // b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   // b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   // b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   // return b;
   uint8_t i, b_ret;
   for(i = 0; i < 8; i++) {
	   b_ret |= (b & (1 << 7));
   }
   return b_ret;
}

// Handle received data from master
 void handle_data(byte data)
{
    i2c_received = false;
	if (data == '`' || data_index == 31) {
		EUSCI_B_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
		*(RX_str + data_index) = '\0';
		lcd_print(RX_str);
		memset(RX_str, 0, sizeof(RX_str));
		data_index = 0;
		temp_RX_data = 0;
		EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
	} else if (data == '~') {
		data_index = 0;
		lcd_clear();
		lcd_set_cursor(0, 0);
		num_chars = 0;
		temp_RX_data = 0;
	} else {
		*(RX_str + data_index) = data;
		data_index++;
		temp_RX_data = 0;
	}
	loop();
}


/************* LCD FUNCTIONS *******************/
 void init_lcd()
{
	displayfunction = LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS;
	__delay_cycles(50000);

	// To begin commands:
	P1OUT &= ~(BIT4 + BIT5 + BIT6);

	// Set LCD function 3 times
	lcd_command(LCD_FUNCTIONSET | displayfunction);
	__delay_cycles(4500);
	lcd_command(LCD_FUNCTIONSET | displayfunction);
	__delay_cycles(200);
	lcd_command(LCD_FUNCTIONSET | displayfunction);

	// Set # lines, font size, etc.
	lcd_command(LCD_FUNCTIONSET | displayfunction);
	__delay_cycles(200);

	// Start with display on, cursor off, blink off
	displaycontrol = LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON;
	lcd_display();
	__delay_cycles(200);

	// Clear
	lcd_clear();

	// Initialize to left-to-right text direction
	displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	// set the entry mode
	lcd_command(LCD_ENTRYMODESET | displaymode);
}

 void lcd_clear()
{
    lcd_command(LCD_CLEARDISPLAY);    // clearparams->display, set cursor position to zero
    __delay_cycles(2000);    // this command takes a long time!
}

 void lcd_home()
{
    lcd_command(LCD_RETURNHOME);    // set cursor position to zero
    __delay_cycles(2000);    // this command takes a long time!
}

 void lcd_set_cursor(byte col, byte row)
{
    const byte max_lines = sizeof(row_offsets) / sizeof(*(row_offsets));
    if ( row >= max_lines ) {
        row = max_lines - 1;        // we count rows starting w/0
    }
    if ( row >= numlines ) {
        row = numlines - 1;        // we count rows starting w/0
    }

    lcd_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn theparams->display on/off (quickly)
 void lcd_nodisplay() {
    displaycontrol &= ~LCD_DISPLAYON;
    lcd_command(displaycontrol | displaycontrol);
}

 void lcd_display() {
    displaycontrol |= LCD_DISPLAYON;
    lcd_command(displaycontrol | displaycontrol);
}

// Turns the underline cursor on/off
 void lcd_no_cursor() {
    displaycontrol &= ~LCD_CURSORON;
    lcd_command(displaycontrol | displaycontrol);
}

 void lcd_cursor() {
    displaycontrol |= LCD_CURSORON;
    lcd_command(displaycontrol | displaycontrol);
}

// Turn on and off the blinking cursor
 void lcd_no_blink() {
    displaycontrol &= ~LCD_BLINKON;
    lcd_command(displaycontrol | displaycontrol);
}

 void lcd_blink() {
    displaycontrol |= LCD_BLINKON;
    lcd_command(displaycontrol | displaycontrol);
}

// These commands scroll theparams->display without changing the RAM
 void lcd_scroll_left() {
    lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
 void lcd_scroll_right(){
    lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
 void lcd_left_to_right() {
	displaymode |= LCD_ENTRYLEFT;
    lcd_command(LCD_ENTRYMODESET | displaymode);
}

// This is for text that flows Right to Left
 void lcd_right_to_left() {
   	displaymode &= ~LCD_ENTRYLEFT;
    lcd_command(LCD_ENTRYMODESET | displaymode);
}

// This will 'right justify' text from the cursor
 void lcd_autoscroll() {
   	displaymode |= LCD_ENTRYSHIFTINCREMENT;
    lcd_command(LCD_ENTRYMODESET | displaymode);
}

// This will 'left justify' text from the cursor
 void lcd_no_autoscroll() {
   	displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    lcd_command(LCD_ENTRYMODESET | displaymode);
}

 void lcd_print(byte *string)
{
	byte *str_ptr = string;
	while ((str_ptr != 0) && (*str_ptr != '\0')) {
		lcd_write(*str_ptr);
		str_ptr++;
		__delay_cycles(50);
	}
}

 void lcd_command(byte value) {
    lcd_send(value, 0);
}

 void lcd_write(byte value) {
	 num_chars++;
	 if (num_chars == 17) {
		 lcd_set_cursor(0, 1);
	 } else if (num_chars == 33) {
		 lcd_clear();
		 lcd_set_cursor(0, 0);
		 num_chars = 0;
	 }
	 lcd_send(value, 1);
}

 void lcd_send(byte value, byte mode) {
	if (mode == 0) {
		P1OUT &= ~BIT1;
	} else if (mode == 1) {
		P1OUT |= BIT1;
	}
	P1OUT &= ~BIT0;
    lcd_write8bits(value);
}

 void lcd_pulse() {
	P1OUT &= ~BIT6;
    __delay_cycles(100);
    P1OUT |= BIT6;
    __delay_cycles(100);        // enable pulse must be >450ns
    P1OUT &= ~BIT6;
    __delay_cycles(10000);     // commands need > 37us to settle
}

//  void lcd_write4bits(uint8_t value) {
//     byte val = value ^ 0b11111111;
// 	int i;
//   	for (i = 0; i < 4; i++) {
//     	digital_write(data[i], (val >> i) & 0x01);
//     if (i == 2) {
//         digital_write(ff_clk, LOW);
//         delay_us(100);
//         digital_write(ff_clk, HIGH);
//     }
//   }
//   lcd_pulse();
// }

 void lcd_write8bits(byte value) {
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
