#define F_CPU 1000000UL  // 8 MHz clock speed
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Pin Definitions
#define Tang PINB0  // Increase button
#define Giam PINB1  // Decrease button
#define Dao  PINB2  // Direction button (Clockwise/Counter-clockwise)
#define Stop PINB3  // Stop button

// LCD Pin Definitions
#define LCD_RS PA0
#define LCD_EN PA2
#define LCD_RW PA1
#define LCD_D4 PA4
#define LCD_D5 PA5
#define LCD_D6 PA6
#define LCD_D7 PA7
#define LCD_DATA PORTA  // Data lines of LCD
#define LCD_CTRL PORTA  // Control lines of LCD
#define LCD_DATA_DDR DDRA  // Data direction register for LCD
#define LCD_CTRL_DDR DDRA  // Control direction register for LCD

// Global variables
unsigned long encoder = 0;  // Encoder count
unsigned char countT0 = 0, countT2 = 0;  // Timer counts
unsigned int speed;  // Speed in RPM
unsigned int temp = 0;  // Temporary value for pulse count (0-7999)
unsigned char phantram = 0;  // Percentage for pulse count
unsigned char dao_state = 0;  // Direction state: 0 for CW, 1 for CCW
unsigned char stop_state = 0;  // Stop state flag

// External Interrupt ISR (for encoder)
ISR(INT0_vect)
{
	encoder++;  // Increment encoder count
}

// Timer0 ISR (used to measure the time interval and calculate speed)
ISR(TIMER0_OVF_vect)
{
	TCCR0 = (0 << CS02) | (0 << CS01) | (0 << CS00);  // Stop the timer temporarily
	if(countT0 >= 20) {
		countT0 = 0;
		speed = ((float)encoder / 102.4) * 60 ;  // Calculate speed in RPM
		encoder = 0;  // Reset encoder count for the next cycle
	}
	countT0++;  // Increment countT0
	TCCR0 = (1 << CS02) | (0 << CS01) | (1 << CS00);  // Restart timer with prescaler 1024
	TCNT0 = 0x06;
}

// Timer2 ISR (handles button presses)
ISR(TIMER2_OVF_vect)
{
	TCCR2 = (0 << CS22) | (0 << CS21) | (0 << CS20);  // Stop the timer temporarily
	if(countT2 >= 5) {
		countT2 = 0;
		// Handle button press logic here if necessary
	}
	countT2++;  // Increment countT2
	TCNT2 = 0x00;
	TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);  // Restart timer with prescaler 1024
}

// Button initialization: configure input pins for buttons
void init_buttons() {
	DDRB = 0b00000000;  // Set all Port B pins as input
	PORTB = 0b00001111;  // Enable pull-up resistors on pins PB0 to PB3
}

// Function to convert long number to string
void long_to_string(long number, char* str) {
	sprintf(str, "%ld", number);  // Convert long to string
}

// LCD command function to send commands to the LCD
void lcd_command(unsigned char cmd) {
	LCD_DATA = (LCD_DATA & 0x0F) | (cmd & 0xF0);  // Send high 4 bits of command
	LCD_CTRL &= ~(1 << LCD_RS);  // Select command mode
	LCD_CTRL &= ~(1 << LCD_RW);  // Write mode
	LCD_CTRL |= (1 << LCD_EN);  // Enable LCD
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);  // Disable LCD
	_delay_us(200);

	LCD_DATA = (LCD_DATA & 0x0F) | (cmd << 4);  // Send low 4 bits of command
	LCD_CTRL |= (1 << LCD_EN);  // Enable LCD
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);  // Disable LCD
	_delay_ms(2);
}

// LCD data function to send data (characters) to the LCD
void lcd_data(unsigned char data) {
	LCD_DATA = (LCD_DATA & 0x0F) | (data & 0xF0);  // Send high 4 bits of data
	LCD_CTRL |= (1 << LCD_RS);  // Select data mode
	LCD_CTRL &= ~(1 << LCD_RW);  // Write mode
	LCD_CTRL |= (1 << LCD_EN);  // Enable LCD
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);  // Disable LCD
	_delay_us(200);

	LCD_DATA = (LCD_DATA & 0x0F) | (data << 4);  // Send low 4 bits of data
	LCD_CTRL |= (1 << LCD_EN);  // Enable LCD
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);  // Disable LCD
	_delay_ms(2);
}

// LCD initialization function
void lcd_init() {
	LCD_DATA_DDR = 0xF0;  // Set upper 4 bits of PORTA as output for data
	LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_EN);  // Set control pins as output

	_delay_ms(20);  // Wait for LCD to power up
	lcd_command(0x02);  // Initialize in 4-bit mode
	lcd_command(0x28);  // Set LCD to 2 lines, 5x8 font
	lcd_command(0x0C);  // Display ON, cursor OFF
	lcd_command(0x06);  // Move cursor to the right
	lcd_command(0x01);  // Clear the screen
}

// LCD print function to display a string on the LCD
void lcd_print(const char *str) {
	while (*str) {
		lcd_data(*str++);  // Send each character to the LCD
	}
}

// LCD cursor positioning function
void lcd_set_cursor(unsigned char x, unsigned char y) {
	unsigned char pos[] = {0x80, 0xC0};  // Row 1 and Row 2 start positions
	lcd_command(pos[y] + x);  // Set cursor position based on x, y
}

// Function to display speed and percentage on the LCD
void lcd_display_info(int speed, int phantram) {
	phantram = ((float)temp / 7999) * 100;
	if (phantram > 100)phantram =100;   // Calculate percentage of pulse count

	lcd_set_cursor(0, 0);  // Set cursor to first line, first column
	lcd_print("PWM ");
	lcd_set_cursor(6, 0);  // Set cursor to first line, 6th column
	char buffer[10];
	long_to_string(phantram, buffer);  // Convert percentage to string
	lcd_print(buffer);
	lcd_set_cursor(9, 0);  // Set cursor to first line, 9th column
	lcd_print("%");
    if (phantram<100){
		lcd_set_cursor(8, 0);  // Set cursor to first line, first column
	    lcd_print(" ");
	}
	lcd_set_cursor(0, 1);  // Set cursor to second line, first column
	lcd_print("SPEED ");
	lcd_set_cursor(6, 1);  // Set cursor to second line, 6th column
	char buffer1[10];
	long_to_string(speed, buffer1);  // Convert speed to string
	lcd_print(buffer1);  // Display speed value
	lcd_set_cursor(9, 1);  // Set cursor to second line, 9th column
	lcd_print("RPM");

	// Display rotation direction
	if (dao_state == 0) {
		for (int i = 12; i <= 14; i++) lcd_set_cursor(i, 0), lcd_data(' ');
		lcd_set_cursor(12, 0);
		lcd_print("CW");
		} else {
		for (int i = 12; i <= 14; i++) lcd_set_cursor(i, 0), lcd_data(' ');
		lcd_set_cursor(12, 0);
		lcd_print("CCW");
	}
}

int main(void) {
	// Initialize Port A, B, C, D for input/output
	DDRA  = 0b00000000;  // All pins of Port A as input
	PORTA = 0b00000000;  // Disable pull-ups on Port A

	DDRB  = 0b00000000;  // All pins of Port B as input
	PORTB = 0b00001111;  // Enable pull-ups on PB0 to PB3
	DDRC  = 0b00000000;  // All pins of Port C as input
	PORTC = 0b00000000;  // Disable pull-ups on Port C

	DDRD  = 0b00101010;  // PD5, PD3, PD1 as output, others as input
	PORTD = 0b00000010;  // Set PD1 high

	// Configure Timer0 for 1024 prescaler
	TCCR0 = (1 << CS02) | (0 << CS01) | (1 << CS00);
	TCNT0 = 0x06;

	// Configure Timer1 for PWM
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	ICR1H  = 0x1F;
	ICR1L  = 0x3F;
	OCR1AH = 0x00;
	OCR1AL = 0x00;
	OCR1BH = 0x00;
	OCR1BL = 0x00;

	// Configure Timer2 for button press handling
	TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);  // Prescaler 1024
	TCNT2 = 0x06;

	// Enable timer interrupts
	TIMSK = (0 << OCIE2) | (1 << TOIE2) | (0 << TICIE1) | (0 << OCIE1A) | (0 << OCIE1B) | (0 << TOIE1) | (0 << OCIE0) | (1 << TOIE0);

	// Enable external interrupt for encoder
	GICR |= (0 << INT1) | (1 << INT0) | (0 << INT2);  // Enable INT0
	MCUCR = (0 << ISC11) | (0 << ISC10) | (1 << ISC01) | (0 << ISC00);  // Falling edge on INT0
	MCUCSR = (0 << ISC2);  // Disable INT2 interrupt
	GIFR = (0 << INTF1) | (1 << INTF0) | (0 << INTF2);  // Clear INT0 interrupt flag
	sei();  // Enable global interrupts

	// Initialize LCD
	lcd_init();

	// Main loop
	while (1) {
		uint8_t pinb_state = PINB;  // Read button states from Port B

		// Handle "Increase" button
		if (stop_state == 0) {
			if (!(pinb_state & (1 << Tang))) {
				_delay_ms(100);  // Debounce delay
				while (!(PINB & (1 << Tang))) {
					temp += 80;
					if (temp >= 7999) temp = 7999;  // Limit the temp value to 7999
					if (!(PORTD & (1 << PIND3))) {  // Check direction (clockwise or counter-clockwise)
						OCR1A = temp;  // Set duty cycle for clockwise rotation
						} else {
						OCR1A = 7999 - temp;  // Set duty cycle for counter-clockwise rotation
					}
					lcd_display_info(speed, phantram);  // Update LCD display with speed and percentage
				}
			}
			// Handle "Decrease" button
			if (!(pinb_state & (1 << Giam))) {
				_delay_ms(100);  // Debounce delay
				while (!(PINB & (1 << Giam))) {
					if (temp >= 80) {
						temp -= 80;  // Decrease duty cycle
						if (!(PORTD & (1 << PIND3))) {
							OCR1A = temp;  // Update duty cycle for clockwise
							} else {
							OCR1A = 7999 - temp;  // Update duty cycle for counter-clockwise
						}
						lcd_display_info(speed, phantram);  // Update LCD display
						} else {
						temp = 0;  // Prevent temp from going below 0
					}
				}
			}

			// Handle "Direction" button (switch between CW/CCW)
			if (!(pinb_state & (1 << Dao))) {
				_delay_ms(100);
				// Debounce delay
				if (!(pinb_state & (1 << Dao))) {
					PORTD ^= (1 << PIND3);  // Toggle direction
					dao_state = (dao_state + 1) % 2;  // Toggle between CW and CCW
					_delay_us(100);  // Debounce delay
				}
			}
		}
		// Handle "Stop" button
		if (!(pinb_state & (1 << Stop))) {
			_delay_ms(100);  // Debounce delay
			if (!(pinb_state & (1 << Stop))) {
				stop_state = !stop_state;  // Toggle stop state
				PORTD ^= (1 << PIND1);  // Toggle stop signal
				_delay_us(100);  // Debounce delay

			}
		}

		// Update the duty cycle based on the direction
		if (!(PORTD & (1 << PIND3))) {
			OCR1A = temp;  // Clockwise
			} else {
			OCR1A = 7999 - temp;  // Counter-clockwise
		}

		lcd_display_info(speed, phantram);  // Display current info on LCD
		_delay_ms(100);  // Update every 100ms
	}
}