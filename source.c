#include <p18f452.h>
#pragma config WDT = OFF

#define TRUE 0x01
#define FALSE 0x00
#define SIZE 0x02

#define 	RS_PIN				LATCbits.LATC0		// RS PIN CONNECTED TO RC0
#define 	RW_PIN 				LATCbits.LATC1		// RW PIN CONNECTED TO RC1
#define 	EN_PIN				LATCbits.LATC2		// ENABLE PIN CONNECTED TO RC2


#define CLOCK_BURST_WRITE 0x3F						// BURST MODE FOR WRITING
#define CLOCK_BURST_READ 0xBF						// BURST MODE FOR READING
#define DUMMY_DATA 	0x00							// DUMMY DATA 
#define CONTROL_WRITE 0x0F							// CONTROL REGISTER USED TO DEFINED THE WP BIT
#define SLAVE_SELECT LATCbits.LATC6					// SLAVE SELECT PIN IS CONNECTED TO RC6

volatile unsigned char bytes[] = {0x50, 0x59, 0x23, 0x31, 0x12, 0x07, 0x24, 0x00}, p = 0;	// THIS ARRAY SAVES THE VALUES OF RTC WHEN A READ OCCURS
unsigned char days[7][20] = {"SUN,", "MON,", "TUE,", "WED,", "THU,", "FRI,", "SAT,"};	// THIS ARRAY CONTAINS ABBREVIATION OF DAYS NAME
unsigned char stat = 0;

enum state				// ENUMERATED TYPE TO SPECIFY THE STATE OF SPI
{
	READ = 0, 			
	WRITE = 1
}spiState = WRITE;

enum registers			// ENUMERATED TYPE TO SPECIFY EITHER SAVE MODE OR DUMMY MODE
{
	DUMMY,
	SAVE
}reg;

enum devices
{
	ADC,
	SPI
}device = ADC;


union type
{
	unsigned int word;
	unsigned char byte[SIZE];
}result;

void clearHome(void);
void capacitorTime(void);
void displayResult(void);
void initialization(void);			
void delay250ms(void);		
void delay3us(void);			
void command(void);
void busyFlag(void);				
void data(void);	
void timerOne(void);
void home(unsigned char value);
void displayTime(void);
void displayDate(void);
void clearSecondLine(void);
void displayTemperature(void);
void extractDigits(float temp);
void displayData(unsigned int value);

#pragma interrupt interruptFunction
void interruptFunction(void)
{
	if(INTCONbits.TMR0IF)
	{
		T0CONbits.TMR0ON = 0;	
		clearHome();
		if(device == ADC)
		{
			SLAVE_SELECT = 1;
			PIE1bits.SSPIE = 0;			
			PIR1bits.SSPIF = 0;
			PIE1bits.ADIE = 1;			
			PIR1bits.ADIF = 0;
			capacitorTime();
			ADCON0bits.GO = 1;
			device = SPI;
		}
		else if(device == SPI)
		{
			device = ADC;
			PIE1bits.ADIE = 0;
			p = 0;			
			PIR1bits.ADIF = 0;
			PIE1bits.SSPIE = 1;			
			PIR1bits.SSPIF = 0;		
			SLAVE_SELECT = 0;
			if(!stat)
			{
				SSPBUF = CLOCK_BURST_WRITE;
				stat = 1;		
			}
			else
				SSPBUF = CLOCK_BURST_READ;
		}
		TMR0H = 0xC2;
		TMR0L = 0xF7;
		INTCONbits.TMR0IF = 0;
		T0CONbits.TMR0ON = 1;
	}
	else if(PIE1bits.ADIE)
	{
		if(PIR1bits.ADIF)
		{
			PIR1bits.ADIF = 0;
			displayTemperature();
			capacitorTime();
			ADCON0bits.GO = 1;
		}
	}
	else if(PIE1bits.SSPIE)
	{
		if(PIR1bits.SSPIF)	
		{
			PIR1bits.SSPIF = 0;
			if(spiState == WRITE)
			{
				if(p < 8)
					SSPBUF = bytes[p++];
				else
				{
					SLAVE_SELECT = 1;
					spiState = READ;
					reg = DUMMY;
					SLAVE_SELECT = 0;
					p = 0;
					SSPBUF = CLOCK_BURST_READ;
				}	
			}
			else
			{
				if(reg == DUMMY)				// SEND DUMMY DATA
				{
					SSPBUF = DUMMY_DATA;
					reg = SAVE;
				}
				else						// STORE THE RECEIVED DATA FROM MISO LINE
				{
					reg = DUMMY;
					bytes[p++] = SSPBUF;	
					if(p < 8)
						PIR1bits.SSPIF = 1;		// TRIGGER A SOFTWARE SPI INTERRUPT 
					else
					{
						SLAVE_SELECT = 1;		// DISCONNECTED THE SLAVE
						p = 0;
						spiState = READ;
						reg = DUMMY;
						displayResult();		// DISPLAY DATA INTO LCD		
						SLAVE_SELECT = 0;						
						SSPBUF = CLOCK_BURST_READ;
					}
				}
			}
		}		
	}
}

#pragma code vector = 0x00008
void vector(void)
{
	_asm
		GOTO interruptFunction
	_endasm
}
#pragma code

void main(void)
{
	TRISA = 0xFF;
	TRISD = 0x00;				// ALL PINS ARE OUTPUT
	TRISC = 0x90;				// MAKE SERIAL PINS AS OUTPUT + LCD CONTROL PINS OUTPUT
	SSPSTAT = 0xC0;				// CONFIGURATION OF SPI PROTOCOL
	SSPCON1 = 0x20;
	ADCON0 = 0x41;
	ADCON1 = 0x85;	
	initialization();			// INITIALIZATION OF LCD(CLEAR HOME, CURSOR OFF, 5*7 PIXEL SIZE)
	INTCONbits.GIE = 1;			// GLOBAL INTERRUPT ENABLE
	INTCONbits.TMR0IE = 1;
	INTCONbits.TMR0IF = 0;
	INTCONbits.PEIE = 1;
	T0CON = 0x06;
	TMR0H = 0xC2;
	TMR0L = 0xF7;
	T0CONbits.TMR0ON = 1;
	while(TRUE);
}
void displayTemperature(void)
{
	static unsigned char state = 0;
	static float oldTemp = 0.0;
	float newTemp = 0.0;
	result.byte[0] = ADRESL;
	result.byte[1] = ADRESH;	
	if(!state)
	{
		state = 0x01;
		oldTemp = (result.word*150.0)/1023.0;
		extractDigits(oldTemp);
	}
	else
	{
		newTemp = (result.word*150.0)/1023.0;
		if(newTemp != oldTemp)
		{
			oldTemp = newTemp;
			clearHome();
			extractDigits(oldTemp);
		}
	}
}
void extractDigits(float temp)
{
	unsigned char decimal = 0, i = 0;
	unsigned char temper[] = "TEMPERATURE IS", celsius[] = " Celsius";
	decimal = (unsigned char)temp;
	while(temper[i] != '\0')
	{
		LATD = temper[i++];
		data();
	}
	home(0xC4);
	displayData(decimal);
	for(i = 0; celsius[i] != '\0'; ++i)	
	{
		LATD = celsius[i];
		data();
	}
}
void clearHome(void)
{
	LATD = 0x01;
	command();
	T1CON = 0x00;
	TMR1H = 0xF9;
	TMR1L = 0x98;
	timerOne();
	LATD = 0x80;
	command();
	busyFlag();
}
void displayData(unsigned int value)
{
	unsigned char stack[3];
	signed char TOP = -1;
	while(value != 0)
	{
		stack[++TOP] = value%10;
		value /= 10;
	}
	while(TOP != -1)
	{
		LATD = stack[TOP--] + 0x30;
		data();
	}
}
void home(unsigned char value)		// THIS FUNCTION IS USED TO CHANGE THE POSITION OF THE CURSOR(SECOND LINE, FIRST LINE)
{
	LATD = value;
	command();
	busyFlag();
}
void displayTime(void)				// THIS IS USED FOR DISPLAYING TIME 
{
	signed char m = 2;
	unsigned char string[] = "TIME : ", j = 0;
	home(0x80);
	while(string[j] != '\0')
	{
		LATD = string[j++];
		data();
	}
	while(m >= 0)
	{
		LATD = 0x30 + ((bytes[m] & 0xF0) >> 4);		// EXTRACT THE HIGH BYTE TO DISPLAY IT
		data();
		LATD = 0x30 + (bytes[m] & 0x0F);			// EXTRACT THE LOW BYTE TO DISPLAY IT
		data();
		if(m != 0)									// DISPLAY TIME IN THIS FORMAT : x:x:x
		{
			LATD = ':';			
			data();
		}
		--m;
	}
}
void displayDate(void)			// THIS FUNCTION DISPLAYS DATE : x/x/20x
{		
	unsigned char m = (bytes[5] - 1), n = 0, date = bytes[3], month = bytes[4], year = bytes[6];
	home(0xC0);
	if(m > 7)
		m = 0;
	home(0xC0);
	for(; days[m][n] != '\0'; ++n)
	{
		LATD = days[m][n];
		data();
	}	
	LATD = 0x30 + ((date & 0xF0) >> 4);
	data();
	LATD = 0x30 + (date & 0x0F);
	data();
	LATD = '/';
	data();
	LATD = 0x30 + ((month & 0xF0) >> 4);
	data();
	LATD = 0x30 + (month & 0x0F);
	data();
	LATD = '/';
	data();
	LATD = 0x32;			
	data();	
	LATD = 0x30;
	data();
	LATD = 0x30 + ((year & 0xF0) >> 4);
	data();
	LATD = 0x30 + (year & 0x0F);
	data();
}
void displayResult(void)
{
	displayTime();
	displayDate();
}
void initialization(void)		
{
	LATD = 0x38;
	command();
	delay250ms();
	LATD = 0x01;
	command();
	delay250ms();
	LATD = 0x0C;
	command();
	delay250ms();
}
void delay250ms(void)
{
	T1CON = 0x20;
	TMR1H = 0x0B;
	TMR1L = 0xDC;
	timerOne();		
}
void delay3us(void)				
{
	T1CON = 0x00;
	TMR1H = 0xFF;
	TMR1L = 0xFD;
	timerOne();
}
void command(void)
{
	RS_PIN = 0;
	RW_PIN = 0;
	EN_PIN = 1;
	delay3us();
	EN_PIN = 0;
}
void data(void)
{
	RS_PIN = 1;
	RW_PIN = 0;
	EN_PIN = 1;
	delay3us();
	EN_PIN = 0;
	busyFlag();
}
void busyFlag(void)
{
	RS_PIN = 0;
	RW_PIN = 1;	
	TRISDbits.TRISD7 = 1;
	do
	{
		EN_PIN = 1;
		delay3us();
		EN_PIN = 0;
	}while(PORTDbits.RD7 == 1);
	EN_PIN = 0;
	TRISDbits.TRISD7 = 0;
}
void timerOne(void)
{
	PIR1bits.TMR1IF = 0;
	T1CONbits.TMR1ON = 1;
	while(PIR1bits.TMR1IF == 0);
	PIR1bits.TMR1IF = 0;
	T1CONbits.TMR1ON = 0;	
}
void capacitorTime(void)				// HELPER FUNCTION USED TO WAIT UNTIL CAPACITOR IS CHARGED TO START CONVERSION
{
	T1CON = 0x00;			// TIMER0 MODE 8, NO PRESCALER
	TMR1H = 0xFF;
	TMR1L = 0xEC;			// INITIAL VALUE OF TIMER0 IN MODE 8 BIT
	PIR1bits.TMR1IF = 0;
	T1CONbits.TMR1ON = 1;		// START TIMER 0
	while(!PIR1bits.TMR1IF);	// POLLING TIMER 0 FLAG
	PIR1bits.TMR1IF = 0;		// CLEAR TIMER0 FLAG
	T1CONbits.TMR1ON = 0;		// STOP TIMER0
}
