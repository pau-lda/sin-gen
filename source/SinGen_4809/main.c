/*
 * SinGen_4809.c
 *
 * Created: 07.12.2021 19:04:47
 * Author : Pauli
 */ 

#define F_CPU 20000000
#define SINTABLESIZE 64
#define DAC_RESOLUTION 1024
#define PHASE (SINTABLESIZE / 4)

//#define DEBUG

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

void init();
inline void Master_Transmit16Bit(uint8_t, uint8_t);
void initializeSinTable();
inline void transferSinTable(int);
uint16_t getADC10Bit();
void initTimerPoti();
void initTimerNormal();

uint16_t sine[SINTABLESIZE];

union LT1661Data
{
	struct {
		uint16_t data:12,
		header:4;
	}wr;
	struct {
		uint8_t hb;
		uint8_t lb;
		
	}rd;
};

union LT1661Data _lt1661data;

ISR (TCA0_CMP0_vect)
{
	static int _index1 = 0;
	static int _index2 = PHASE;

	_index1++;
	//_index2++;
		
	_index1 = _index1 % SINTABLESIZE;
	_lt1661data.wr.header = 0x9;
	transferSinTable( _index1 );
		
	//_index2 = _index2 % SINTABLESIZE;
	//_lt1661data.wr.header = 0xA;
	//transferSinTable(_index2);

	// Flag has to be cleared manually
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
}

int main(void)
{
	init();
	initializeSinTable();
	
	//Enable global Interrupts
	sei();
	
	while (1)
	{
		//Write full 10Bit to timer1 CMP0-Buffer register
		//Gets written to CMP0 register on next CMP0-interrupt
		TCA0.SINGLE.CMP0BUF = getADC10Bit();
	}
}

void initializeSinTable()
{
	double x = 0;
	for( int i = 0; i < SINTABLESIZE; i++)
	{
		x = 0.95 * sin(i * 2*(M_PI/SINTABLESIZE)) + 1.0;	// shift 0 ... 2
		x *= (double)(DAC_RESOLUTION >> 1);					// shift 0 ... 2 to 0 ... DAC_RESOLUTION (1023)
		x *= 4.0;											// Scale to 0 ... 5V
		sine[i] = (uint16_t)x;
	}
}

inline void transferSinTable(int _index)
{
	uint16_t sinVal = sine[_index];
	_lt1661data.wr.data = sinVal & 0xFFF; //limit to 12 Bit
	
	Master_Transmit16Bit(_lt1661data.rd.hb, _lt1661data.rd.lb);
}
inline void Master_Transmit16Bit(uint8_t hb, uint8_t lb)
{
	//SS to HIGH while sending Bits
	PORTD.OUT &= ~(1 << PIN1_bp);
	
	SPI0.DATA = lb;
	while(!(SPI0.INTFLAGS & SPI_IF_bm)){;}

	SPI0.DATA = hb;
	while(!(SPI0.INTFLAGS & SPI_IF_bm)){;}
	
	//SS to LOW again
	PORTD.OUT |= (1 << PIN1_bp);
}

uint16_t getADC10Bit() 
{
	// start an ADC conversion
	ADC0.COMMAND = ADC_STCONV_bm;

	// Wait for conversion to finish
	while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) ){;}
	
	return ADC0.RES;
}
void init()
{
	//	--Pins--	//
	//Set standard MOSI, MISO, SCK, SS pins
	PORTC.DIRSET = (1 << PIN0_bp) | (0 << PIN1_bp) | (1 << PIN2_bp) | (1 << PIN3_bp);
	//Set new SS output for 16Bit transmit
	PORTD.DIRSET = (1 << PIN1_bp);
	//ADC-Input
	PORTD.DIRSET = (0 << PIN0_bp);
	
	//Preset SS
	PORTD.OUT |= (1 << PIN1_bp);
	
	//	--TIMER1--	//
	initTimerPoti();
	
	//	--SPI--	//
	PORTMUX.TWISPIROUTEA |= PORTMUX_SPI00_bm; //SPI on PORTC[3:0] -> [_SS,SCK,MISO,MOSI]
	//Controllregister
	SPI0.CTRLA |= SPI_CLK2X_bm /* Double Speed */ | SPI_MASTER_bm /* Set as Master Device */ | SPI_PRESC_DIV4_gc /* CLK Prescaler */| SPI_ENABLE_bm /* SPI enable */ ; 
	
	//  --ADC-- //
	// PD0 is input by default, but we need to disable its "digital input buffer" and Pull-Up-res
	PORTD.PIN0CTRL &= ~PORT_ISC_gm;
	PORTD.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN0CTRL &= ~PORT_PULLUPEN_bm;
	// p.408
	ADC0.CTRLC = ADC_SAMPCAP_bm /* reduced input impdance */ | ADC_REFSEL_VDDREF_gc /*Vdd as reference */ | ADC_PRESC_DIV4_gc /* CLK Prescaler */;
	ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc;	// Select AIN0 (first on PORTD?) as input channel
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc /* 10-Bit res */ | ADC_ENABLE_bm /* ADC enable */;
}

void initTimerPoti()
{
	/*	---Frquency Waveform Generation: p.191---	*/

	//Set AND ENABLE waveform to Frequency to be able to change the TOP value of timer
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc | TCA_SINGLE_CMP0EN_bm;
	
	//Set this to only count clock ticks and not events
	TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
	
	//Set timer to compare			p.207
	//TCA0.SINGLE.INTCTRL = (1 << PIN4_bp);
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;
	
	//16-Bit value of compare-register	p.213
	TCA0.SINGLE.CMP0 = 0x00FF;
	
	//set prescaler and enable single mode
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}