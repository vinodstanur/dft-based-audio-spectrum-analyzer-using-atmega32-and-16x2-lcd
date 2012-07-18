#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#define N 32
#define _N32_
#include "lookup.h"
#define RS PD6
#define EN PD5
#define LCD_NIBBLE PORTC

void adc_init();
uint16_t adc_read();
void TRANSFORM();
void timer1_init();
void LCD_STROBE(void);
void lcd_data(unsigned char c);
void lcd_cmd(unsigned char c);
void lcd_clear(void);
void lcd_init();
void lcd_print(char *p, char l);
void lcd_fill_custom();

uint8_t lcd_buf1[16];
uint8_t lcd_buf2[16];
int32_t fx[N];
int32_t Fu[N/2][2];

void main()
{
	uint8_t mag;
	int i,j, temp_value;
	uint8_t temp_index;
	adc_init();
	lcd_init();
	lcd_fill_custom();
	lcd_print("DFT SPECTROMETER",1);
	lcd_print("0Hz - 10KHz(16)",2);
	_delay_ms(1000);
	lcd_clear();
	timer1_init();
	while(1) {
		TCNT1 = 0;
		TIFR |= 1<<OCF1A;
		for(i=0;i<N;i++) {
			while((TIFR & (1<<OCF1A)) == 0);
			fx[i] = ((int16_t)adc_read());
			TIFR |= 1<<OCF1A;	
		}
		TRANSFORM();
		lcd_cmd(0xc0);
		for(i =1; i<N/2; i++) {
			if(Fu[i][0]<0)Fu[i][0]*=-1;
			if(Fu[i][1]<0)Fu[i][1]*=-1;
			mag = (uint8_t)(Fu[i][0] + Fu[i][1])/4;
			if((mag)>7) {
				lcd_buf1[i] = (mag) - 7 - 1;
				if(lcd_buf1[i] > 7)
					lcd_buf1[i] = 7;
				lcd_buf2[i] = 7;
			}
			else {
				lcd_buf1[i] = ' ';
				lcd_buf2[i] = mag;
			}
		}
		lcd_cmd(0x80);
		for(i=1;i<16;i++)
			lcd_data(lcd_buf1[i]);
		lcd_cmd(0xc0);
		for(i=1;i<16;i++)
			lcd_data(lcd_buf2[i]);
		
	}
}

void TRANSFORM()
{
	int16_t count,degree;
	uint8_t u,k;
	count = 0;
	for (u=0; u<N/2; u++) {
		for (k=0; k<N; k++) {
			degree = (uint16_t)pgm_read_byte_near(degree_lookup + count)*2;
			count++;
		   	Fu[u][0] +=  fx[k] * (int16_t)pgm_read_word_near(cos_lookup + degree);
		    Fu[u][1] += -fx[k] * (int16_t)pgm_read_word_near(sin_lookup + degree);
		}
		    Fu[u][0] /= N;
		    Fu[u][0] /= 10000;
		    Fu[u][1] /= N;
		    Fu[u][1] /= 10000;
	}
}

void timer1_init()
{
	TCCR1B = (1<<WGM12)|(1<<CS10);
	OCR1A = 800;	
}

void adc_init()
{
	ADMUX = 0b11000000;
	ADCSRA =0b10000010;
}

uint16_t adc_read()
{
	volatile uint16_t retl,reth;
	ADCSRA |= 1<<ADSC;
	while(!ADIF);
	ADCSRA |= 1<<ADIF;
	retl = ADCL;
	reth = ADCH;
	reth<<=8;
	reth|=retl;
	return reth;
}

void LCD_STROBE(void)
{
    PORTD |= (1 << EN);
    _delay_us(1);
    PORTD &= ~(1 << EN);
}
 
void lcd_data(unsigned char c)
{
    PORTD |= (1 << RS);
    _delay_us(50);
    LCD_NIBBLE = (c >> 4)|(LCD_NIBBLE&0xf0);
    LCD_STROBE();
    LCD_NIBBLE = (c)|(LCD_NIBBLE&0XF0);
    LCD_STROBE();
}
 
void lcd_cmd(unsigned char c)
{
    PORTD &= ~(1 << RS);
    _delay_us(50);
    LCD_NIBBLE = (c >> 4)|(LCD_NIBBLE&0xf0);
    LCD_STROBE();
    LCD_NIBBLE = (c)|(LCD_NIBBLE&0XF0);
    LCD_STROBE();
}
 
void lcd_clear(void)
{
    lcd_cmd(0x01);
    _delay_ms(5);
}
 
void lcd_init()
{
    DDRC = 0b00001111;
    DDRD |= (1 << RS)|(1 << EN);
 	PORTC |= (1<<PC4);
    _delay_ms(15);
    lcd_cmd(0x30);
    _delay_ms(1);
    lcd_cmd(0x30);
    _delay_us(100);
    lcd_cmd(0x30);
    lcd_cmd(0x28);
    lcd_cmd(0x28);
    lcd_cmd(0x0c);
    lcd_clear();
    lcd_cmd(0x6);
}

void lcd_print(char *p, char l)
{
	if(l==1)lcd_cmd(0x80);
	else lcd_cmd(0xc0);
	while(*p)
		lcd_data(*p++);
}
	
void lcd_fill_custom()
{
	uint8_t i,j;
	i=0;j=0;
	lcd_cmd(64);
	for(i=1;i<=8;i++) {
		for(j=8;j>i;j--) 
			lcd_data(0);
		for(j=i;j>0;j--)
			lcd_data(0xff);
	}
}

