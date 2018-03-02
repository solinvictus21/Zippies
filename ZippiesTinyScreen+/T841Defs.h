/*
T841Defs.h - Last modified 30 July 2015

These are register definitions for the ATTiny841, generated from files
included in the AVR-GCC project.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Written by Ben Rose for TinyCircuits.

The latest version of this library can be found at https://tiny-circuits.com/
*/

//Registers and associated bit numbers

#define T841_ADCSRB  0x20+0x04
#define T841_ADTS0   0
#define T841_ADTS1   1
#define T841_ADTS2   2
#define T841_ADLAR   3

#define T841_ADCSRA  0x20+0x05
#define T841_ADPS0   0
#define T841_ADPS1   1
#define T841_ADPS2   2
#define T841_ADIE    3
#define T841_ADIF    4
#define T841_ADATE   5
#define T841_ADSC    6
#define T841_ADEN    7

#define T841_ADCL    0x20+0x06
#define T841_ADCH    0x20+0x07

#define T841_ADMUXB  0x20+0x08
#define T841_GSEL0   0
#define T841_GSEL1   1
#define T841_REFS0   5
#define T841_REFS1   6
#define T841_REFS2   7

#define T841_ADMUXA  0x20+0x09
#define T841_MUX0    0
#define T841_MUX1    1
#define T841_MUX2    2
#define T841_MUX3    3
#define T841_MUX4    4
#define T841_MUX5    5

#define T841_ACSR0A  0x20+0x0A
#define T841_ACIS00  0
#define T841_ACIS01  1
#define T841_ACIC0   2
#define T841_ACIE0   3
#define T841_ACI0    4
#define T841_ACO0    5
#define T841_ACPMUX2 6
#define T841_ACD0    7

#define T841_ACSR0B  0x20+0x0B
#define T841_ACPMUX0 0
#define T841_ACPMUX1 1
#define T841_ACNMUX0 2
#define T841_ACNMUX1 3
#define T841_ACOE0   4
#define T841_HLEV0   6
#define T841_HSEL0   7

#define T841_ACSR1A  0x20+0x0C
#define T841_ACIS10  0
#define T841_ACIS11  1
#define T841_ACIC1   2
#define T841_ACIE1   3
#define T841_ACI1    4
#define T841_ACO1    5
#define T841_ACBG1   6
#define T841_ACD1    7

#define T841_ACSR1B  0x20+0x0D
#define T841_ACME1   2
#define T841_ACOE1   4
#define T841_HLEV1   6
#define T841_HSEL1   7

#define T841_TIFR1   0x20+0x0E
#define T841_TOV1    0
#define T841_OCF1A   1
#define T841_OCF1B   2
#define T841_ICF1    5

#define T841_TIMSK1  0x20+0x0F
#define T841_TOIE1   0
#define T841_OCIE1A  1
#define T841_OCIE1B  2
#define T841_ICIE1   5

#define T841_TIFR2   0x20+0x10
#define T841_TOV2    0
#define T841_OCF2A   1
#define T841_OCF2B   2
#define T841_ICF2    5

#define T841_TIMSK2  0x20+0x11
#define T841_TOIE2   0
#define T841_OCIE2A  1
#define T841_OCIE2B  2
#define T841_ICIE2   5

#define T841_PCMSK0  0x20+0x12

#define T841_GPIOR0  0x20+0x13

#define T841_GPIOR1  0x20+0x14

#define T841_GPIOR2  0x20+0x15

#define T841_PINB    0x20+0x16
#define T841_PINB3   3
#define T841_PINB2   2
#define T841_PINB1   1
#define T841_PINB0   0

#define T841_DDRB    0x20+0x17
#define T841_DDRB3   3
#define T841_DDRB2   2
#define T841_DDRB1   1
#define T841_DDRB0   0

#define T841_PORTB   0x20+0x18
#define T841_PORTB3  3
#define T841_PORTB2  2
#define T841_PORTB1  1
#define T841_PORTB0  0

#define T841_PINA    0x20+0x19
#define T841_PINA7   7
#define T841_PINA6   6
#define T841_PINA5   5
#define T841_PINA4   4
#define T841_PINA3   3
#define T841_PINA2   2
#define T841_PINA1   1
#define T841_PINA0   0

#define T841_DDRA    0x20+0x1A
#define T841_DDRA7   7
#define T841_DDRA6   6
#define T841_DDRA5   5
#define T841_DDRA4   4
#define T841_DDRA3   3
#define T841_DDRA2   2
#define T841_DDRA1   1
#define T841_DDRA0   0

#define T841_PORTA   0x20+0x1B
#define T841_PORTA7  7
#define T841_PORTA6  6
#define T841_PORTA5  5
#define T841_PORTA4  4
#define T841_PORTA3  3
#define T841_PORTA2  2
#define T841_PORTA1  1
#define T841_PORTA0  0

#define T841_EECR    0x20+0x1C
#define T841_EERE    0
#define T841_EEPE    1
#define T841_EEMPE   2
#define T841_EERIE   3
#define T841_EEPM0   4
#define T841_EEPM1   5

#define T841_EEDR    0x20+0x1D

#define T841_EEARL   0x20+0x1E
#define T841_EEARH   0x20+0x1F

#define T841_PCMSK1  0x20+0x20

#define T841_WDTCSR  0x20+0x21
#define T841_WDE     3
#define T841_WDP0    0
#define T841_WDP1    1
#define T841_WDP2    2
#define T841_WDP3    5
#define T841_WDIE    6
#define T841_WDIF    7

#define T841_TCCR1C  0x20+0x22
#define T841_FOC1B   6
#define T841_FOC1A   7

#define T841_GTCCR   0x20+0x23
#define T841_PSR     0
#define T841_TSM     7

#define T841_ICR1L   0x20+0x24
#define T841_ICR1H   0x20+0x25

// Reserved [0x26..0x27]

#define T841_OCR1BL  0x20+0x28
#define T841_OCR1BH  0x20+0x29

#define T841_OCR1AL  0x20+0x2A
#define T841_OCR1AH  0x20+0x2B

#define T841_TCNT1L  0x20+0x2C
#define T841_TCNT1H  0x20+0x2D

#define T841_TCCR1B  0x20+0x2E
#define T841_CS10    0
#define T841_CS11    1
#define T841_CS12    2
#define T841_WGM12   3
#define T841_WGM13   4
#define T841_ICES1   6
#define T841_ICNC1   7

#define T841_TCCR1A  0x20+0x2F
#define T841_WGM10   0
#define T841_WGM11   1
#define T841_COM1B0  4
#define T841_COM1B1  5
#define T841_COM1A0  6
#define T841_COM1A1  7

#define T841_TCCR0A  0x20+0x30
#define T841_WGM00   0
#define T841_WGM01   1
#define T841_COM0B0  4
#define T841_COM0B1  5
#define T841_COM0A0  6
#define T841_COM0A1  7

// Reserved [0x31]

#define T841_TCNT0   0x20+0x32

#define T841_TCCR0B  0x20+0x33
#define T841_CS00    0
#define T841_CS01    1
#define T841_CS02    2
#define T841_WGM02   3
#define T841_FOC0B   6
#define T841_FOC0A   7

#define T841_MCUSR   0x20+0x34
#define T841_PORF    0
#define T841_EXTRF   1
#define T841_BORF    2
#define T841_WDRF    3

#define T841_MCUCR   0x20+0x35
#define T841_ISC00   0
#define T841_ISC01   1
#define T841_SM0     3
#define T841_SM1     4
#define T841_SE      5

#define T841_OCR0A   0x20+0x36

#define T841_SPMCSR  0x20+0x37
#define T841_SPMEN   0
#define T841_PGERS   1
#define T841_PGWRT   2
#define T841_RFLB    3
#define T841_CTPB    4
#define T841_RSIG    5

#define T841_TIFR0   0x20+0x38
#define T841_TOV0    0
#define T841_OCF0A   1
#define T841_OCF0B   2

#define T841_TIMSK0  0x20+0x39
#define T841_TOIE0   0
#define T841_OCIE0A  1
#define T841_OCIE0B  2

#define T841_GIFR    0x20+0x3A
#define T841_PCIF0   4
#define T841_PCIF1   5
#define T841_INTF0   6

#define T841_GIMSK   0x20+0x3B
#define T841_PCIE0   4
#define T841_PCIE1   5
#define T841_INT0    6

#define T841_OCR0B   0x20+0x3C

// SP [0x3D..0x3E]

// SREG [0x3F]

#define T841_DIDR0   0x60
#define T841_ADC0D   0
#define T841_ADC1D   1
#define T841_ADC2D   2
#define T841_ADC3D   3
#define T841_ADC4D   4
#define T841_ADC5D   5
#define T841_ADC6D   6
#define T841_ADC7D   7

#define T841_DIDR1   0x61
#define T841_ADC11D  0
#define T841_ADC10D  1
#define T841_ADC8D   2
#define T841_ADC9D   3

#define T841_PUEB    0x62

#define T841_PUEA    0x63

#define T841_PORTCR  0x64
#define T841_BBMB    1
#define T841_BBMA    0

#define T841_REMAP   0x65
#define T841_U0MAP   0
#define T841_SPIMAP  1

#define T841_TOCPMCOE 0x66
#define T841_TOCC0OE 0
#define T841_TOCC1OE 1
#define T841_TOCC2OE 2
#define T841_TOCC3OE 3
#define T841_TOCC4OE 4
#define T841_TOCC5OE 5
#define T841_TOCC6OE 6
#define T841_TOCC7OE 7

#define T841_TOCPMSA0 0x67
#define T841_TOCC0S0 0
#define T841_TOCC0S1 1
#define T841_TOCC1S0 2
#define T841_TOCC1S1 3
#define T841_TOCC2S0 4
#define T841_TOCC2S1 5
#define T841_TOCC3S0 6
#define T841_TOCC3S1 7

#define T841_TOCPMSA1 0x68
#define T841_TOCC4S0 0
#define T841_TOCC4S1 1
#define T841_TOCC5S0 2
#define T841_TOCC5S1 3
#define T841_TOCC6S0 4
#define T841_TOCC6S1 5
#define T841_TOCC7S0 6
#define T841_TOCC7S1 7

// Reserved [0x69]

#define T841_PHDE    0x6A
#define T841_PHDEA0  0
#define T841_PHDEA1  1

// Reserved [0x6B..0x6F]

#define T841_PRR     0x70
#define T841_PRADC   0
#define T841_PRTIM0  1
#define T841_PRTIM1  2
#define T841_PRTIM2  3
#define T841_PRSPI   4
#define T841_PRUSART0 5
#define T841_PRUSART1 6
#define T841_PRTWI   7

#define T841_CCP     0x71

#define T841_CLKCR   0x72
#define T841_CKSEL0  0
#define T841_CKSEL1  1
#define T841_CKSEL2  2
#define T841_CKSEL3  3
#define T841_SUT     4
#define T841_CKOUTC  5
#define T841_CSTR    6
#define T841_OSCRDY  7

#define T841_CLKPR   0x73
#define T841_CLKPS0  0
#define T841_CLKPS1  1
#define T841_CLKPS2  2
#define T841_CLKPS3  3

#define T841_OSCCAL0 0x74

#define T841_OSCTCAL0A 0x75

#define T841_OSCTCAL0B 0x76

#define T841_OSCCAL1 0x77

// Reserved [0x78..0x7F]

#define T841_UDR0    0x80

#define T841_UBRR0L  0x81
#define T841_UBRR0H  0x82

#define T841_UCSR0D  0x83
#define T841_SFDE0   5
#define T841_RXS0    6
#define T841_RXSIE0  7

#define T841_UCSR0C  0x84
#define T841_UCPOL0  0
#define T841_UCSZ00  1
#define T841_UCSZ01  2
#define T841_USBS0   3
#define T841_UPM00   4
#define T841_UPM01   5
#define T841_UMSEL00 6
#define T841_UMSEL01 7

#define T841_UCSR0B  0x85
#define T841_TXB80   0
#define T841_RXB80   1
#define T841_UCSZ02  2
#define T841_TXEN0   3
#define T841_RXEN0   4
#define T841_UDRIE0  5
#define T841_TXCIE0  6
#define T841_RXCIE0  7

#define T841_UCSR0A  0x86
#define T841_MPCM0   0
#define T841_U2X0    1
#define T841_UPE0    2
#define T841_DOR0    3
#define T841_FE0     4
#define T841_UDRE0   5
#define T841_TXC0    6
#define T841_RXC0    7

// Reserved [0x87..0x8F]

#define T841_UDR1    0x90

#define T841_UBRR1L  0x91
#define T841_UBRR1H  0x92

#define T841_UCSR1D  0x93
#define T841_SFDE1   5
#define T841_RXS1    6
#define T841_RXSIE1  7

#define T841_UCSR1C  0x94
#define T841_UCPOL1  0
#define T841_UCSZ10  1
#define T841_UCSZ11  2
#define T841_USBS1   3
#define T841_UPM10   4
#define T841_UPM11   5
#define T841_UMSEL10 6
#define T841_UMSEL11 7

#define T841_UCSR1B  0x95
#define T841_TXB81   0
#define T841_RXB81   1
#define T841_UCSZ12  2
#define T841_TXEN1   3
#define T841_RXEN1   4
#define T841_UDRIE1  5
#define T841_TXCIE1  6
#define T841_RXCIE1  7

#define T841_UCSR1A  0x96
#define T841_MPCM1   0
#define T841_U2X1    1
#define T841_UPE1    2
#define T841_DOR1    3
#define T841_FE1     4
#define T841_UDRE1   5
#define T841_TXC1    6
#define T841_RXC1    7

// Reserved [0x97..0x9F]

#define T841_TWSD    0xA0
#define T841_TWSD0   0
#define T841_TWSD1   1
#define T841_TWSD2   2
#define T841_TWSD3   3
#define T841_TWSD4   4
#define T841_TWSD5   5
#define T841_TWSD6   6
#define T841_TWSD7   7

#define T841_TWSAM   0xA1
#define T841_TWAE    0
#define T841_TWSAM1  1
#define T841_TWSAM2  2
#define T841_TWSAM3  3
#define T841_TWSAM4  4
#define T841_TWSAM5  5
#define T841_TWSAM6  6
#define T841_TWSAM7  7

#define T841_TWSA    0xA2

#define T841_TWSSRA  0xA3
#define T841_TWAS    0
#define T841_TWDIR   1
#define T841_TWBE    2
#define T841_TWC     3
#define T841_TWRA    4
#define T841_TWCH    5
#define T841_TWASIF  6
#define T841_TWDIF   7

#define T841_TWSCRB  0xA4
#define T841_TWCMD0  0
#define T841_TWCMD1  1
#define T841_TWAA    2
#define T841_TWHNM   3

#define T841_TWSCRA  0xA5
#define T841_TWSME   0
#define T841_TWPME   1
#define T841_TWSIE   2
#define T841_TWEN    3
#define T841_TWASIE  4
#define T841_TWDIE   5
#define T841_TWSHE   7

// Reserved [0xA6..0xAF]

#define T841_SPDR    0xB0

#define T841_SPSR    0xB1
#define T841_SPI2X   0
#define T841_WCOL    6
#define T841_SPIF    7

#define T841_SPCR    0xB2
#define T841_SPR0    0
#define T841_SPR1    1
#define T841_CPHA    2
#define T841_CPOL    3
#define T841_MSTR    4
#define T841_DORD    5
#define T841_SPE     6
#define T841_SPIE    7

// Reserved [0xB3..0xBF]

#define T841_ICR2L   0xC0
#define T841_ICR2H   0xC1

#define T841_OCR2BL  0xC2
#define T841_OCR2BH  0xC3

#define T841_OCR2AL  0xC4
#define T841_OCR2AH  0xC5

#define T841_TCNT2L  0xC6
#define T841_TCNT2H  0xC7

#define T841_TCCR2C  0xC8
#define T841_FOC2B   6
#define T841_FOC2A   7

#define T841_TCCR2B  0xC9
#define T841_CS20    0
#define T841_CS21    1
#define T841_CS22    2
#define T841_WGM22   3
#define T841_WGM23   4
#define T841_ICES2   6
#define T841_ICNC2   7

#define T841_TCCR2A  0xCA
#define T841_WGM20   0
#define T841_WGM21   1
#define T841_COM2B0  4
#define T841_COM2B1  5
#define T841_COM2A0  6
#define T841_COM2A1  7

