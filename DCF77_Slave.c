//
//  DCF77.c
//  DCF77
//
//  Created by Sysadmin on 05.10.07.
//  Copyright Ruedi Heimlicher 2007. All rights reserved.
//


#include <stdio.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
#include <util/twi.h>
#include <string.h>
#include <avr/eeprom.h>

#include <avr/wdt.h>

# include "DCF77.h"
#include "twislave.c"
#include "lcd.c"

#define delay_us(us)  _delay_loop_2 (((F_CPU/4000)*us)/1000)		// wartet µs
/*
	Aufbau 7-Segment
          a
         ---
       f| g |b
         ---
       e|   |c
         ---
          d
*/

//	Welcher Port an welchem Segement:
#define segm_a PD0
#define segm_b PD1
#define segm_c PD2
#define segm_d PD3
#define segm_e PD4
#define segm_f PD5
#define segm_g PD6

// Welcher Port an welcher Digit:
#define digit0 PB2		// Wochentag
#define digit1 PB3		// Einer
#define digit2 PB4		// Zehner
#define digit3 PB5		// Hunderter
#define digit4 PB6		// Tausender
#define digit5 PB7		// Wochentag

// An welchem Port sind Taster
//#define mode		PC2

// Eingang Uhr
//#define takt		PC0

// Ausgang Sekunde
//#define sekundentakt		PC1

// Ports definieren
#define port_segment	PORTD
#define dir_segment		DDRD

#define port_digit		PORTB
#define dir_digit		DDRB

#define dir_keys		DDRC
#define pin_keys		PINC

#define mode_select		PC1	//Taste an PIN 1 von PORTC
#define display_select	PB1

#define dimm_up			PB1	//Taste an PIN 1 von PORTB
#define dimm_down		PB2	//Taste an PIN 2 von PORTB

// Definiert, welche Segmente für die einzelnen Zahlenwerte aufleuchten sollen. HIGH-Aktiv
//#define sign0   (1 << segm_c) | (1 << segm_d) | (1 << segm_e) | (1 << segm_f)
#define sign0 (1 << segm_a) | (1 << segm_b) | (1 << segm_c) | (1 << segm_d) | (1 << segm_e) | (1 << segm_f)
#define sign1 (1 << segm_b) | (1 << segm_c)
#define sign2 (1 << segm_a) | (1 << segm_b) | (1 << segm_g) | (1 << segm_e) | (1 << segm_d)
#define sign3 (1 << segm_a) | (1 << segm_b) | (1 << segm_g) | (1 << segm_c) | (1 << segm_d)
#define sign4 (1 << segm_f) | (1 << segm_g) | (1 << segm_b) | (1 << segm_c)
#define sign5 (1 << segm_a) | (1 << segm_f) | (1 << segm_g) | (1 << segm_c) | (1 << segm_d)
#define sign6 (1 << segm_a) | (1 << segm_f) | (1 << segm_g) | (1 << segm_e) | (1 << segm_d) | (1 << segm_c)
#define sign7 (1 << segm_a) | (1 << segm_b) | (1 << segm_c)
#define sign8 (1 << segm_a) | (1 << segm_b) | (1 << segm_c) | (1 << segm_d) | (1 << segm_e) | (1 << segm_f) | (1 << segm_g)
#define sign9 (1 << segm_a) | (1 << segm_b) | (1 << segm_c) | (1 << segm_d) | (1 << segm_g) | (1 << segm_f)

#define start_Minute			21
#define end_Minute				27
#define min_Par					28
#define start_Stunde			29
#define end_Stunde				34
#define stunde_Par				35
#define start_Kalendertag		36
#define end_Kalendertag			41
#define start_Wochentag			42
#define end_Wochentag			44
#define start_Monat				45
#define end_Monat				49
#define start_Jahr				50
#define end_Jahr				57
#define kalender_Par			58
#define sync					59


#define null	(1<<PD0)
#define ein		(1<<PD1)
#define zwei	(1<<PD2)
#define drei	(1<<PD3)
#define vier	(1<<PD4)
#define fuenf	(1<<PD5)
#define sechs	(1<<PD6)
#define sieben	(1<<PD7)

/********************************************/
//DCF77										*
#define SLAVE_ADRESSE 0x54				//	*
//											*
/********************************************/



#define SDAPIN			4
#define SCLPIN			5
#define WDTBIT			7

#define LOOPLEDPORT		PORTC
#define LOOPLED			3
#define TWILED			5

#define STARTDELAYBIT	0
#define HICOUNTBIT		1

#define TAKTFAKTOR   1 // Anzahl MHz der Taktfrequenz

struct time Zeit;
uint8_t wtag[]=		{ null, ein, zwei, drei, vier, fuenf, sechs, sieben};
uint8_t num[] =		{sign0, sign1, sign2, sign3, sign4, sign5, sign6, sign7, sign8, sign9 };
uint8_t digit[] =	{ digit0, digit1, digit2, digit3, digit4, digit5};
volatile uint16_t 	display_value=0;		// Startwert. 

//Datumwerte	
volatile uint8_t  	sekunden=0;
volatile uint8_t  	minuten=0;
uint8_t				minutenpar=0;
uint8_t				anzeigeminuten=0;
volatile uint8_t  	stunden=0;
uint8_t				anzeigestunden=0;
uint8_t				stundenpar=0;
volatile uint8_t  	kalendertag=0;
volatile uint8_t  	kalendermonat=0;
volatile uint8_t  	kalenderjahr=0;
volatile uint8_t  	wochentag=0;
uint8_t				kalenderpar=0;
//Position der Datumangabe
volatile uint8_t  	bitzeiger=0; //aktuelles Bit innerhalb der Datenübertragung
uint8_t				ImpulsdauerL=0;
uint8_t				ImpulsdauerH=0;
uint8_t				TastendauerL=0;
uint8_t				TastendauerH=0;
uint8_t				Displaymode=0; //Zeit anzeigen
volatile uint16_t  	PausenOverflowCounter=0;
uint8_t				dcf77OK=1;
uint8_t				segDelay=10; //Delay für Segmentanzeige
uint16_t			segDimm=0;
uint16_t			Anzeigewert=0;
uint16_t			zaehler=0;
uint16_t			dauer=0;
volatile uint8_t  	digitnummer=0;
volatile uint8_t  	maxdigit=5;
volatile uint8_t  	digitdelaycountL=0;
volatile uint8_t  	digitdelaycountH=0;
uint16_t             twicount=0;

volatile uint8_t           timer0div=0;
volatile uint8_t           timer2div=0;

extern volatile uint8_t	DCFImpuls_wait;

extern volatile uint8_t	TWIACK_wait;

extern volatile uint8_t DCF_busy;

volatile uint8_t status=0;

uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset


void delay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void init()
{
DDRD = 0xFF;	// PORTD als Ausgang setzen fuer Segmente

DDRB = 0xFF;	// PORTB als Ausgang setzen fuer Digits

//PORTD |= (1<<PORTD2)
//DDRD &= ~(1<<DDD2); // Alles belassen nur PORTD2 als Eingang setzen
//PORTD |= (1<<PORTD2); // Den zugehörigen internen Pull-Up Widerstand aktivieren


//Board DCF77: 
DDRB &= ~(1<<DDB0);		//	Pin 0 von PORT B als Eingang fuer DCF
PORTB |= (1<<PORTB0);	// Den zugehörigen internen Pull-Up Widerstand aktivieren

DDRC |= (1<<DDC0);		//	Pin 0 von PORT C als Ausgang fuer Sekunden-LED

DDRC &= ~(1<<DDC1);		//	Pin 1 von PORT C als Eingang fuer Modeselect
PORTC |= (1<<PORTC1);	// Den zugehörigen internen Pull-Up Widerstand aktivieren

DDRC &= ~(1<<DDC4);		//	Pin 4 von PORT C als Eingang fuer SCL
PORTC |= (1<<PORTC4);	//	Pull-Up Widerstand aktivieren
DDRC &= ~(1<<DDC5);		//	Pin 5 von PORT C als Eingang fuer SDA
PORTC |= (1<<PORTC5);	//	Pull-Up Widerstand aktivieren

DDRB &= ~(1<<DDB1);		//	Pin 1 von PORT B als Eingang fuer Dimm up
PORTB |= (1<<PORTB1);	//	Den zugehörigen internen Pull-Up Widerstand aktivieren

DDRB &= ~(1<<DDB2);		//	Pin 2 von PORT B als Eingang fuer Dimm down
PORTB |= (1<<PORTB2);	//	Den zugehörigen internen Pull-Up Widerstand aktivieren

//***********************************
// Belegung Prov fuer myAVR2
DDRC |= (1<<DDC3);	//Pin 3 von PORT C als Ausgang fuer TWI-LED

//
//***********************************



//***********************************
//									*
//  TWI-Resistors sind extern		*
//									*
//***********************************
}

void ZifferZeigen(uint8_t wert, uint8_t pos)
{
PORTB &= ~((1<<PB2) |(1<<PB3) | (1<<PB4) | (1<<PB5) | (1<<PB6)| (1<<PB7));

PORTB |= (1<<digit[pos+1]);//Digit von pos einschalten

if (pos==maxdigit-1)
{
PORTD=wtag[wert];
}
else
{
PORTD=num[wert];

}

}


void main (void) 
{
	init();
	fehler=0;
	wdt_enable (WDTO_1S);
	uint8_t TastenStatus=0x00;
	uint8_t DimmUpTastenStatus=0x00;
	uint8_t DimmUpTastendauerL=0x00;
	uint8_t DimmUpTastendauerH=0x00;
	uint8_t DimmDownTastenStatus=0x00;
	uint8_t DimmDownTastendauerL=0x00;
	uint8_t DimmDownTastendauerH=0x00;
	uint8_t prellOK=3;
	volatile uint8_t DCFImpuls=0;
	int datenOK=0;
	int eins=0;
	delay_ms(400);
	init_twi_slave (SLAVE_ADRESSE);
	Displaymode=0;
	
//	uint8_t tempWDT_Count=eeprom_read_byte(&WDT_ErrCount);

	uint16_t loopcount0=0;
	//	Zaehler fuer Wartezeit nach dem Start
	uint16_t startdelay0=0x00AF;
	//uint16_t startdelay1=0;

	//Zaehler fuer Zeit von (SDA || SCL = LO)
	uint16_t twi_LO_count0=0;
	uint16_t twi_LO_count1=0;
	uint8_t SlaveStatus=0x00; //status

	//Zaehler fuer Zeit von (SDA && SCL = HI)
	uint16_t twi_HI_count0=0;

	uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0); 
	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);

//	PORTB = 0xFF;
//	delay_ms(200);
//	PORTB = 0x00;
//	delay_ms(400);
	sei();
	
	// Ankuendigen, dass schon ein wdt erfolgte
	if (!(eepromWDT_Count0==eepromWDT_Count1))
	{
		lcd_gotoxy(18,1);
		lcd_puts("W\0");

	}



	while(1)
	{
		wdt_reset();
		loopcount0++;
		if (loopcount0==0x0FFF)
		{
			loopcount0=0;
			LOOPLEDPORT ^=(1<<LOOPLED);
			//lcd_gotoxy(0,0);
			//lcd_puts("wdt\0");
			//lcd_puthex(tempWDT_Count);
			//delay_ms(10);
         
		}



		
		/**	Beginn Startroutinen	***********************/
		/*
		// Startfunktion: SCL und SDA pruefen, ob lang genug beide HI sind
		if (PINC & (1<<4) && PINC & (1<<5)) // SCL UND SDA ist HI
		{
			if (startdelay0) // Zaehler ist noch nicht auf 0
			{
				if (startdelay0 == 1) // Startroutinen laufen lassen
				{
					SlaveStatus |= (1<<STARTDELAYBIT);
					{
					}
				}
				startdelay0--;
				
			}
			
			// nach Ablauf des Startdelay Dauer von HI messen. HI_count ist max FF

			if (startdelay0==0) 			
			{
				if (twi_HI_count0 <0xFF && startdelay0==0)
				{
					twi_HI_count0++; // Dauer von HI auf SDA und SCL messen
//					lcd_gotoxy(17,1);
//					lcd_puts("-\0");
					//lcd_gotoxy(18,0);
					//lcd_puts("HIcount OK\0");
					//lcd_puthex(twi_HI_count0);

				}
				else // HI_count ist FF, Bit in SlaveStatus setzen
				{
					if ((SlaveStatus & (1<<HICOUNTBIT)))
					{
						//lcd_gotoxy(17,1);
						//lcd_puts("h\0");
					}
					else
					{
//					lcd_gotoxy(17,1);
//					lcd_puts("B\0");
					//lcd_puthex(twi_HI_count0);
					SlaveStatus |= (1<<HICOUNTBIT); // HICount ist gross genug
					}
				}
			}// if startdelay
		}
		
		// TWI-Start detektieren
		
		// Bit 7 in eepromWDT_Count ist noch gesetzt nach wdt: warten auf Start des Master
		
		if ((eepromWDT_Count0 & (1 << WDTBIT)) && (!(PINC & (1<<SDAPIN))) && PINC & (1<<SCLPIN)) // TWI-Start vonm Master
		{
			
			
			// Startdelay abgelaufen, HI_Count IST GESETZT und (eepromWDT_Count1 > eepromWDT_Count0), also pendenter wdt-reset
			if ((SlaveStatus & (1<<STARTDELAYBIT)) && (SlaveStatus & (1<<HICOUNTBIT)))
				
//			if ((SlaveStatus & (1<<STARTDELAYBIT)) && (SlaveStatus & (1<<HICOUNTBIT)) && !(eepromWDT_Count1 == eepromWDT_Count0))
			{
				//lcd_gotoxy(15,1);
				//lcd_puts("S\0");
				
				//Bit 7 reseten
				eepromWDT_Count0 &= ~(1<<WDTBIT);
				//lcd_gotoxy(0,0);
				//lcd_puthex(eepromWDT_Count0);
				
				// Zaehler erhoehen
				eepromWDT_Count0++;
				
				// eepromWDT_Count0  aktualisieren
				eeprom_write_byte(&WDT_ErrCount0,eepromWDT_Count0); 
				
				// HICOUNTBIT zuruecksetzen
				SlaveStatus &= ~(1<<HICOUNTBIT); 
				
				
			}
		}// End Start detektieren


		
		// wenn Startdelay abgelaufen und wdt_count0 gleich wdt_count1: TWI_slave initiieren
		if ((SlaveStatus & (1<<STARTDELAYBIT)) && (!(eepromWDT_Count0 & (1<<WDTBIT))))// kein pendenter wdt-Reset
			{
				//lcd_clr_line(1);
//				lcd_gotoxy(19,1);
//				lcd_puts("I\0");
				
				// wait-Anzeige loeschen
				lcd_gotoxy(18,1);
				lcd_puts(" \0");
				
				//lcd_puthex(SlaveStatus);
				//delay_ms(1000);
				init_twi_slave (SLAVE_ADRESSE);
				sei();
				
				// StartDelayBit zuruecksetzen
				SlaveStatus &= ~(1<<STARTDELAYBIT); 
								
			}
			else
			{
				if (SlaveStatus & (1<<STARTDELAYBIT))
				{
		//			lcd_gotoxy(18,1);
		//			lcd_puts("W\0");
				}
				
				
			}
						
							
		*/										
								
		// TWI-PINs ueberwachen
		/************ SCL SDA checken *************/
      /*
		if ((!(PINC & (1<<SCLPIN))) && PINC & (1<<SDAPIN))		// SDA ist HI und SCL ist LO (warten auf Ack)
	//	if ((!(PINC & (1<<4)) || !(PINC & (1<<5)))) 	// SCL oder SDA ist low
		{
			// HICOUNTBIT zuruecksetzen
//			SlaveStatus &= ~(1<<HICOUNTBIT); 
			
			// Low-Zeit messen
			twi_LO_count0++;
			if (twi_LO_count0 == 0x0BFF)
			{
			// HICOUNTBIT zuruecksetzen
				SlaveStatus &= ~(1<<HICOUNTBIT); 
				//PORTD |= (1<<TWILED); //TWI-LED ON
				
				twi_LO_count0=0;
				twi_LO_count1++;
				if (twi_LO_count1>=0x0003) //	
				{
					//PORTD |= (1<<TWILED); //TWI-LED ON
					//twierrcount++;
					// TWI neu starten wenn schon mal gestartet
					if (twi_LO_count1 == 0x0006 && startdelay0 == 0) 
					{
						//twi_LO_count1=0;
						//PORTD &= ~(1<<TWILED); // TWI-LED OFF
						//lcd_clr_line(1);
						lcd_gotoxy(12,0);
						lcd_puts("neu\0");
						
						TWBR =0;
						init_twi_slave (SLAVE_ADRESSE);
						sei();
						lcd_gotoxy(12,0);
						lcd_puts("   \0");
						
					}
					//lcd_gotoxy(12,0);
					//lcd_puts("ct \0");
					//lcd_puthex(twi_LO_count1);

					//delay_ms(100);
					if (twi_LO_count1== 0x000A && startdelay0 == 0)	// Zeitverzoegerung
					{
						
						if (!(eepromWDT_Count0 & (1<<WDTBIT)))// reset wenn noch kein wdt-reset ausgelöst wurde
						{
							//twi_LO_count1=0;
							
							//eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1); //letzter gespeicherter Wert
							//eepromWDT_Count1++;
							
							eepromWDT_Count0 &= ~(1<<WDTBIT);
							//lcd_clr_line(1);
							lcd_gotoxy(12,0);
							lcd_puts("wdt \0");
							lcd_puthex(eepromWDT_Count0);
							delay_ms(1000);
							eeprom_write_byte(&WDT_ErrCount0,eepromWDT_Count0);
							//slavestatus |= (1<<0); // wdt-bit setzen
							cli();
							wdt_enable (WDTO_15MS);
							{while(1);}
						}
					}
				}
			}
		}
		else
		{
			//PORTD &= ~(1<<TWILED); //TWI-LED OFF
			//lcd_gotoxy(0,1);
			//lcd_puts("   \0"); // neu löschen
			//lcd_puthex(twierrcount);
			twi_LO_count0=0;
			twi_LO_count1=0;
			//lcd_gotoxy(13,1);
			//lcd_puthex(twi_LO_count1);

		}
		*/
		/**	Ende Startroutinen	***********************/
		
		
	
		//PORTB &= ~(1<<PORTB1);
//		uint8_t lr=strlen(rxbuffer);
		if (TWCR & ~(1<<TWINT))
		{
			//PORTB |=(1<<PB0);
//			TWCR |= (1<<TWINT);
		}
		
		
		if (PINC & (1<<PINC0)) // SekundenLED ist ON
		{
			ImpulsdauerL++;
			if (ImpulsdauerL==0)//255 ticks
			{
				ImpulsdauerH++;
			}	
			if (ImpulsdauerH == 2)
			{
				//SekundenLED Loeschen
				PORTC &= ~(1<<PORTC0);
				ImpulsdauerH=0;
				ImpulsdauerL=0;
			}
		}

//		Start Taste ModeSelect		
		
		if (!(PINC & (1<<PINC1)))					// Taste ist LOW (gedrueckt)
		{
			if (!(TastenStatus & (1<<mode_select)))		// Taste war noch nicht gedrueckt
			{
				TastenStatus |= (1<<mode_select);
				TastendauerH=0;							// Zaehlung beginnen
				TastendauerL=0;
			}
			else
			{
				TastendauerL++;							// LowBit incr
				if (TastendauerL==0)					// 255 ticks, HighBit incr
				{
					TastendauerH++;
				}	
				
				
				
				if (TastendauerH > prellOK) //Entprellung OK
				{
					//19.8.08
					if ((DimmDownTastendauerH > prellOK)&& (segDimm==0)) //SegDowntaste ist gedrückt und segDimm auf 0 
					{
						// TWI toggle
						if (TWCR &  (1<<TWEN)) // TWI ist enabled
						{
							TWCR &= ~(1<<TWEN); //TWI ausschalten
							TWCR |= (1<<TWINT); // TWINT clear (set to 1)
						}
						else
						{
							init_twi_slave (SLAVE_ADRESSE);	// TWI einschalten
						}
					}
					else
						
					{
						if (Displaymode<=3)	//	weiterschalten
						{
							Displaymode++;
						}
						else
						{
							Displaymode=0;	//neue Runde
						}
						
						
					}
					TastendauerH=0;
					TastendauerL=0;
					TastenStatus &= ~(1<<mode_select);
					
				} // if TastendauerH > prellOK
			}
			
		}
		else					
		{
			if ((TastenStatus & (1<<mode_select)))		// Taste war  gedrueckt
			{
				TastenStatus &= ~(1<<mode_select);		// Status reset
			}
		}
//		Ende Taste ModeSelect
		
//		Start Taste Dimm up
		if (!(PINB & (1<<PINB1)))					// Taste ist LOW (gedrückt),
		{
			if (!(DimmUpTastenStatus & (1<<dimm_up)))		// Taste war noch nicht gedrueckt
			{
				DimmUpTastenStatus |= (1<<dimm_up);
				DimmUpTastendauerH=0;							// Zaehlung beginnen
				DimmUpTastendauerL=0;
			}
			else
			{
				DimmUpTastendauerL++;							// LowBit incr
				if (DimmUpTastendauerL==0)					// 255 ticks, HighBit incr
				{
					DimmUpTastendauerH++;
				}	
				if (DimmUpTastendauerH > prellOK) //Entprellung OK
					
				{
					if (segDimm< segDelay)//	weiterschalten
					{
						segDimm++;	//Heller
					}
					DimmUpTastendauerH=0;
					DimmUpTastendauerL=0;
					DimmUpTastenStatus &= ~(1<<dimm_up);
				}
								
			}
			
		}
		else					//
		{
			if ((DimmUpTastenStatus & (1<<dimm_up)))		// Taste war  gedrueckt
			{
				DimmUpTastenStatus &= ~(1<<dimm_up);		// Status reset
			}
		}
		


//		Ende Taste Dimm up


//		Start Taste Dimm down
		if (!(PINB & (1<<PINB2)))					// Taste ist LOW (Gedrückt),
		{
			if (!(DimmDownTastenStatus & (1<<dimm_down)))		// Taste war noch nicht gedrueckt
			{
				DimmDownTastenStatus |= (1<<dimm_down);
				DimmDownTastendauerH=0;							// Zaehlung beginnen
				DimmDownTastendauerL=0;
			}
			else
			{
				DimmDownTastendauerL++;							// LowBit incr
				if (DimmDownTastendauerL==0)					// 255 ticks, HighBit incr
				{
					DimmDownTastendauerH++;
				}	
				
				if (DimmDownTastendauerH > prellOK) //Entprellung OK
				{
					if (segDimm)//	weiter zurückschalten
					{
						segDimm--;	//Dunkler
					}
					
					if (segDimm==0) // Disply aus, eventuell TWI ausschalten
					{
					
					
					}

					
					DimmDownTastendauerH=0;
					DimmDownTastendauerL=0;
					DimmDownTastenStatus &= ~(1<<dimm_down);
				
				
				}
								
			}
			
		}
		else					//
		{
			if ((DimmDownTastenStatus & (1<<dimm_down)))		// Taste war  gedrueckt
			{
				DimmDownTastenStatus &= ~(1<<dimm_down);		// Status reset
			}
		}
		


//		Ende Taste Dimm down

		
//		Displaymode=0;
      
#pragma mark timer 0
		
		if (PINB & (1<<PINB0)) // DCF Signal H,
		{
			if (DCFImpuls)
			{
				//Impuls läuft schon
			}
			else
			{
            
				//Impuls hat gerade angefangen, Zaehler starten
				PORTC |= (1<<PORTC0);			//Takt-LED ein
												//TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
				TCCR1B |= (1<<CS10)|(1<<CS12);	//Takt /1024
				TCNT1=0x00;
				DCFImpuls=1;
				DCFImpuls_wait=1; //fuer ISR() in twislave
				sekunden++;
				zaehler++;	
				//Pause beendet, Timer2 stoppen
            
            timer0div &= ~((1<<0)|(1<<1));
            
				
            TCCR2 &= ~((1<<CS20)|(1<<CS21)|(1<<CS22));//Timer 2 stoppen
	//			DCF_busy=1;	// Impuls vorhanden, kein TWI
	//			cli();
				
			}//if DCFImpuls
		}
		else
		{
			//kein DCF-Impuls
			//	PORTC &= ~(1<<PORTC1);//Sicherheitshalber Sekundenled löschen
			if (DCFImpuls) //Impuls ist fertig
			{
				//dauer=(uint16_t)TCNT1;
				int par=0;
				
				if (TCNT1<100*TAKTFAKTOR)//Kurzer Impuls
				{
               
					eins=0;
					//PORTC &= ~(1<<PORTC4);	//Overflow Impuls aus
//					PORTC &= ~(1<<PORTC3);	//langer Impuls aus
//					PORTC |= (1<<PORTC2);	//kurzer Impuls ein
				}
				else if (TCNT1<180*TAKTFAKTOR)	//langer Impuls
				{
               
					eins=1;
					//PORTC &= ~(1<<PORTC4);	//Overflow Impuls aus
//					PORTC &= ~(1<<PORTC2);	//kurzer Impuls aus
//					PORTC |= (1<<PORTC3);	//Langer Impuls ein
				}
				else if (TIFR |=(1<<TOV1))//Overflow Flag gesetzt
				{
					eins=4;
					datenOK=1;
					//PORTC &= ~(1<<PORTC2);//kurzer Impuls aus
					//PORTC &= ~(1<<PORTC3);//langer Impuls aus
					TIFR |=(1<<TOV1);//Interrupt Flag loeschen
				}
				//TCCR1 &= ~(1<<CS00)|(1<<CS02);
				
				if ((bitzeiger >= start_Minute)&&(bitzeiger<= end_Minute))	//Minutenbits
				{
					if (eins==1)
					{
						minuten+=minuteBits[bitzeiger - start_Minute];
						minutenpar++;
					}
				}
				
				
				if  (bitzeiger==28)					//par Stunden
				{
					par=minutenpar % 2; //Parität
					if (par==eins) //Parität stimmt
					{
						anzeigeminuten=minuten;
						Zeit.minute=minuten;
                  
					}
					else
					{
						dcf77OK=0;
						if (minuten<60)
						{
							minuten++;
						}
						else
						{
							minuten=0;
							if (stunden<24)
							{
								stunden++;
							}
							else
							{
								stunden=0;
							}
						}
						//anzeigeminuten=99;
					}
				}
				
				if ((bitzeiger>= start_Stunde)&&(bitzeiger <= end_Stunde))	//Stundenbits
				{
					if (eins==1)
					{
						stunden+=stundeBits[bitzeiger - start_Stunde];
						stundenpar++;
					}
				}
				
				if  (bitzeiger == stunde_Par)					//par Stunden
				{
					par=stundenpar %2; //Parität
					if (par==eins) //Parität stimmt
					{
						anzeigestunden=stunden;
						Zeit.stunde=stunden;
					}
					else
					{
						dcf77OK=0;
						//anzeigestunden=99;
					}
				}
				
				if ((bitzeiger>=start_Kalendertag)&&(bitzeiger<=end_Kalendertag))	//Kalendertagsbits
				{
					if (eins==1)
					{
						kalendertag+=dcf77Bits[bitzeiger - start_Kalendertag];
						kalenderpar++;
					}
				}
			
				if ((bitzeiger>=start_Wochentag)&&(bitzeiger<=end_Wochentag))	//Wochentagsbits
				{
					if (eins==1)
					{
						wochentag+=dcf77Bits[bitzeiger - start_Wochentag];
						kalenderpar++;
					}
				}
				
				if ((bitzeiger>=start_Monat)&&(bitzeiger<=end_Monat))	//Monatsbits
				{
					if (eins==1)
					{
						kalendermonat+=dcf77Bits[bitzeiger - start_Monat];
						kalenderpar++;
					}
				}
				
				if ((bitzeiger>=start_Jahr)&&(bitzeiger<=end_Jahr))	//Jahrbits
				{
					if (eins==1)
					{
						kalenderjahr+=dcf77Bits[bitzeiger - start_Jahr];
						kalenderpar++;
					}
				}
				
				if  (bitzeiger == kalender_Par)					//par Kalender
				{
					par=kalenderpar %2; //Parität
					if (par==eins) //Parität stimmt
					{
						Zeit.kalendertag=kalendertag;
						Zeit.wochentag=wochentag;
						Zeit.kalendermonat=kalendermonat;
						Zeit.kalenderjahr=kalenderjahr;
						txbuffer[0]=Zeit.minute;
						txbuffer[1]=Zeit.stunde;
						txbuffer[2]=Zeit.kalendertag;
						txbuffer[3]=Zeit.kalendermonat;
						txbuffer[4]=Zeit.kalenderjahr;
						txbuffer[5]=Zeit.wochentag;
						txbuffer[6]=fehler;
					}
					else
					{
						dcf77OK=0;
						fehler++;
						anzeigestunden=99;
					}
				}
				
				bitzeiger++;//Beginn der relevanten Datenreihe
					
				TCCR1B &= ~((1<<CS10)|(1<<CS11)|(1<<CS12));//Timer1 stoppen
				DCFImpuls =0;
				DCFImpuls_wait =0;
				if (TWIACK_wait)
				{
//					TWCR_ACK; // nachträglich ACK schicken:nächstes Datenbyte empfangen, ACK danach
//					TWIACK_wait=0;
				}
            
            
   #pragma mark timer2 Overflow
				//TIMSK |= (1<<TOIE1);			//Overflow Interrupt aktivieren
				//Timer2 starten: Pause messen
				TCCR2 |= (1<<CS20)|(1<<CS22);	//Takt /1024
				TCNT2=0x00;
            timer0div &= ~(1<<4);
            timer2div=0;
				PausenOverflowCounter=0;
				
//				DCF_busy=0; // Impuls fertig
				sei();
						
			}//If DCFImpuls
			
			else
			{
				//Pause
				if (TIFR &(1<<TOV2))//Overflow Flag Zaehler2 gesetzt 
				{
               timer2div ++;
               if (timer2div & 0x02)
               {
                  //PausenOverflowCounter++;
                  //timer2div=0;
               }
               
               
               timer0div ^= (1<<4);
               if (timer0div & (1<<4))
               {
                  //PausenOverflowCounter++;
               }
               PausenOverflowCounter++;
					TIFR |=(1<<TOV2);//Interrupt Flag loeschen (Atmega complete p. 71)
				}
            
				//if (PausenOverflowCounter>20) // 1 MHz
            
            if (PausenOverflowCounter>40*TAKTFAKTOR) // 1 Mhz
				{
					//                Sekunde 59
					bitzeiger=0;
					stunden=0;
					stundenpar=0;
					minuten=0;
					minutenpar=0;
					kalenderpar=0;
					kalendertag=0;
					kalendermonat=0;
					kalenderjahr=0;
					wochentag=0;
               
               timer0div=0;
					
					if (dcf77OK==0)
					{
						PORTD |=(1<<PORTD7);//WarnLED ON
					}
					else
					{
						PORTD &= ~(1<<PORTD7);
					}
					
					dcf77OK=1;
					//PORTC &= ~(1<<PORTC2);//kurzer Impuls aus
					//PORTC &= ~(1<<PORTC3);//langer Impuls aus
							
							//				PORTC |= (1<<PORTC4);	//Overflow LED ein
							//TCCR2 &= ~((1<<CS20)|(1<<CS21)|(1<<CS22));//Timer2 stoppen in Beginn neuer Impuls
					PausenOverflowCounter=0;
							
							
							
				}
				
			}
			
		}//if DCF
		 
		switch (Displaymode)
		{
			case 0://Uhrzeit
			{
            //Anzeigewert= 9990;
            //anzeigeminuten=22;
            //anzeigestunden=5;
            
				Anzeigewert=anzeigeminuten+100*anzeigestunden+10000*(Zeit.wochentag-1);
            //Anzeigewert=anzeigeminuten+100*anzeigestunden;
            //Anzeigewert= 1111;
            //Anzeigewert=timer0div + 100*timer2div;
            //Anzeigewert=timer0div;
            
			}break;
			case 1://Datum
			{
				Anzeigewert=Zeit.kalendermonat+100*Zeit.kalendertag;//+10000*Zeit.wochentag;
				//Anzeigewert= 9991;
			}break;
			case 2://Jahr
			{
				Anzeigewert=Zeit.kalenderjahr;//+0x7d0+10000*Zeit.wochentag;
            //Anzeigewert= 9992;
			}break;
			case 3://Wochentag
			{
				Anzeigewert=Zeit.wochentag;// +10000*Zeit.wochentag;
            //Anzeigewert= 9993;
            
			}break;
			case 4:
			{
			Anzeigewert=9999;
				
			}break;	
		
		}//switch displaymode
		
		//	Anzeigewert=0x05;
		// 	Anzeigewert=dauer;
		//	Anzeigewert=eins;
		//	Anzeigewert=bitzeiger+100*eins;
		//	Anzeigewert=bitzeiger+100*dauer/4;
		//	Anzeigewert=stunden;
		//	Anzeigewert=minuten;
		//	Anzeigewert=bitzeiger+100*stunden;
		//	Anzeigewert=Zeit.wochentag;
		
		//Digits anzeigen		
		if (digitdelaycountH==0)//segDelay cycles sind abgelaufen
		{
			int pos=0;
			uint8_t ziffer=0;
			while (pos<digitnummer)
			{
				Anzeigewert/=10;
				pos++;
			}
			ziffer=Anzeigewert % 10;
			
			{
				
				ZifferZeigen(ziffer,digitnummer);
			}
			if (digitnummer<maxdigit)
			{
				digitnummer++;
			}
			else
			{
				digitnummer=0;
			}
		}


		if (digitdelaycountH==segDimm)//Dimmdauer erreicht
		{
			PORTD=0x00;//Display löschen
		}
		
		if (digitdelaycountH==segDelay) //Anzeigedauer erreicht
		{
			digitdelaycountH=0;//neue Runde
		}
		else
		{
			digitdelaycountH++;
		}
		
		/*
		//TWI-Routine von ISR
		if (TWCR & (1<<TWINT))	//Es ist etwas passiert am TWI
		{
		
//			while (PINC & (PINC<<PC0;); //Warten bis eventueller Sekundenimpuls beendet ist
			
//			PORTC |= (1<<PC3);	//Pin 3 blinken
//			delay_ms(200);
//			PORTC &= ~(1<<DDC3);
			
			//PORTB = 0xFF;
			//delay_ms(200);
			//PORTB = 0x00;
			//delay_ms(400);

			uint8_t data=0;
			switch (TW_STATUS) //TWI-Statusregister prüfen und nötige Aktion bestimmen 
			{
				
				case TW_SR_SLA_ACK: // 0x64 Slave Receiver, wurde adressiert	
					{
					//25.11. Warten bis ev Impuls fertig
	//				while (PINB & (1<<PB0));
	
					PORTC |= (1<<DDC3);	//Pin 3 blinken
	//				delay_ms(50);
//					PORTC &= ~(1<<DDC3);
					
					
					
			//		segDimm=0;
					
					TWCR_ACK; // nächstes Datenbyte empfangen, ACK danach
					buffer_adr=0xFF; //Bufferposition ist undefiniert
					}
					break;
					
				case TW_SR_DATA_ACK: // 0x80 Slave Receiver,Daten empfangen
					data=TWDR; //Empfangene Daten auslesen
					if (buffer_adr == 0xFF) //erster Zugriff, Bufferposition setzen
					{
						
						//Kontrolle ob gewünschte Adresse im erlaubten bereich
						if(data<=buffer_size)
						{
							buffer_adr= data; //Bufferposition wie adressiert setzen
						}
						else
						{
							buffer_adr=0; //Adresse auf Null setzen. Ist das sinnvoll?
						}				
						TWCR_ACK;	// nächstes Datenbyte empfangen, ACK danach, um nächstes Byte anzufordern
					}
						else //weiterer Zugriff, Daten empfangen
						{
							rxbuffer[buffer_adr]=data; //Daten in Buffer schreiben
							buffer_adr++; //Buffer-Adresse weiterzählen für nächsten Schreibzugriff
							if(buffer_adr<(buffer_size-1)) //im Buffer ist noch Platz für mehr als ein Byte
							{
								TWCR_ACK;// nächstes Datenbyte empfangen, ACK danach, um nächstes Byte anzufordern
							}
							else   //es kann nur noch ein Byte kommen, dann ist der Buffer voll
							{
								TWCR_NACK;//letztes Byte lesen, dann NACK, um vollen Buffer zu signaliseren
							}
						}
						break;
					
				case TW_ST_SLA_ACK: //?!?
				case TW_ST_DATA_ACK: //0xB8 Slave Transmitter, weitere Daten wurden angefordert
					
					if (buffer_adr == 0xFF) //zuvor keine Leseadresse angegeben! 
					{
						buffer_adr=0;
					}	
					TWDR = txbuffer[buffer_adr]; //Datenbyte senden 
					buffer_adr++; //bufferadresse für nächstes Byte weiterzählen
					if(buffer_adr<(buffer_size)) //im Buffer ist mehr als ein Byte, das gesendet werden kann
					{
						TWCR_ACK; //nächstes Byte senden, danach ACK erwarten
					}
						else
						{
							TWCR_NACK; //letztes Byte senden, danach NACK erwarten
						}
						break;
					
				case TW_ST_DATA_NACK: //0xC0 Keine Daten mehr gefordert 
				case TW_SR_DATA_NACK: //0x88 
				case TW_ST_LAST_DATA: //0xC8  Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
				case TW_SR_STOP: // 0xA0 STOP empfangen
				default: 	
					TWCR_RESET; //Übertragung beenden, warten bis zur nächsten Adressierung
					buffer_adr = 0xFF;
			//		segDimm=2;
					PORTC &= ~(1<<DDC3);

					break;
					
					
			}	//if TWINT	
		}
		//Ende TWI-Routine
		
		*/
		
		
	}//while Hauptschleife
	return 0;
}
