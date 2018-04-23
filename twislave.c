#ifndef _TWISLAVE_H
#define _TWISLAVE_H

#include <util/twi.h> //enthält z.B. die Bezeichnungen für die Statuscodes in TWSR
#include <avr/interrupt.h> //dient zur behandlung der Interrupts
#include <stdint.h> //definiert den Datentyp uint8_t


/*
Dieses Programm in einer separaten Datei (z.B. twislave.c) abspeichern und in das eigene Programm
einbinden.

Betrieb eines AVRs mit Hardware-TWI-Schnittstelle als Slave. Zu Beginn muss init_twi_slave mit der gewünschten
Slave-Adresse als Parameter aufgerufen werden. Der Datenaustausch mit dem Master erfolgt über die Buffer 
rxbuffer und txbuffer, auf die von Master und Slave zugegriffen werden kann. 
rxbuffer und txbuffer sind globale Variablen (Array aus uint8_t). 
Die Ansteuerung des rxbuffers, in den der Master schreiben kann, erfolgt ähnlich wie bei einem normalen I2C-EEPROM.
Man sendet zunächst die Bufferposition, an die man schreiben will, und dann die Daten. Die Bufferposition wird 
automatisch hochgezählt, sodass man mehrere Datenbytes hintereinander schreiben kann, ohne jedesmal
die Bufferadresse zu schreiben.
Um den txbuffer vom Master aus zu lesen, überträgt man zunächst in einem Schreibzugriff die gewünschte Bufferposition und
liest dann nach einem repeated start die Daten aus. Die Bufferposition wird automatisch hochgezählt, sodass man mehrere
Datenbytes hintereinander lesen kann, ohne jedesmal die Bufferposition zu schreiben.

Autor: Uwe Große-Wortmann (uwegw)
Status: Testphase, keine Garantie für ordnungsgemäße Funktion! 
letze Änderungen: 
23.03.07 Makros für TWCR eingefügt. Abbruch des Sendens, wenn der TXbuffer komplett gesendet wurde. 
24.03.07 verbotene Buffergrößen abgefangen
25.03.07 nötige externe Bibliotheken eingebunden


Abgefangene Fehlbedienung durch den Master:
- Lesen über die Grenze des txbuffers hinaus
- Schreiben über die Grenzen des rxbuffers hinaus
- Angabe einer ungültigen Schreib/Lese-Adresse
- Lesezuggriff, ohne vorher Leseadresse geschrieben zu haben


 */ 
 
#define TWI_PORT        PORTC
#define TWI_DDR         DDRC
#define TWI_PIN         PINC
#define SCLPIN         PORTC5
#define SDAPIN         PORTC4

#define TWI_WAIT_BIT      3
#define TWI_OK_BIT      4

#define STARTDELAYBIT   0
#define HICOUNTBIT      1


//%%%%%%%% von Benutzer konfigurierbare Einstellungen %%%%%%%%

#define buffer_size 8 //Größe der Buffer in Byte (2..254)


//%%%%%%%% Globale Variablen, die vom Hauptprogramm genutzt werden %%%%%%%%

/*Der Buffer, in dem die empfangenen Daten gespeichert werden. Der Slave funktioniert ähnlich  wie ein normales
 Speicher-IC [I2C-EEPROM], man sendet die Adresse, an die man schreiben will, dann die Daten, die interne Speicher-Adresse
 wird dabei automatisch hochgezählt*/
volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];

// Anzeige, dass Sekundenimpuls anliegt
volatile uint8_t DCF_busy;

// Daten sind angekommen
volatile uint8_t rxdata=0;

//%%%%%%%% Funktionen, die vom Hauptprogramm aufgerufen werden können %%%%%%%%
 

volatile uint8_t SlaveStatus = 0;

/*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
Parameter: adr: gewünschte Slave-Adresse*/
void init_twi_slave (uint8_t adr);



//%%%%%%%% ab hier sind normalerweise keine weiteren Änderungen erforderlich! %%%%%%%%//
//____________________________________________________________________________________//

#include <util/twi.h> //enthält z.B. die Bezeichnungen für die Statuscodes in TWSR


//Bei zu alten AVR-GCC-Versionen werden die Interrupts anders genutzt, daher in diesem Fall mit Fehlermeldung abbrechen
#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
	#error "This library requires AVR-GCC 3.4.5 or later, update to newer AVR-GCC compiler !"
#endif

//Schutz vor unsinnigen Buffergrößen
#if (buffer_size > 254)
	#error Buffer zu groß gewählt! Maximal 254 Bytes erlaubt.
#endif

#if (buffer_size < 2)
	#error Buffer muss mindestens zwei Byte groß sein!
#endif

void twidelay_ms(unsigned int ms)
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

volatile uint8_t DCFImpuls_wait;
volatile uint8_t TWIACK_wait;
volatile uint8_t buffer_adr; //"Adressregister" für den Buffer

extern volatile uint8_t status;

/*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
Parameter: adr: gewünschte Slave-Adresse
*/
void init_twi_slave (uint8_t adr)
{
	TWAR= adr; //Adresse setzen
	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
	TWCR|= (1<<TWEA) | (1<<TWEN)|(1<<TWIE); 	
	buffer_adr=0xFF; 
	DCF_busy=0; 
//	sei();
	//txbuffer mit Werten füllen, die der Master auslesen soll
	for(uint8_t ii=0;ii<buffer_size;ii++)
	{
		txbuffer[ii]=0;
	}
   SlaveStatus=0;
   SlaveStatus |= (1<<TWI_WAIT_BIT);

}


//Je nach Statuscode in TWSR müssen verschiedene Bitmuster in TWCR geschrieben werden(siehe Tabellen im Datenblatt!). 
//Makros für die verwendeten Bitmuster:

//ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  
//NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten     
#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
//switched to the non adressed slave mode...
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  

//Die Bitmuster für TWCR_ACK und TWCR_RESET sind gleich. Dies ist kein Fehler und dient nur der Übersicht!


/*ISR, die bei einem Ereignis auf dem Bus ausgelöst wird. Im Register TWSR befindet sich dann 
ein Statuscode, anhand dessen die Situation festgestellt werden kann.
*/
ISR (TWI_vect)  
{
	uint8_t data=0;
	//PORTB |= (1<<PORTB0);
	//twidelay_ms(400);
	//PORTB &= ~(1<<PORTB0);
	
	//PORTB &=~(1<<PB3)|(1<<PB4)|(1<<PB5)|(1<<PB6)|(1<<PB7);	//PORT löschen
	
	if (DCF_busy)	// Waehrend DCF-Impuls keine Antwort auf TWI
	{
		return;
	
	}
	switch (TW_STATUS) //TWI-Statusregister prüfen und nötige Aktion bestimmen 
	{
			
		case TW_SR_SLA_ACK: // 0x60 Slave Receiver, wurde adressiert
			
			status |= (1<<2);
			
			TWCR_ACK; // nächstes Datenbyte empfangen, ACK danach
			buffer_adr=0xFF; //Bufferposition ist undefiniert
			
			break;
			
		case TW_SR_DATA_ACK: // 0x80 Slave Receiver,Daten empfangen
			data=TWDR; //Empfangene Daten auslesen
			status |= (1<<1);
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
				rxdata=1;
				rxbuffer[buffer_adr]=data; //Daten in Buffer schreiben
				buffer_adr++; //Buffer-Adresse weiterzählen für nächsten Schreibzugriff
				if(buffer_adr<(buffer_size-1)) //im Buffer ist noch Platz für mehr als ein Byte
				{
					TWCR_ACK;// nächstes Datenbyte empfangen, ACK danach, um nächstes Byte anzufordern
				}
				else   //es kann nur noch ein Byte kommen, dann ist der Buffer voll
				{
					status |= (1<<4);
					buffer_adr = 0xFF;		// von Buero N: buffer_adr wieder als undefiniert setzen

					TWCR_NACK;//letztes Byte lesen, dann NACK, um vollen Buffer zu signaliseren
					rxdata=1;
				}
			}
			break;
			
			case TW_ST_SLA_ACK: //?!?
			case TW_ST_DATA_ACK: //0xB8 Slave Transmitter, weitere Daten wurden angefordert
			
			if (buffer_adr == 0xFF) //zuvor keine Leseadresse angegeben! 
			{
				buffer_adr=0;
				status |= (1<<5);
			}	
			TWDR = txbuffer[buffer_adr]; //Datenbyte senden 
			buffer_adr++; //bufferadresse für nächstes Byte weiterzählen
			if(buffer_adr<(buffer_size-1)) //im Buffer ist mehr als ein Byte, das gesendet werden kann
			{	
				status |= (1<<6);
				TWCR_ACK; //nächstes Byte senden, danach ACK erwarten
				
			}
			else
			{
				status |= (1<<7);
				TWCR_NACK; //letztes Byte senden, danach NACK erwarten
				
				buffer_adr=0xFF; //Bufferposition ist undefiniert
			}
			break;
			
			case TW_ST_DATA_NACK: //0xC0 Keine Daten mehr gefordert 
			case TW_SR_DATA_NACK: //0x88 
			case TW_ST_LAST_DATA: //0xC8  Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
			case TW_SR_STOP: // 0xA0 STOP empfangen
			default: 	
			TWCR_RESET; //Übertragung beenden, warten bis zur nächsten Adressierung
			buffer_adr=0xFF; //Bufferposition ist undefiniert

			
			break;
			
			
	} //end.switch (TW_STATUS)
} //end.ISR(TWI_vect)

uint8_t i2c_debloc(void)
{   
   TWCR &= ~(1<<TWEN); //disable TWI
   TWI_DDR &= ~(1<<SDAPIN);   // SDA-Pin als EINgang
   TWI_PORT |= (1<<SDAPIN);  // HI
   TWI_DDR &= ~(1<<SCLPIN);   // SCL-Pin als EINgang
   TWI_PORT |= (1<<SCLPIN);  // HI
   _delay_ms(2);
   TWI_DDR |= (1<<SCLPIN);   // SCL-Pin als Ausgang
   TWI_PORT |= (1<<SCLPIN);  // HI
   _delay_ms(2);
   uint8_t i=0x0F;
   while (i ) 
   {
      // SCL-Pulse schicken, um Slave zu deblockieren
      TWI_PORT &= ~(1<<SCLPIN); // LO
      _delay_ms(1);
      TWI_PORT |= (1<<SCLPIN);   // HI
      _delay_ms(10);
      i--;
      if (TWI_PIN & (1<<SDAPIN)) // SDA ist wieder HI
      {
         
         return i;
      }
      
   }
   return i;
}

#endif //#ifdef _TWISLAVE_H
////Ende von twislave.c////

