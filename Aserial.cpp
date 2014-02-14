/* ============================================================
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================*/

// Based loosely on:
// Atmel AVR304: Half Duplex Interrupt Driven Software UART

// Author: Mike Blandford

#include "oxs_config.h"
#include <Arduino.h>
#include "Aserial.h"

#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

struct t_sportData *FirstData = 0 ;
struct t_sportData *ThisData = 0 ;

#define DEBUG 1

#ifdef FRSKY_SPORT
#define BR_57600     //!< Desired baudrate

//This section chooses the correct timer values for the chosen baudrate.
// Assumes a 16MHz clock
#define TICKS2COUNT         278  // Ticks between two bits.
#define TICKS2WAITONE       278  // Wait one bit period.
#define TICKS2WAITONE_HALF  416	 // Wait one and a half bit period.

#else
#define BR_9600     //!< Desired baudrate
//This section chooses the correct timer values for 9600 baud.
// Assumes a 16MHz clock
#define TICKS2COUNT         (278*6)  // Ticks between two bits.
#define TICKS2WAITONE       (278*6)  // Wait one bit period.
#define TICKS2WAITONE_HALF  (416*6)	 // Wait one and a half bit period.
#endif

#define INTERRUPT_EXEC_CYCL   90       // Cycles to execute interrupt routines from interrupt.
#define INTERRUPT_EARLY_BIAS  32       // Cycles to allow of other interrupts.
// INTERRUPT_EARLY_BIAS is to bias the sample point a bit early in case
// the Timer 0 interrupt (5.5uS) delays the start bit detection

//Some IO, timer and interrupt specific defines.

#define ENABLE_TIMER_INTERRUPT( )       ( TIMSK1 |= ( 1<< OCIE1A ) )
#define DISABLE_TIMER_INTERRUPT( )      ( TIMSK1 &= ~( 1<< OCIE1A ) )
#define CLEAR_TIMER_INTERRUPT( )        ( TIFR1 = (1 << OCF1A) )

#define	DISABLE_TIMER0_INT()					( TIMSK0 &= ~(1<<TOIE0) )
#define	ENABLE_TIMER0_INT()						( TIMSK0 |= (1<<TOIE0) )
//#define	DISABLE_TWI_INT()							( TWCR &= ~(1<<TWINT) )
//#define	ENABLE_TWI_INT()							( TWCR |= (1<<TWINT) )

#define TCCR             TCCR1A             //!< Timer/Counter Control Register
#define TCCR_P           TCCR1B             //!< Timer/Counter Control (Prescaler) Register
#define OCR              OCR1A              //!< Output Compare Register
#define EXT_IFR          EIFR               //!< External Interrupt Flag Register
#define EXT_ICR          EICRA              //!< External Interrupt Control Register
//  #define TIMER_COMP_VECT  TIMER1_COMPA_vect  //!< Timer Compare Interrupt Vector

#define TRXDDR  DDRD
#define TRXPORT PORTD
#define TRXPIN  PIND

#define SET_TX_PIN( )    ( TRXPORT |= ( 1 << PIN_SerialTX ) )
#define CLEAR_TX_PIN( )  ( TRXPORT &= ~( 1 << PIN_SerialTX ) )
#define GET_RX_PIN( )    ( TRXPIN & ( 1 << PIN_SerialTX ) )

#ifdef FRSKY_SPORT
 #ifndef SENSOR_ID
  #define SENSOR_ID		0x1B
 #endif
#endif

// UART's state.
#define   IDLE               0        // Idle state, both transmit and receive possible.
#define   TRANSMIT           1        // Transmitting byte.
#define   TRANSMIT_STOP_BIT  2        // Transmitting stop bit.
#define   RECEIVE            3        // Receiving byte.
#define		TxPENDING          4
#define		WAITING            5

static volatile uint8_t state ;     //!< Holds the state of the UART.
static volatile unsigned char SwUartTXData;     //!< Data to be transmitted.
static volatile unsigned char SwUartTXBitCount; //!< TX bit counter.
static volatile uint8_t SwUartRXData;     //!< Storage for received bits.
static volatile unsigned char SwUartRXBitCount; //!< RX bit counter.

#ifdef FRSKY_SPORT
uint8_t LastRx ;
uint8_t TxCount ;
uint8_t TxData[7] ;
uint16_t Crc ;
uint8_t DataSent ;
uint8_t SportSync ;

void setNewData( struct t_sportData *pdata, uint16_t id, uint32_t value )
{
	pdata->dataLock = 1 ;
	pdata->data[0] = 0x10 ;
	pdata->data[1] = id ;
	pdata->data[2] = id >> 8 ;
	pdata->data[3] = value ;
	pdata->data[4] = value >> 8 ;
	pdata->data[5] = value >> 16 ;
	pdata->data[6] = value >> 24 ;
	pdata->dataLock = 0 ;
}
#else

volatile uint8_t ByteToSend ;
volatile uint8_t TxNotEmpty ;

void startTransmit()
{
	if ( state != IDLE )
	{
		return ;
	}
  SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
	cli() ;
	OCR1A = TCNT1 + TICKS2WAITONE ;   // Count one period into the future.
 	CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
	sei() ;
  SwUartTXBitCount = 0 ;
	SwUartTXData = ByteToSend ;
	TxNotEmpty = 0 ;
  state = TRANSMIT ;
 	ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
#if DEBUG
	PORTC &= ~1 ;
#endif

}

void sendHubByte( uint8_t byte )
{
	while ( TxNotEmpty )		// Busy
	{
		// wait
	}	
	ByteToSend = byte ;
	TxNotEmpty = 1 ;
	startTransmit() ;
	while ( state != IDLE )
	{
		// wait
	}
}

#endif

 /*! \brief  External interrupt service routine.
 *
 *  The falling edge in the beginning of the start
 *  bit will trig this interrupt. The state will
 *  be changed to RECEIVE, and the timer interrupt
 *  will be set to trig one and a half bit period
 *  from the falling edge. At that instant the
 *  code should sample the first data bit.
 *
 *  \note  initSoftwareUart( void ) must be called in advance.
 */
// This is the pin change interrupt for port D
// This assumes it is the only pin change interrupt
// on this port
#ifdef FRSKY_SPORT
ISR(PCINT2_vect)
{
	if ( TRXPIN & ( 1 << PIN_SerialTX ) )			// Pin is high = start bit (inverted)
	{
		PCICR &= ~(1<<PCIE2) ;						// pin change interrupt disabled
//PORTC &= ~2 ;
		state = RECEIVE ;                 // Change state
  	
  	DISABLE_TIMER_INTERRUPT() ;       // Disable timer to change its registers.
  	
  	OCR1A = TCNT1 + TICKS2WAITONE_HALF - INTERRUPT_EXEC_CYCL - INTERRUPT_EARLY_BIAS ; // Count one and a half period into the future.

#if DEBUG
	PORTC |= 1 ;
#endif

  	SwUartRXBitCount = 0 ;            // Clear received bit counter.
  	CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
  	ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
	}
}
#endif
/*! \brief  Timer0 interrupt service routine.
 *
 *  Timer0 will ensure that bits are written and
 *  read at the correct instants in time.
 *  The state variable will ensure context
 *  switching between transmit and recieve.
 *  If state should be something else, the
 *  variable is set to IDLE. IDLE is regarded
 *  as a safe state/mode.
 *
 */

uint8_t ByteStuffByte = 0 ;

ISR(TIMER1_COMPA_vect)
{
  switch (state)
	{
  // Transmit Byte.
	  case TRANSMIT :		// Output the TX buffer.
#if DEBUG
	PORTC |= 1 ;
#endif
    	if( SwUartTXBitCount < 8 )
			{
    	  if( SwUartTXData & 0x01 )
				{           // If the LSB of the TX buffer is 1:
    	    CLEAR_TX_PIN() ;                    // Send a logic 1 on the TX_PIN.
    	  }
    	  else
				{                                // Otherwise:
    	    SET_TX_PIN() ;                      // Send a logic 0 on the TX_PIN.
    	  }
    	  SwUartTXData = SwUartTXData >> 1 ;    // Bitshift the TX buffer and
    	  SwUartTXBitCount += 1 ;               // increment TX bit counter.
    	}
    	else		//Send stop bit.
			{
    	  CLEAR_TX_PIN();                         // Output a logic 1.
    	  state = TRANSMIT_STOP_BIT;
				ENABLE_TIMER0_INT() ;									// Allow this in now.
    	}
	  	OCR1A += TICKS2WAITONE ;  // Count one period into the future.
#if DEBUG
	PORTC &= ~1 ;
#endif
  	break ;

  // Go to idle after stop bit was sent.
  case TRANSMIT_STOP_BIT:
#ifdef FRSKY_SPORT
		if ( ByteStuffByte || (++TxCount < 8) )		// Have we sent 8 bytes?
		{
			if ( ByteStuffByte )
			{
				SwUartTXData = ByteStuffByte ;
				ByteStuffByte = 0 ;
			}
			else
			{
				if ( TxCount < 7 )		// Data (or crc)?
				{
					SwUartTXData = TxData[TxCount] ;
				  Crc += SwUartTXData ; //0-1FF
				  Crc += Crc >> 8 ; //0-100
				  Crc &= 0x00ff ;
				}
				else
				{
					SwUartTXData = 0xFF-Crc ;
				}
				if ( ( SwUartTXData == 0x7E ) || ( SwUartTXData == 0x7D ) )
				{
					ByteStuffByte = SwUartTXData ^ 0x20 ;
					SwUartTXData = 0x7D ;					
				}
			}
      SET_TX_PIN() ;                // Send a logic 0 on the TX_PIN.
	  	OCR1A = TCNT1 + TICKS2WAITONE ;   // Count one period into the future.
		  SwUartTXBitCount = 0 ;
		  state = TRANSMIT ;
			DISABLE_TIMER0_INT() ;						// For the byte duration
		}
		else
		{
			state = WAITING ;
			OCR1A += ((uint16_t)3500*16) ;		// 3.5mS gap before listening
			TRXDDR &= ~( 1 << PIN_SerialTX );   // PIN is input, tri-stated.
		  TRXPORT &= ~( 1 << PIN_SerialTX );  // PIN is input, tri-stated.

			struct t_sportData *pdata = ThisData ;
			FORCE_INDIRECT( pdata ) ;

			pdata->serialSent = 1 ;
			DataSent = 1 ;
			SportSync = 1 ;
			pdata = pdata->next ;
			if ( pdata == 0 )		// Wrap at end
			{
				pdata = FirstData ;
			}
			ThisData = pdata ;
		}

#else
		if ( TxNotEmpty )
		{
      SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
	  	OCR1A = TCNT1 + TICKS2WAITONE ;   // Count one period into the future.
			SwUartTXData = ByteToSend ;		// Grab byte
			TxNotEmpty = 0 ;							// Mark 'sent'
 			state = TRANSMIT ;
		  SwUartTXBitCount = 0 ;
		}
		else
		{
			OCR1A += ((uint16_t)100*16) ;		// 100uS gap 
	    state = WAITING ;               // Go back to idle.
		}

#endif

  break ;

#ifdef FRSKY_SPORT
  //Receive Byte.
  case RECEIVE :
    OCR1A += TICKS2WAITONE ;                // Count one period after the falling edge is trigged.
    //Receiving, LSB first.
		{
			uint8_t data ;				// Use a temporary local storage
		 	data = SwUartRXBitCount ;
    	if( data < 8 )
			{
    	  SwUartRXBitCount = data + 1 ;
				data = SwUartRXData ;
				data >>= 1 ;		         // Shift due to receiving LSB first.
#if DEBUG
	PORTC &= ~1 ;
#endif
    	  if( GET_RX_PIN( ) == 0 )
				{
    	    data |= 0x80 ;          // If a logical 1 is read, let the data mirror this.
    	  }
#if DEBUG
	PORTC |= 1 ;
#endif
				SwUartRXData = data ;
    	}
    	else	//Done receiving
			{
#if DEBUG
	PORTC &= ~1 ;
#endif
				if ( LastRx == 0x7E )
				{
					if ( SwUartRXData == SENSOR_ID )
					{
						// This is us
						struct t_sportData *pdata = ThisData ;
						FORCE_INDIRECT( pdata ) ;
						if ( pdata )	// We have something to send
						{
							if ( pdata->dataLock == 0 )
							{
 								TxData[0] = pdata->data[0] ;
 								TxData[1] = pdata->data[1] ;
 								TxData[2] = pdata->data[2] ;
 								TxData[3] = pdata->data[3] ;
 								TxData[4] = pdata->data[4] ;
 								TxData[5] = pdata->data[5] ;
 								TxData[6] = pdata->data[6] ;
							}
							else
							{	// Discard frame to be sent
								TxData[0] = 0 ;
								TxData[1] = 0 ;
								TxData[2] = 0 ;
								TxData[3] = 0 ;
								TxData[4] = 0 ;
								TxData[5] = 0 ;
								TxData[6] = 0 ;
							}
							state = TxPENDING ;
							OCR1A += (400*16-TICKS2WAITONE) ;		// 400 uS gap before sending
						}
						else
						{
							// Wait for idle time
							state = WAITING ;
							OCR1A += ((uint16_t)3500*16) ;		// 3.5mS gap before listening
						}
					}
					else
					{
						state = WAITING ;
						OCR1A += ((uint16_t)3500*16) ;		// 3.5mS gap before listening
						SportSync = 1 ;
					}
				}
				else
				{
		    	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	  		  state = IDLE ;                // Go back to idle.
					PCIFR = (1<<PCIF2) ;					// clear pending interrupt
					PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled
//PORTC |= 2 ;
				}
				LastRx = SwUartRXData ;
    	}
		}
  break ;
  
		case TxPENDING :
#if DEBUG
	PORTC |= 1 ;
#endif
			TRXDDR |= ( 1 << PIN_SerialTX ) ;       // PIN is output
      SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
	  	OCR1A = TCNT1 + TICKS2WAITONE ;   // Count one period into the future.
		  SwUartTXBitCount = 0 ;
			Crc = SwUartTXData = TxData[0] ;
			TxCount = 0 ;
		  state = TRANSMIT ;
			DISABLE_TIMER0_INT() ;						// For the byte duration
#if DEBUG
	PORTC &= ~1 ;
#endif
  	break ;
#endif

		case WAITING :
#ifdef FRSKY_SPORT
    	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	    state = IDLE ;                // Go back to idle.
			PCIFR = (1<<PCIF2) ;					// clear pending interrupt
			PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled
//PORTC |= 2 ;
#else  	
		if ( TxNotEmpty )
		{
      SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
	  	OCR1A = TCNT1 + TICKS2WAITONE ;   // Count one period into the future.
			SwUartTXData = ByteToSend ;		// Grab byte
			TxNotEmpty = 0 ;							// Mark 'sent'
 			state = TRANSMIT ;
		  SwUartTXBitCount = 0 ;
		}
		else
		{
    	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	    state = IDLE ;                // Go back to idle.
		}
#endif
		
		break ;

  // Unknown state.
	  default:        
  	  state = IDLE;                           // Error, should not occur. Going to a safe state.
  }
}


/*! \brief  Function to initialize the UART.
 *
 *  This function will set up pins to transmit and
 *  receive on. Control of Timer0 and External interrupt 0.
 *
 *  \param  void
 *
 *  \retval void
 */

#ifdef FRSKY_SPORT
void initSportUart( struct t_sportData *pdata )
{
  FirstData = ThisData = pdata ;

  // PORT
  TRXDDR &= ~( 1 << PIN_SerialTX ) ;       // PIN is input, tri-stated.
  TRXPORT &= ~( 1 << PIN_SerialTX ) ;      // PIN is input, tri-stated.

  // External interrupt

#if PIN_SerialTX == 4
  PCMSK2 |= 0x10 ;  // IO4 (PD4) on Arduini mini
#elif PIN_SerialTX == 2
  PCMSK2 |= 0x04 ;  // IO2 (PD2) on Arduini mini
#else
  #error "This PIN is not supported"
#endif

  PCIFR = (1<<PCIF2) ;	// clear pending interrupt
  PCICR |= (1<<PCIE2) ;	// pin change interrupt enabled

  // Internal State Variable
  state = IDLE ;

#if DEBUG
  DDRC = 0x03 ;		// PC0,1 as o/p debug
  PORTC = 0 ;
#endif
}

#else // FRSKY_SPORT

void initHubUart()
{
  TRXPORT &= ~( 1 << PIN_SerialTX ) ;      // PIN is low
  TRXDDR |= ( 1 << PIN_SerialTX ) ;       // PIN is output.
	
  //Internal State Variable
  state = IDLE ;

#if DEBUG
  DDRC = 0x03 ;		// PC0,1 as o/p debug
  PORTC = 0 ;
#endif
}

#endif // FRSKY_SPORT

uint16_t MillisPrecount ;
uint16_t lastTimerValue ;
uint32_t TotalMicros ;
uint32_t TotalMillis ;

uint32_t lastEventTime ;
uint8_t RpmCounter ;
uint8_t RpmSet ;
uint16_t Rpm = 60 ;

// RPM code
#ifdef MEASURE_RPM
ISR( TIMER1_CAPT_vect, ISR_NOBLOCK )
{
	uint16_t elapsed ;
	uint32_t eventTime ;
	uint32_t difference ;

	if ( ++RpmCounter > 3 )
	{
		cli() ;
		uint16_t time = ICR1 ;	// Read timer 1
		sei() ;
		elapsed = time - lastTimerValue ;
		eventTime = TotalMicros + ( elapsed >> 4 ) ;
		RpmCounter = 0 ;
		difference = eventTime - lastEventTime ;
		lastEventTime = eventTime ;
		if ( difference > 400 )
		{
			Rpm = 4000000 / difference ;
		}
		else
		{
			Rpm = 0 ;
		}
		RpmSet = 1 ;
	}
}

#endif // MEASURE_RPM

uint32_t micros()
{
	uint16_t elapsed ;
	uint8_t millisToAdd ;
	uint8_t oldSREG = SREG ;
	cli() ;
	uint16_t time = TCNT1 ;	// Read timer 1
	SREG = oldSREG ;

	elapsed = time - lastTimerValue ;
	elapsed >>= 4 ;
	
	uint32_t ltime = TotalMicros ;
	ltime += elapsed ;
	cli() ;
	TotalMicros = ltime ;	// Done this way for RPM to work correctly
	lastTimerValue = time ;
	SREG = oldSREG ;	// Still valid from above
	
	elapsed += MillisPrecount;
	millisToAdd = 0 ;
	if ( elapsed  > 3999 )
	{
		millisToAdd = 4 ;
		elapsed -= 4000 ;
	}
	else if ( elapsed  > 2999 )
	{
		millisToAdd = 3 ;		
		elapsed -= 3000 ;
	}
	else if ( elapsed  > 1999 )
	{
		millisToAdd = 2 ;
		elapsed -= 2000 ;
	}
	else if ( elapsed  > 999 )
	{
		millisToAdd = 1 ;
		elapsed -= 1000 ;
	}
	TotalMillis += millisToAdd ;
	MillisPrecount = elapsed ;
	return TotalMicros ;
}

uint32_t millis()
{
	micros() ;
	return TotalMillis ;
}

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void init()
{
  // Timer1
	TIMSK1 &= ~( 1<< OCIE1A ) ;
  TCCR1A = 0x00 ;    //Init.
  TCCR1B = 0xC1 ;    // I/p noise cancel, rising edge, Clock/1

#if defined(ADCSRA)
	// set a2d prescale factor to 128
	// 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
	// XXX: this will not work properly for other clock speeds, and
	// this code should use F_CPU to determine the prescale factor.
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);

	// enable a2d conversions
	sbi(ADCSRA, ADEN);
#endif

	// the bootloader connects pins 0 and 1 to the USART; disconnect them
	// here so they can be used as normal digital i/o; they will be
	// reconnected in Serial.begin()
#if defined(UCSRB)
	UCSRB = 0;
#elif defined(UCSR0B)
	UCSR0B = 0;
#endif

#ifdef MEASURE_RPM
	DDRB &= ~0x01 ;	// Pin is input
	PORTB |= 1 ; 		// With pullup
	sbi( TIMSK1, ICIE1 ) ;
#endif // MEASURE_RPM
	sei();

}

void delay(unsigned long ms)
{
	uint16_t start = (uint16_t)micros();
	uint16_t lms = ms ;

	while (lms > 0) {
		if (((uint16_t)micros() - start) >= 1000) {
			lms--;
			start += 1000;
		}
	}
}
 
// Delay for the given number of microseconds.  Assumes a 8 or 16 MHz clock. 
void delayMicroseconds(unsigned int us)
{
//	 calling avrlib's delay_us() function with low values (e.g. 1 or
//	 2 microseconds) gives delays longer than desired.
//	delay_us(us);
#if F_CPU >= 20000000L
//	 for the 20 MHz clock on rare Arduino boards

//	 for a one-microsecond delay, simply wait 2 cycle and return. The overhead
//	 of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop"); //just waiting 2 cycle
	if (--us == 0)
		return;

//	 the following loop takes a 1/5 of a microsecond (4 cycles)
//	 per iteration, so execute it five times for each microsecond of
//	 delay requested.
	us = (us<<2) + us; // x5 us

//	 account for the time taken in the preceeding commands.
	us -= 2;

#elif F_CPU >= 16000000L
//	 for the 16 MHz clock on most Arduino boards

//	 for a one-microsecond delay, simply return.  the overhead
//	 of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

//	 the following loop takes a quarter of a microsecond (4 cycles)
//	 per iteration, so execute it four times for each microsecond of
//	 delay requested.
	us <<= 2;

//	 account for the time taken in the preceeding commands.
	us -= 2;
#else
//	 for the 8 MHz internal clock on the ATmega168

//	 for a one- or two-microsecond delay, simply return.  the overhead of
//	 the function calls takes more than two microseconds.  can't just
//	 subtract two, since us is unsigned; we'd overflow.
	if (--us == 0)
		return;
	if (--us == 0)
		return;

//	 the following loop takes half of a microsecond (4 cycles)
//	 per iteration, so execute it twice for each microsecond of
//	 delay requested.
	us <<= 1;
    
//	 partially compensate for the time taken by the preceeding commands.
//	 we can't subtract any more than this or we'd overflow w/ small delays.
	us--;
#endif

//	 busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}

