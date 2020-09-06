/* 
 * File:   main.c COILS
 * Author: th
 * Coil Control for Bally 17er & 35er MPU
 * Part of LISY35
 * READ Port / Write LAT
 * 
 */

//no warnings about 'implicite signed to unsigned conversion'
#pragma warning push
#pragma warning disable 373

#include "def_coils.h"
#include <stdint.h>
#include <pic18f45k22.h>

#include "fifo_coils.h"
#include "eeprom.h"
#include "interrupt_delay.h"

int lisy35_coil_version = 499; //LISY35 COIL Version  // Development build

/* global definitions */         
volatile uint8_t data_slave;  
volatile bitv_t inhibit[16];  //inhibit bits for board0
volatile bitv_t inhibit2[16];  //inhibit bits for board1

volatile uint8_t lampboard_active = 0;
volatile uint8_t lampdriver_variant = NO_AUX_BOARD;
volatile uint8_t ext_soundboard_type = EXT_SB_IS_51;
volatile uint8_t sound_raw = 0;

bitv_t dip1,dip2,dip3,dip4;   //the four dip switches

    
//Lamdriverboard refresh routine
//called by ext INT via Timer0
//refreshs also second board if available
void do_lampdriver_refresh(void)
{
        //static int zerocross = 0;
        static uint16_t zerocross = 0;
    //this is the refresh cycle for the lampdriver
    //lets indicate via LED once asecond
    //as we have 100 zerocrossings per second ( 120 in US)
    //the LED does blink every second ( one zerocross every 10 milliseconds) 
     if  (zerocross++ > 100) { 
           zerocross=0;
           ZERO_CR_LED = ~ZERO_CR_LED; }

    bitv_t address;
    
    //we do ignite all thyristors for this zero cross cycle
    //where lamps are on (1)
    //this information is stored in the 15 inhibit bytess
    //which are prepared during lamp_set
    
    //this is for board 0
    //which is always a 60-lamps "AS_2518_23"
    for ( address.byte = 0; address.byte < 15; address.byte = address.byte +1)
    {          
    //inhibit (data) is all 1, we deactivate all decoders
    PD0 = PD1 = PD2 = PD3 = 1;  
    //set strobe to high
    LAMPSTR1 = 1;        
    //set the address
    AD0 = address.bitv.b0;  
    AD1 = address.bitv.b1;
    AD2 = address.bitv.b2;
    AD3 = address.bitv.b3;
    // do some calculation to give signals stable time before strobe    
    //read status of LED (bit2) in order not change it by write
    inhibit[address.byte].bitv.b2 = LED_READ;
    //add extra delay
    __delay_us(3);  //we add 3us->8us (without it was 5us) Bally has 15us
    //set strobe to low, this will strobe address into the decoders
    LAMPSTR1 = 0;    
    //set the rigth inhibit for this address
    //we write the whole byte
    INHIBIT = inhibit[address.byte].byte;
    __delay_us(3);  //give also some extra time for inhibit    
    }    
    //final step: set the address back to 15 for all decoders (Rest)
    //inhibit (data) is all 1, we deactivate all decoders
    PD0 = PD1 = PD2 = PD3 = 1;          
    //set strobe to high
    LAMPSTR1 = 1;    
    //set the address back to 15 for all decoders
    AD0 = AD1 = AD2 = AD3 = 1;
    //wait ab bit
    __delay_us(8); 
    //set strobe to low, this will strobe address into the decoders
    LAMPSTR1 = 0;        
    //activate all decoders for rest address
    PD0 = PD1 = PD2 = PD3 = 0;          
    
    //this is for board 1, if it exists    
    switch (lampdriver_variant)   
    {     
       case AS_2518_43_12_LAMPS:    //push with low-high!
        for ( address.byte = 0; address.byte < 3; address.byte = address.byte +1)
        {          
        //inhibit (data) is all 1, we deactivate all decoders
        PD0 = PD1 = PD2 = PD3 = 1;  
        //set strobe to low
        LAMPSTR2 = 0;            
        //set the address
        AD0 = address.bitv.b0;  
        AD1 = address.bitv.b1;        
        // do some calculation to give signals stable time before strobe    
        //read status of LED (bit2) in order not change it by write
        inhibit2[address.byte].bitv.b2 = LED_READ;        
        //add extra delay
        __delay_us(3);  //we add 3us->8us (without it was 5us) Bally has 15us        
        //set strobe to high, this will push strobe address to the decoders
        LAMPSTR2 = 1;    
        //set the rigth inhibit for this address which will activate decoder or not
        //we write the whole byte
        INHIBIT = inhibit2[address.byte].byte;
        __delay_us(3);  //give also some extra time for inhibit
        }      
        //final step: set the address back to 3 for all decoders (Rest)
        //inhibit (data) is all 1, we deactivate all decoders
        PD0 = PD1 = PD2 = PD3 = 1;          
        //set strobe to high
        LAMPSTR2 = 0;            
        //set the address back to 3 for all decoders
        AD0 = AD1 = 1;        
        //give the 14175 some data setup time        
        __delay_us(8);                 
        //set strobe to high, this will push strobe address to the decoders
        LAMPSTR2 = 1;        
        //activate all decoders for rest address
        PD0 = PD1 = PD2 = PD3 = 0;          
        break;
       case AS_2518_52_28_LAMPS:    //push with low-high!
        for ( address.byte = 0; address.byte < 7; address.byte = address.byte +1)
        {                     
        //inhibit (data) is all 1, we deactivate all decoders
        PD0 = PD1 = PD2 = PD3 = 1;  
        //set strobe to low
        LAMPSTR2 = 0;            
        //set the address
        AD0 = address.bitv.b0;  
        AD1 = address.bitv.b1;        
        AD2 = address.bitv.b2;                
        // do some calculation to give signals stable time before strobe    
        //read status of LED (bit2) in order not change it by write
        inhibit2[address.byte].bitv.b2 = LED_READ;        
        //add extra delay
        __delay_us(3);  //we add 3us->8us (without it was 5us) Bally has 15us        
        //set strobe to high, this will push strobe address to the decoders
        LAMPSTR2 = 1;    
        //set the rigth inhibit for this address which will activate decoder or not
        //we write the whole byte
        INHIBIT = inhibit2[address.byte].byte;
        __delay_us(3);  //give also some extra time for inhibit        
        }      
        //final step: set the address back to 3 for all decoders (Rest)
        //inhibit (data) is all 1, we deactivate all decoders
        PD0 = PD1 = PD2 = PD3 = 1;          
        //set strobe to high
        LAMPSTR2 = 0;            
        //set the address back to 7 for all decoders
        AD0 = AD1 = AD2 = 1;    
        //give the 14175 some data setup time
        __delay_us(8);         
        //set strobe to high, this will push strobe address to the decoders
        LAMPSTR2 = 1;        
        //activate all decoders for rest address
        PD0 = PD1 = PD2 = PD3 = 0;  
        break;
       case AS_2518_23_60_LAMPS:    //push with high-low
        for ( address.byte = 0; address.byte < 15; address.byte = address.byte +1)
        {          
            //inhibit (data) is all 1, we deactivate all decoders
            PD0 = PD1 = PD2 = PD3 = 1;  
            //set strobe to high
            LAMPSTR2 = 1;        
            //set the address
            AD0 = address.bitv.b0;  
            AD1 = address.bitv.b1;
            AD2 = address.bitv.b2;
            AD3 = address.bitv.b3;
            // do some calculation to give signals stable time before strobe    
            //read status of LED (bit2) in order not change it by write
            inhibit2[address.byte].bitv.b2 = LED_READ;
            //add extra delay
            __delay_us(3);  //we add 3us->8us (without it was 5us) Bally has 15us            
            //set strobe to low, this will strobe address into the decoders
            LAMPSTR2 = 0;    
            //set the rigth inhibit for this address
            //we write the whole byte
            INHIBIT = inhibit2[address.byte].byte;
            __delay_us(3);  //give also some extra time for inhibit            
        }      
            //final step: set the address back to 15 for all decoders (Rest)
            //inhibit (data) is all 1, we deactivate all decoders
            PD0 = PD1 = PD2 = PD3 = 1;          
            //set strobe to high
            LAMPSTR2 = 1;    
            //set the address back to 15 for all decoders
            AD0 = AD1 = AD2 = AD3 = 1;
            //wait ab bit
            __delay_us(8);             
            //set strobe to low, this will strobe address into the decoders
            LAMPSTR2 = 0;        
            //activate all decoders for rest address
            PD0 = PD1 = PD2 = PD3 = 0;          
        break;                
    }//switch lampdriver_variant
    
}


/* eeprom background write via buffer */
/* called by interrupt routine each 8 ms */
/* this should NOT block execution as eeprom write */
/* is in the background and last write should already be finished */

/* to do a full write, we need 2 seconds at least */

void do_eeprom_write(void)
{
    static unsigned char address = 0;         
    
    if (eeprom_content_current[address] != eeprom_content_towrite[address])
    {
        writeEEPROM( address, eeprom_content_towrite[address] );
        eeprom_content_current[address] = eeprom_content_towrite[address];
    }
    
    //prepare next round
    if ( address == 255) address = 0; else address++;
    
}

/* interrut service routine for eeprom background write */
void __interrupt(low_priority) low_prio_Isr(void) {        
    
    // only process timer-triggered interrupts here
   if(INTCONbits.T0IE && INTCONbits.T0IF) {     
   
    //timer0 overflow,        
    //disable timer0
    T0CONbits.TMR0ON = 0;
       
    //and start now with refresh   
    do_lampdriver_refresh();
    
   /* do background eeprom write */
   /* we are her appr. each 10ms */   
   do_eeprom_write();             
  
  INTCONbits.T0IF = 0;  // clear this interrupt condition
  }    
  else if (INTCON3bits.INT1IE && INTCON3bits.INT1IF) 
  {
      
   // external INT/RB1 interupt      
   // we cannot start the refresh yet
   // so lets start the timer and do it there   
      TMR0 = TMR0_VALUE;    //set TMR0 to old value
      T0CONbits.TMR0ON = 1; //enable Timer 0           
            
    INTCON3bits.INT1IF = 0; // clear this interrupt condition
  }     
  if(PIE1bits.TMR2IE == 1 && PIR1bits.TMR2IF == 1)
  {
    // Handle interrupt for timer2 for the interruptable delay
    TMR2_ISR();
  }
}
    
/* interrut service routine NEW from AN734 */
void __interrupt(high_priority) myIsr(void)
{
    uint8_t junk;
    
    if(SSPIF)                               // check to see if SSP interrupt
    {
        //Master wants to read, we need to write        
        if(SSPSTATbits.R_nW)                // Master read (R_nW = 1)
        {
            if(!SSPSTATbits.D_nA)         // last byte was an address (D_nA = 0)
            {                                
                junk = SSPBUF;           // dummy read to clear BF bit (Step 6)                                               
                SSPBUF = data_slave;     //load transmit data to SSPBUF                
                SSPCON1bits.CKP = 1;     // release CLK
            }
            if(SSPSTATbits.D_nA)       // last byte was data
            {
                //we are only here with reading multiple bytes
                //one byte read is handled in state 3
                junk = SSPBUF;      // dummy read to clear BF bit                
                SSPBUF = data_slave;     //load transmit data to SSPBUF
                SSPCON1bits.CKP = 1;    // release CLK
            }
        }
        //Master wants to write, we need to read
        if(!SSPSTATbits.R_nW)			// master write (R_nW = 0)
        {                        
            if(!SSPSTATbits.D_nA)        // last byte was an address (D_nA = 0)
            {
                junk = SSPBUF;			// read buffer to clear BF
				SSPCON1bits.CKP = 1;            // release CLK
            }
            if(SSPSTATbits.D_nA)                // last byte was data (D_nA = 1)
            {
                //put byte from master to Buffer, interpret in main
            	BufferIn ( SSP1BUF );
				if(SSPCON1bits.WCOL)		// Did a write collision occur?
				{
                    SSPCON1bits.WCOL = 0;       // clear WCOL bit
                    junk = SSPBUF;              // clear SSPBUF
				}
				SSPCON1bits.CKP = 1;    		// release CLK
            }
    	}
        // this means it's time for the next command, thus stop the interruptable delay in sound_data_standard
        interrupt_delay();
        // Of course this is too often, but a single command to turn off tmr2 is probably quicker 
        // than to always check whether it is necessary.
    }
    if(BCLIF)                       // Did a bus collision occur?
    {
        junk = SSPBUF;              // clear SSPBUF
		BCLIF = 0;                  // clear BCLIF
		SSPCON1bits.CKP = 1;        // Release CLK
    }
    SSPIF = 0;                      // clear SSPIF
}

/*
 subrotines for settings
 */

/*
        lisy35_lamp_set
        coil -> nr of lamp 0..59
        action -> 0-off, 1-on
*/
void lamp_set( int lamp, int action) {
 
    unsigned char inhibit_bit;
    
    //we prepare the inhibits here for the interrupt refresh routine  
    //so the interrupt is shorter    
    //if inhibit is 1, we do NOT set the lamp
    if ( action) inhibit_bit = 0; else inhibit_bit = 1;
    
  //see for what board the lamp activity is for
  if (lampboard_active == 0)  
  {    
    switch(lamp)
    {
        case 0 ... 14: //decoder 1 PD0 which is bit4
            inhibit[lamp].bitv.b4 = inhibit_bit;           
            break;
        case 15 ... 29: //decoder 2 PD1 which is bit5
            lamp = lamp-15;
            inhibit[lamp].bitv.b5 = inhibit_bit;           
            break;
        case 30 ... 44: //decoder 3 PD2 which is bit6
            lamp = lamp-30;
            inhibit[lamp].bitv.b6 = inhibit_bit;           
            break;
        case 45 ... 59: //decoder 4 PD3 which is bit7
            lamp = lamp-45;
            inhibit[lamp].bitv.b7 = inhibit_bit;           
            break;                
          }    
  }
  else if(lampboard_active == 1)  
  {
   switch(lampdriver_variant)      
   {
       case AS_2518_43_12_LAMPS:   
        switch(lamp)
        {      
            case 0 ... 2: //decoder 1 PD0 which is bit4
                inhibit2[lamp].bitv.b4 = inhibit_bit;           
                break;
            case 3 ... 5: //decoder 2 PD1 which is bit5
                lamp = lamp-3;
                inhibit2[lamp].bitv.b5 = inhibit_bit;           
                break;
            case 6 ... 8: //decoder 3 PD2 which is bit6
                lamp = lamp-6;
                inhibit2[lamp].bitv.b6 = inhibit_bit;           
                break;
            case 9 ... 11: //decoder 4 PD3 which is bit7
                lamp = lamp-9;
                inhibit2[lamp].bitv.b7 = inhibit_bit;           
                break;                
            }    
        break;       
       case AS_2518_52_28_LAMPS:   
        switch(lamp)
        {      
            case 0 ... 6: //decoder 1 PD0 which is bit4
                inhibit2[lamp].bitv.b4 = inhibit_bit;           
                break;
            case 7 ... 13: //decoder 2 PD1 which is bit5
                lamp = lamp-7;
                inhibit2[lamp].bitv.b5 = inhibit_bit;           
                break;
            case 14 ... 20: //decoder 3 PD2 which is bit6
                lamp = lamp-14;
                inhibit2[lamp].bitv.b6 = inhibit_bit;           
                break;
            case 21 ... 27: //decoder 4 PD3 which is bit7
                lamp = lamp-21;
                inhibit2[lamp].bitv.b7 = inhibit_bit;           
                break;                
            }    
        break;
       case AS_2518_23_60_LAMPS:   
        switch(lamp)
        {      
            case 0 ... 14: //decoder 1 PD0 which is bit4
                inhibit2[lamp].bitv.b4 = inhibit_bit;           
                break;
            case 15 ... 29: //decoder 2 PD1 which is bit5
                lamp = lamp-15;
                inhibit2[lamp].bitv.b5 = inhibit_bit;           
                break;
            case 30 ... 44: //decoder 3 PD2 which is bit6
                lamp = lamp-30;
                inhibit2[lamp].bitv.b6 = inhibit_bit;           
                break;
            case 45 ... 59: //decoder 4 PD3 which is bit7
                lamp = lamp-45;
                inhibit2[lamp].bitv.b7 = inhibit_bit;           
                break;                
            }                  
        break;
   }
  }
}

void cont_sol_set(unsigned char value)
{
    bitv_t my_value;
    
    my_value.byte = value;
    //just set the values to the ports
    CO2_PB4 = my_value.bitv.b0;
    CO4_PB5 = my_value.bitv.b1;  //coin lockout
    CO1_PB6 = my_value.bitv.b2;  //flipper disable
    CO3_PB7 = my_value.bitv.b3;

}

//set momentary solenoids or sound raw
void solenoid_set(unsigned char value)
{
    bitv_t my_value;    
    
    my_value.byte = value;
    
    //set to solenoid mode, as we will need it
    //we do not check current status because of delay
    SOUNDSELECT = SOLENOIDS_SELECTED;
    //in 'solenoid' mode just pass data   
    //we hope that port settings is not to sequential
    //in order not to pulse wrongs solenoids
    //AND that Raspberry timing is accurate enough
    MOM_SOL_SOUND_DATA_A = my_value.bitv.b0;
    MOM_SOL_SOUND_DATA_B = my_value.bitv.b1;
    MOM_SOL_SOUND_DATA_C = my_value.bitv.b2;
    MOM_SOL_SOUND_DATA_D = my_value.bitv.b3;    
  }

//sound data standard soundboard
//implemented based on tarces on org Bally MPU by holli8123
void sound_data_standard(unsigned char value)
{
    bitv_t my_value;    
    uint8_t my_DATA_A1, my_DATA_B1, my_DATA_C1, my_DATA_D1;
    
    my_value.byte = value;    
    
    if (sound_raw)
    {   
    //in 'raw sound mode ' mode just pass data   
    MOM_SOL_SOUND_DATA_A = my_value.bitv.b0;
    MOM_SOL_SOUND_DATA_B = my_value.bitv.b1;
    MOM_SOL_SOUND_DATA_C = my_value.bitv.b2;
    MOM_SOL_SOUND_DATA_D = my_value.bitv.b3;            
                
    }
    else
    {   //cooked mode 
    //Prepare nybble1 to save time after di())
    my_DATA_A1 = my_value.bitv.b0;
    my_DATA_B1 = my_value.bitv.b1;
    my_DATA_C1 = my_value.bitv.b2;
    my_DATA_D1 = my_value.bitv.b3;    
    
    di(); // wen need exact timing, so no interrupts here
    //check if we are in soundmode already, if yes we need to go to solenoidmode
    //for at least 166us ?? RTH: what is the timing for standardmode?
    if (SOUNDSELECT == SOUNDS_SELECTED)
    {
      // set data back to rest address
      MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;   
      //and set to solneoid 
      SOUNDSELECT = SOLENOIDS_SELECTED;
     }       
    //we do the delay indepemdent of the current state
      __delay_us(166);             
    //in order to have an interrupt from soundboard go to sound mode
    SOUNDSELECT = SOUNDS_SELECTED; //this will set signal low->high = INT for SB
    //wait a bit ( Bally do wait 32us here )
    __delay_us(30);
    //now set sound data
    MOM_SOL_SOUND_DATA_A = my_DATA_A1;
    MOM_SOL_SOUND_DATA_B = my_DATA_B1;
    MOM_SOL_SOUND_DATA_C = my_DATA_C1;
    MOM_SOL_SOUND_DATA_D = my_DATA_D1;        
    // enable interrupts again, as further timing is in milliseconds range
    // so interrupts do not change much
    ei();     
    //let data stable for 18 milliseconds(33ms according to trace)
    //__delay_ms(18);         
    
    // NEW
    // instead use interruptable delay. In most cases, this will play the melody as planned, 
    //but when new commands arrive fast enough, this gets cut short.
    delay_18ms_interruptable();
    
    // and set data back to rest address
    MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;   
 }//cooked mode      
}

//sound data extended Sounboard, collect and send
void sound_data_extended(unsigned char value)
{
    bitv_t my_value;    
    static unsigned char is_first_nybble = 1;
    static unsigned char first_nybble;
    
    uint8_t my_DATA_A1, my_DATA_B1, my_DATA_C1, my_DATA_D1;
    uint8_t my_DATA_A2, my_DATA_B2, my_DATA_C2, my_DATA_D2;
                
    //first data? ijust store it
    if ( is_first_nybble)
    {
        is_first_nybble = 0;
        first_nybble = value;                
        return;
    }
    else //second nybble, ready to send
    {
      is_first_nybble = 1; //get ready for next pair of data
            
    //Prepare nybble1 to save time after di())
    my_value.byte = first_nybble;    
    my_DATA_A1 = my_value.bitv.b0;
    my_DATA_B1 = my_value.bitv.b1;
    my_DATA_C1 = my_value.bitv.b2;
    my_DATA_D1 = my_value.bitv.b3;    
    
    //Prepare nybble2  to save time after di())
    my_value.byte = value;    
    my_DATA_A2 = my_value.bitv.b0;
    my_DATA_B2 = my_value.bitv.b1;
    my_DATA_C2 = my_value.bitv.b2;
    my_DATA_D2 = my_value.bitv.b3;    
    
    //implemented according Bally FO-560-3
       di(); // wen need exact timing, so no interrupts here
    //check if we are in soundmode already, if yes we need to go to solenoidmode
    //for at least 40us for S&T ( -51SB: 62us))
    if (SOUNDSELECT == SOUNDS_SELECTED)
    {
      // set data back to rest address
      MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;   
      //and set to solneoid mode for ...
      SOUNDSELECT = SOLENOIDS_SELECTED;
    }
    //we do the delay indepemdent of the current state       
      __delay_us(40);   
      if (ext_soundboard_type == EXT_SB_IS_SAT) __delay_us(25); //extra time for 51SB

    //in order to have an interrupt from soundboard go to sound mode
    SOUNDSELECT = SOUNDS_SELECTED;  //this will set signal low->high = INT
    //wait 22 u seconds
    __delay_us(22);  
    //now set sound data nybble 1
    MOM_SOL_SOUND_DATA_A = my_DATA_A1;
    MOM_SOL_SOUND_DATA_B = my_DATA_B1;
    MOM_SOL_SOUND_DATA_C = my_DATA_C1;
    MOM_SOL_SOUND_DATA_D = my_DATA_D1;    
    //and let it stable for 145 u seconds for S&T ( -51SB: 118)
    __delay_us(118);              
    if (ext_soundboard_type == EXT_SB_IS_SAT) __delay_us(25); //extra time for S&T
    //now send second nybble
    MOM_SOL_SOUND_DATA_A = my_DATA_A2;
    MOM_SOL_SOUND_DATA_B = my_DATA_B2;
    MOM_SOL_SOUND_DATA_C = my_DATA_C2;
    MOM_SOL_SOUND_DATA_D = my_DATA_D2;       
    //and let it stable for 78 u seconds for S&T ( -51SB: 520u)
    __delay_us(78);      
    if (ext_soundboard_type == EXT_SB_IS_51) __delay_us(440); //extra time for 51SB       
    // enable interrupts again, as go back to rest(15) is not time critical
    ei();     
    // and set data back to rest address
    MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;   
    //wait a bit for ensure stable data
    __delay_us(5); 
    //and select solenoids at rest address
    SOUNDSELECT = SOLENOIDS_SELECTED; 
   }//send routine
}

/*
  read all 4 dip switches and
  set inernal vars for later use
 */
void get_dip_setting(void)
{
    
    //concurrent use of inhibit signal for lamp driver
    //so init address with 1

    //set strobe to high
    LAMPSTR1 = 1;    
    //inhibit (data) is all 1, we deactivate all decoders
    PD0 = PD1 = PD2 = PD3 = 1;  
    //set the rest address
    AD0 = AD1 = AD2 = AD3 = 1;    
    //set strobe to low, this will strobe address into the decoders
    LAMPSTR1 = 0;    
    //RTH: we may nee to do same for other lamp driver boards
    
    //all sond/solenoid data to rest address
    MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;        
    // select soundboard in order not to fire up solenoids here
    SOUNDSELECT = 1;
    // wait one ms, with rest data as SB would interprete that as sound cmd otherwise
    __delay_ms(1);        
    
    //no read dip switches
    //use STROBE1 which is DIP1..3 od SWitch1
    STROBE1 = 0;
    STROBE2 = STROBE3 = STROBE4 = STROBE5 = STROBE6 = STROBE7 = STROBE8 = 1;
    //let the value be stable
    __delay_ms(10);    
    dip1.bitv.b0 = ~RETURN1;
    dip1.bitv.b1 = ~RETURN2;
    dip1.bitv.b2 = ~RETURN3;
    dip1.bitv.b3 = ~RETURN4;    
    //use STROBE2 which is DIP4..7 od Switch1
    STROBE1 = 1; STROBE2 = 0;
    //let the value be stable
    __delay_ms(10);    
    dip1.bitv.b4 = ~RETURN1;
    dip1.bitv.b5 = ~RETURN2;
    dip1.bitv.b6 = ~RETURN3;
    dip1.bitv.b7 = ~RETURN4;

    //use STROBE3 which is DIP1..3 od SWitch2
    STROBE2 = 1; STROBE3 = 0;    
    //let the value be stable
    __delay_ms(10);    
    dip2.bitv.b0 = ~RETURN1;
    dip2.bitv.b1 = ~RETURN2;
    dip2.bitv.b2 = ~RETURN3;
    dip2.bitv.b3 = ~RETURN4;    
    //use STROBE4 which is DIP4..7 od Switch2
    STROBE3 = 1; STROBE4 = 0;
    //let the value be stable
    __delay_ms(10);    
    dip2.bitv.b4 = ~RETURN1;
    dip2.bitv.b5 = ~RETURN2;
    dip2.bitv.b6 = ~RETURN3;
    dip2.bitv.b7 = ~RETURN4;
    
    //use STROBE5 which is DIP1..3 od SWitch3
    STROBE4 = 1; STROBE5 = 0;    
    //let the value be stable
    __delay_ms(10);    
    dip3.bitv.b0 = ~RETURN1;
    dip3.bitv.b1 = ~RETURN2;
    dip3.bitv.b2 = ~RETURN3;
    dip3.bitv.b3 = ~RETURN4;    
    //use STROBE6 which is DIP4..7 od Switch3
    STROBE5 = 1; STROBE6 = 0;
    //let the value be stable
    __delay_ms(10);    
    dip3.bitv.b4 = ~RETURN1;
    dip3.bitv.b5 = ~RETURN2;
    dip3.bitv.b6 = ~RETURN3;
    dip3.bitv.b7 = ~RETURN4;

    //use STROBE7 which is DIP1..3 od SWitch4
    STROBE6 = 1; STROBE7 = 0;    
    //let the value be stable
    __delay_ms(10);    
    dip4.bitv.b0 = ~RETURN1;
    dip4.bitv.b1 = ~RETURN2;
    dip4.bitv.b2 = ~RETURN3;
    dip4.bitv.b3 = ~RETURN4;    
    //use STROBE8 which is DIP4..7 od Switch4
    STROBE7 = 1; STROBE8 = 0;
    //let the value be stable
    __delay_ms(10);    
    dip4.bitv.b4 = ~RETURN1;
    dip4.bitv.b5 = ~RETURN2;
    dip4.bitv.b6 = ~RETURN3;
    dip4.bitv.b7 = ~RETURN4;
    
    // back to default
    //Momentary solenoid all 1, which will select '15' - 'open'
    MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;        
    //we select initially the solenoid bank 
    SOUNDSELECT = 0;                
}

/*
 * Main
 */
void main(void) {
        
union four {
    unsigned char byte;
    struct {
    unsigned LAMP:6, ACTION:1, IS_CMD:1;
    //signed b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv;     
    struct {
    unsigned COMMAND:3, COILS:4, IS_CMD:1;
    //signed b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv2;  
    } mydata;


//Clock Source and Value: Internal 16MHz
OSCCONbits.IRCF0 = 1; 
OSCCONbits.IRCF1 = 1;
OSCCONbits.IRCF2 = 1;
//Device is running from the internal oscillator
OSCCONbits.OSTS = 0;

//Primary clock (determined by FOSC<3:0> in CONFIG1H).
//OSCCONbits.SCS = 0;
OSCCONbits.SCS = 3; //RTH: 64MHz still need to be tested
//Internal oscillator block "OSCCONbits.SCS = 3;"does not work with PLL
//OSCTUNEbits.PLLEN = 1;      // turn on the PLL 16*4
OSCTUNEbits.PLLEN = 0;      // turn OFF the PLL
// Wait until PLL is ready
//while(!OSCCON2bits.PLLRDY); 

/* Port init */
// Port A
ANSELA = 0;   //All I/O pins are configured as digital
ADCON1=0x6;   // PortA Digital
TRISA = 0b00000100;    // LA2 initial input, all others output
// Port B
ANSELB = 0;   //All I/O pins are configured as digital
TRISB = 0b00001011;    // PORTB All Output except RB0 & RB3 for K3 & RB1 for INT1
//Port C
ANSELC = 0;   //All I/O pins are configured as digital
TRISC = 0b01111111; // PORTC all Input except LC7 (Lampstrobe 1))
//Port D
ANSELD = 0;   //All I/O pins are configured as digital
TRISD = 0; //all output
//Port E
ANSELE = 0;   //All I/O pins are configured as digital
TRISE = 0b1001; // E3 is input, E0 is initial input, others output

/* I2C */        
// Setup MSSP in 7 bit I2C Slave mode
    SSPADD         = I2C_ADDR;      // Set I2C address
    SSPCON1        = 0x36;          // SSPEN: Synchronous Serial Port Enable bit - Enables the serial port and configures the SDA and SCL pins as the serial port pins
                                    // CKP: SCK Release Control bit              - Release clock
                                    // SSPM3:SSPM0: SSP Mode Select bits         - 0110 = I2C Slave mode, 7-bit address    
    SSPSTAT        = 0x00;    
    SSPCON2        = 0x01;          // GCEN: General Call address (00h) (Slave mode only) 0 = General call address disabled
                                    // SEN: Start Condition Enable/Stretch Enable bit(1) ()Slave mode) 1 = Clock stretching is enabled 
    PIR1bits.SSPIF = 0;             // Clear MSSP interrupt request flag
    PIE1bits.SSPIE = 1;             // Enable MSSP interrupt enable bit
    INTCONbits.GIE_GIEH  = 1;       // GIE/GIEH: Global Interrupt Enable bit
    INTCONbits.PEIE_GIEL = 1;       // PEIE/GIEL: Peripheral Interrupt Enable bit

    //Init High/Low Interrupts    
    IPR1bits.SSP1IP = 1;        //MSSP high prio interrupt
    RCONbits.IPEN 		= 1;	//Enable Interrupt Priorities
    INTCONbits.GIEL 	= 1; 	//Enable Low Priority Interrupt	
    INTCONbits.GIE 		= 1; 	//Enable GlobalInterrupt			  
    INTCONbits.TMR0IE 	= 1;	//Enable Timer0 Interrupt
    INTCON2bits.TMR0IP	= 0;	//TMR0 set to Low Priority Interrupt
    
    // Timer interrupt init TIMER0
    // 16Mhz Takt -> ergibt 16/4 gleich 4 MHz fuer Timer0
    // entspricht 0,25us oder 250ns pro cycle/takt
    // prescaler 64 ergibt 16us per TMR0 Value
    // 2000us / 16us = 125 -> 256 - 125 = 131 als TMR0_VALUE    
    TMR0 = TMR0_VALUE;   //see def.h    
    //will be anabled in ext interrupt routine
    T0CONbits.TMR0ON = 1; //enable Timer 0
    T0CONbits.T08BIT = 1; //8bit timer
    T0CONbits.T0CS = 0;  //Internal instruction cycle clock (CLKOUT)
    T0CONbits.PSA = 0;  //Timer0 prescaler is assigned
    T0CONbits.T0PS = 0b101; // 1:64 prescale value
    // now enable Interrupts 
    INTCONbits.T0IE=1; // enable Timer0 interrupt 
    INTCONbits.T0IF=0;  //clear flag

//internal pull up for k3 which is RB0 & RB3
INTCON2bits.RBPU = 0;
WPUB = 0b00001001;

/*External interrupts on the RB0/INT0, RB1/INT1 and
RB2/INT2 pins are edge-triggered. If the corresponding
INTEDGx bit in the INTCON2 register is set (= 1), the
interrupt is triggered by a rising edge; if the bit is clear,
the trigger is on the falling edge. When a valid edge
appears on the RBx/INTx pin, the corresponding flag
bit, INTxF, is set. This interrupt can be disabled by
clearing the corresponding enable bit, INTxE. Flag bit,
INTxF, must be cleared by software in the Interrupt
Service Routine before re-enabling the interrupt.*/
INTCON2bits.INTEDG1 = 0; //INT1 on falling edge
INTCON3bits.INT1IP = 0, //INT1 External Interrupt Priority bit, low prio
INTCON3bits.INT1IF = 0;  //INT1 External Interrupt Flag bit cleared
INTCON3bits.INT1IE = 1; //INT1 External Interrupt Enable bit

// initialize timer2 for the interruptable delay
TMR2_Initialize();

//local vars
bitv_t k3;
int i;
uint8_t eeprom_address, eeprom_data;
uint8_t eeprom_read_mode = 0;
uint8_t eeprom_write_mode = 0;
uint8_t current_dip_switch = 1; 

/* init */
//all inhibits one
for( i=0; i<=15; i++) inhibit[i].byte = inhibit2[i].byte = 0xff;

//Momentary solenoid all 1, which will select '15' - 'open'
MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;        
//we select initially the solenoid bank (0))
SOUNDSELECT = SOLENOIDS_SELECTED;        
//lampstrobes are zero
LAMPSTR1 = LAMPSTR2 = 0;
//continous solenoid all off ( activ low))
CO2_PB4 = CO4_PB5 = CO1_PB6 = CO3_PB7 = 1;
//Lamp Address data  all 1, which will select '15' - 'open'
AD0 = AD1 = AD2 = AD3 = 1;
//Lamp data '15' means all decoders on inhibit
PD0 = PD1 = PD2 = PD3 = 1;
//LEDs
LED = 1;  //red LED for PIC activity
GREENLED = 0; //green LED (Bally org))
ZERO_CR_LED = 1; //Zerocross LED initial on
//
//the dip settings pinmame default
dip1.byte = dip2.byte = dip4.byte =0;
dip3.byte = 0x18;
                
//read content of eeprom into buffer
for( i=0; i<=255; i++) 
    {
     eeprom_content_current[i] = readEEPROM(i);
     eeprom_content_towrite[i] = eeprom_content_current[i];
    }

k3.byte = 0;

//initial read of mpu dip switches
di();
get_dip_setting();                                          
//enable interrupts
ei();

while( 1 ) { //Loop forever
    
//check if we have a byte in the Buffer
//and act accordently
if ( BufferOut(&mydata.byte) == BUFFER_SUCCESS )
{             
    //handle possible eeprom read/write 
    if (( eeprom_read_mode > 0 ) || ( eeprom_write_mode > 0 ) )
    {
        //state 1 of read mode, get address and write to buffer
        if ( eeprom_read_mode == 1) 
        {            
            eeprom_address = mydata.byte;
            data_slave = eeprom_content_current[eeprom_address];            
            eeprom_read_mode = 0;     
        }//if eeprom read mode
        
        //state 1 of write mode, get address
        if ( eeprom_write_mode > 0 )        
        {    
            //state 1 of write mode, get address
            if ( eeprom_write_mode == 1) 
            {
                eeprom_address = mydata.byte;
                eeprom_write_mode = 2;
            }        
        //state 2 of write mode, get data and write to buffer
            else if ( eeprom_write_mode == 2) 
            {                
                eeprom_data = mydata.byte;
                eeprom_content_towrite[eeprom_address] = eeprom_data;                
                eeprom_write_mode = 0;
            }                
        } //eeprom write_mode
    }//if eeprom mode
    else
    {        
         //is this an command? 
         if ( mydata.bitv.IS_CMD == 1 )            
            {                          
              switch(mydata.bitv2.COMMAND) { 
                      case LS80COILCMD_EXT_CMD_ID:    //ext cmd with ver 4
                            switch(mydata.bitv2.COILS) {             
                                 case LISY_EXT_CMD_EEPROM_READ:
                                    eeprom_read_mode = 1;                     
                                    break;
                                 case LISY_EXT_CMD_EEPROM_WRITE:
                                    eeprom_write_mode = 1;                     
                                    break;             
                                case LISY35_EXT_CMD_AUX_BOARD_0:  // no aux driverboard
                                    lampdriver_variant = NO_AUX_BOARD;
                                    break;                                                                          
                                case LISY35_EXT_CMD_AUX_BOARD_1:  // AS-2518-43 12 lamps
                                    lampdriver_variant = AS_2518_43_12_LAMPS;
                                    break;                                                                          
                                case LISY35_EXT_CMD_AUX_BOARD_2:  // AS-2518-52 28 lamps
                                    lampdriver_variant = AS_2518_52_28_LAMPS;
                                    break;                                                                          
                                case LISY35_EXT_CMD_AUX_BOARD_3:  // AS-2518-23 60 lamps
                                    lampdriver_variant = AS_2518_23_60_LAMPS;
                                    break;       
                                case LISY35_EXT_CMD_SB_IS_51: //Soundboard is a 2581-51 (extended mode)
                                    ext_soundboard_type = EXT_SB_IS_51;
                                    break;                                        
                                case LISY35_EXT_CMD_SB_IS_SAT:  //Soundboard is a S&T (extended mode)
                                    ext_soundboard_type = EXT_SB_IS_SAT;
                                    break;                                                                                                                
                                case LISY35_EXT_CMD_J4PIN5_INPUT:  // CO2_PB4 LATEbits.LE0   cont Sol 1
                                    TRISEbits.RE0 = 1;
                                    break;    
                                case LISY35_EXT_CMD_J4PIN5_OUTPUT:  // CO2_PB4 LATEbits.LE0   cont Sol 1
                                    TRISEbits.RE0 = 0;
                                    break;        
                                case LISY35_EXT_CMD_J4PIN8_INPUT:  //CO3_PB7 LATAbits.LA2   cont Sol 4
                                    TRISAbits.RA2 = 1;
                                    break;                                                                                                                                                  
                                case LISY35_EXT_CMD_J4PIN8_OUTPUT:  //CO3_PB7 LATAbits.LA2   cont Sol 4
                                    TRISAbits.RA2 = 0;
                                    break;                                                                                                                                                  
                                case LISY35_EXT_CMD_J1PIN8_INPUT:  //LAMPSTR2 LATCbits.LC6  Lamp Strobe 2
                                    TRISCbits.RC6 = 1;
                                    break;                                                                                                                                                  
                                case LISY35_EXT_CMD_J1PIN8_OUTPUT:  //LAMPSTR2 LATCbits.LC6  Lamp Strobe 2
                                    TRISCbits.RC6 = 0;
                                    break;          
                                case LISY35_READ_DIP_SWITCHES:  //re read dip switches and buffer value
                                    di();
                                    get_dip_setting();                                          
                                    ei();
                                    LED = ~LED;
                                    break;                                                                                                                                                                                                                                                              
                                case LISY35_GET_DIP_SWITCHES:  //return dip dip switches 1...4                                    
                                    switch(current_dip_switch)
                                    {
                                        case 1: data_slave = dip1.byte;
                                            break;
                                        case 2: data_slave= dip2.byte;
                                            break;
                                        case 3: data_slave= dip3.byte;
                                            break;
                                        case 4: data_slave= dip4.byte;
                                            break;                                                                                                                        
                                    }
                                    //read all switches already?
                                    if ( ++current_dip_switch > 4)current_dip_switch = 1 ;
                                    LED = ~LED;
                                    break;                                                                                                                                                                                                                          
                                    }                                                                             
                                break;                         
                      case LS80COILCMD_GET_SW_VERSION_MAIN: //send SW version                            
                                data_slave=lisy35_coil_version / 100; //main version number               
                                break;            
                      case LS80COILCMD_GET_SW_VERSION_SUB: //send SW version                 
                                data_slave=lisy35_coil_version % 100; //sub version number
                                break;            
                      case LS80COILCMD_GET_K3:         //send value of K3                 
                                k3.bitv.b0 = NOINTROVOICE;
                                k3.bitv.b1 = NOVOLOPT;
                                data_slave = k3.byte;
                                break;           
                      case LS35COIL_CONT_SOL: //continous solenoid
                                cont_sol_set(mydata.bitv2.COILS);
                                LED = ~LED;
                                break;            
                      case LS35COIL_MOM_SOL: //momentary solenoids
                                solenoid_set(mydata.bitv2.COILS);
                                LED = ~LED;
                                break;
                      case LISY35_STANDARD_SOUND: //sound data standard SB
                                sound_data_standard(mydata.bitv2.COILS);
                                LED = ~LED;                                
                                break;
                      case LISY35_EXTENDED_SOUND: //sound data extended SB
                                sound_data_extended(mydata.bitv2.COILS);
                                LED = ~LED;                                                                
                                break;                                    
                      }//switch Command_byte                     
                }
            else //lamp set mode 
               {
                   //check for 'special' lamps
                   switch (mydata.bitv.LAMP)
                   {
                       case LISY35_COIL_GREEN_LED:
                           GREENLED = mydata.bitv.ACTION;
                           break;
                       case LISY35_COIL_SOUNDSELECT:
                           SOUNDSELECT = mydata.bitv.ACTION;
                           break;
                       case LISY35_COIL_LAMPBOARD:
                           lampboard_active = mydata.bitv.ACTION;
                           break;                           
                       case LISY35_COIL_SOUNDRAW:
                           sound_raw = mydata.bitv.ACTION;
                           break;                                                      
                       default:
                           lamp_set(mydata.bitv.LAMP, mydata.bitv.ACTION);              
                           break;
                   }                         
                   LED = ~LED;
               }                                           
     } // if Bufferstatus  
   } //if not eeprom
 } //while 1, endless loop
} //main

#pragma warning pop
