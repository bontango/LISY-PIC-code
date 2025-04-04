/* 
 * File:   main.c
 * Author: th
 * Ballypinball tester
 * PIC 18F45K22
 * READ Port / Write LAT
 * 
 * v 0.4 changed lamptest
 * v 0.5 added AS-2518-147 lamp combo board
 * v 0.6 sound corrections
 * v 0.7 soundtest with t=0 one time
 */

//no warnings about 
#pragma warning push
#pragma warning disable 359 //illegal conversion between pointer types
#pragma warning disable 373 //'implicite signed to unsigned conversion'

#include "def.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "lcd_hd44780.h"

#define VERSION "0.7"

//typedefs
typedef union typed {
    unsigned char byte;
    struct {    
    unsigned b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv;
    } bitv_t;
    
//switches
#define SW_OK 1
#define SW_UP 3
#define SW_DOWN 4
#define SW_LEFT 2
#define SW_RIGHT 5

    //other
#define SOLENOIDS_SELECTED 0
#define SOUNDS_SELECTED 1
#define EXT_SB_IS_51 0  //Soundboard is a 2581-51 (extended mode)
#define EXT_SB_IS_SAT 1  //Soundboard is a S&T (extended mode)

//the aux lampdriverboards variants
#define NO_AUX_BOARD 0
#define AS_2518_43_12_LAMPS 1
#define AS_2518_52_28_LAMPS 2
#define AS_2518_23_60_LAMPS 3
#define AS_2518_147_60_LAMPS 4    
    

//timer0
#define TMR0_VALUE  131         //interrupt after 2 ms

//function prototypes
void refresh_display(void);
uint8_t check_OK_sw(void);

//globals
volatile bitv_t inhibit[16];  //inhibit bits for board0
volatile bitv_t inhibit2[16];  //inhibit bits for board1

volatile uint8_t lampboard_active = 0;
volatile uint8_t lampdriver_variant = NO_AUX_BOARD;
volatile uint8_t ext_soundboard_type = EXT_SB_IS_SAT; //default S&T
volatile uint8_t ZC_phase = 0; //incase of comboboard 0=phaseA 1=phaseB

char line[30];
uint8_t cur_menu = 0; //current selected menu
uint8_t cur_line = 0; //current selected line in menu
uint8_t cur_option[3] = { 0, 0, 0 }; //current option in selection
int cur_pulse = 150; //pulsetime in msec
int cur_sound_t = 1; //time n seconds to go to next sound i  test
int cur_sol_t = 1; //time n seconds to go to next solenoid i  test
int cur_lamp_t = 1; //time n seconds to go to next device i  test
uint8_t cur_mode = 0;

/*const char * mode[] = {
        "blAll",
        "blHal",
        "serie"        
    };
  */

//menu entries
const char * main_menu[] = {
    "Test Lamps",
    "Test Sound",    
    "Test Solenoids"
    };
uint8_t n_main_menu = 3;    

    const char * lamp_option[] = {
        "1st AS-2518-23(60)",
        "2nd AS-2518-43(12)",
        "2nd AS-2518-52(24)",
        "2nd AS-2518-23(60)",
		"Co AS-2518-147(60)"      
		
    };

    const char * sound_option[] = {
        "Chimes",
        "AS-2518-32",
        "AS-2518-50",
        "AS-2518-51",
        "AS-2518-56 (So+)",
        "AS-2518-57 (Voc)",
        "AS-2518-61 (S&T)",
        "Say It Again",
        "Cheap Squeak"        
    };

  const char * solenoid_option[] = {
        "Momentary Solenoids",
        "Continous Solenoids"
    };

  //number of options fior the menus lamp/sound/solenoid/
  uint8_t n_option[3] = { 5, 9, 2 };
  
  //default values from to for the 3 main menues
  int select_from[3] = { 1, 1, 1 };
  int select_to[3] = { 60, 31, 15 };
  
  
    
 char line3[21], line4[21];
                 
//globals
unsigned char switchstatus[10]={0,0,0,0,0,0,0,0,0,0}; //status of the 9 switches

//util sleep n * 100ms
//while checking OK_SW
//breaks & return 1 if OK SW was pressed
//return 0 at end of sleep time
uint8_t sleep_n_100ms( int n)
{
    int i;
    for (i=0; i<n; i++)
    {
        __delay_ms(100);
        //OK switch pressed?
        if ( check_OK_sw() == 1 ) return 1;
    }
    
    return 0;
        
}

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
    //which is either a 60-lamps "AS_2518_23"
    //or a comboboard with 60 lamp and phasA & phaseB    
   if ( lampdriver_variant !=  AS_2518_147_60_LAMPS) //always when no comboboard
   {
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
    //add extra delay give signals stable time before strobe    
    __delay_us(8);  //we add 3us->8us (without it was 5us) Bally has 15us
    //set strobe to low, this will strobe address into the decoders
    LAMPSTR1 = 0;    
    //set the rigth inhibit for this address    
    PD0 = inhibit[address.byte].bitv.b4;
    PD1 = inhibit[address.byte].bitv.b5;
    PD2 = inhibit[address.byte].bitv.b6;
    PD3 = inhibit[address.byte].bitv.b7;    
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
   }   
   else //this is a COMBOBOARD two phases
   {    
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
    //add extra delay give signals stable time before strobe    
    __delay_us(8);  //we add 3us->8us (without it was 5us) Bally has 15us
    //set strobe to low, this will strobe address into the decoders
    LAMPSTR1 = 0;    
    //set the rigth inhibit for this address
    //comboboard has only PD" & PD3 but two phases
    if (ZC_phase == 0) //phase A
    {
        PD2 = inhibit[address.byte].bitv.b4;
        PD3 = inhibit[address.byte].bitv.b5;                
    }
    else //phase B
    {
        PD2 = inhibit[address.byte].bitv.b6;
        PD3 = inhibit[address.byte].bitv.b7;                    
    }                        
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
   } 
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
        //add extra delay to give signals stable time before strobe    
        __delay_us(8);  //we add 3us->8us (without it was 5us) Bally has 15us        
        //set strobe to high, this will push strobe address to the decoders
        LAMPSTR2 = 1;    
        //set the rigth inhibit for this address which will activate decoder or not
        PD0 = inhibit[address.byte].bitv.b4;
        PD1 = inhibit[address.byte].bitv.b5;
        PD2 = inhibit[address.byte].bitv.b6;
        PD3 = inhibit[address.byte].bitv.b7;    
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
        //add extra delay to give signals stable time before strobe    
        __delay_us(8);  //we add 3us->8us (without it was 5us) Bally has 15us        
        //set strobe to high, this will push strobe address to the decoders
        LAMPSTR2 = 1;    
        //set the rigth inhibit for this address which will activate decoder or not
        PD0 = inhibit[address.byte].bitv.b4;
        PD1 = inhibit[address.byte].bitv.b5;
        PD2 = inhibit[address.byte].bitv.b6;
        PD3 = inhibit[address.byte].bitv.b7;    
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
            //add extra delay to give signals stable time before strobe    
            __delay_us(8);  //we add 3us->8us (without it was 5us) Bally has 15us            
            //set strobe to low, this will strobe address into the decoders
            LAMPSTR2 = 0;    
            //set the rigth inhibit for this address
            PD0 = inhibit[address.byte].bitv.b4;
            PD1 = inhibit[address.byte].bitv.b5;
            PD2 = inhibit[address.byte].bitv.b6;
            PD3 = inhibit[address.byte].bitv.b7;    
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
/*
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
*/

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
   //do_eeprom_write();             
  
  INTCONbits.T0IF = 0;  // clear this interrupt condition
  }    
  else if (INTCON3bits.INT1IE && INTCON3bits.INT1IF) 
  {
      
   // external INT/RB1 interupt      
      
   
      
   // we cannot start the refresh yet
   // so lets start the timer and do it there   
      TMR0 = TMR0_VALUE;    //set TMR0 to old value
      T0CONbits.TMR0ON = 1; //enable Timer 0           
      
   //determine phase in case of comboboard
   //and change edge sensitivity if needed   
   if ( lampdriver_variant ==  AS_2518_147_60_LAMPS)
   {
       
     if ( INTCON2bits.INTEDG1 == 0)    //INT1 on falling edge    
     {
         ZC_phase = 0; //current phase A
         INTCON2bits.INTEDG1 = 1; //rising edge for next int                  
     }     
     else
     {
         ZC_phase = 1; //current phase B
         INTCON2bits.INTEDG1 = 0; //falling edge for next int                                    
     }                
   }    
   else  INTCON2bits.INTEDG1 = 0;    //INT1 on falling edge    
       
       
    INTCON3bits.INT1IF = 0; // clear this interrupt condition
  }     
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
    uint8_t my_DATA_A1, my_DATA_B1, my_DATA_C1, my_DATA_D1, my_DATA_E1;
    
    my_value.byte = value;    
    
    //Prepare nybble1 to save time after di())
    my_DATA_A1 = my_value.bitv.b0;
    my_DATA_B1 = my_value.bitv.b1;
    my_DATA_C1 = my_value.bitv.b2;
    my_DATA_D1 = my_value.bitv.b3;    
    my_DATA_E1 = my_value.bitv.b4;    
    
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
    SOUND_DATA_E = my_DATA_E1;      
    // enable interrupts again, as further timing is in milliseconds range
    // so interrupts do not change much
    ei();     
    //let data stable for 18 milliseconds(33ms according to trace)
    __delay_ms(18);          
    // and set data back to rest address
    MOM_SOL_SOUND_DATA_A = MOM_SOL_SOUND_DATA_B = MOM_SOL_SOUND_DATA_C = MOM_SOL_SOUND_DATA_D = 1;   

}


//debounce
//check if status changed and was stable for 100ms
//if yes assign new switchstatus
//give back 1 if it changed from 0 to 0
//give back 0 otherwise

unsigned char debounce(unsigned char sw, unsigned char status)
{
    if (status != switchstatus[sw])
    {
        __delay_ms(100);    //wait debounce time
        //still changed?
        if (status != switchstatus[sw])
        {
            switchstatus[sw] = status;
            return(status);            
        }        
    }
    
    return 0;
}

//check status of 'OK' (S1) switch
//no blocking
// 1== closed; 0==open
uint8_t check_OK_sw(void)
{        
    
    unsigned char status;    
            
    STROBE_1 = 1; STROBE_2=0; STROBE_3=0; 
    __delay_ms(10);    
    //S1 == RET1
    status = RETURN_1;
    
    //strobeall zero
    STROBE_1 = 0; STROBE_2=0; STROBE_3=0; 

    return(status);
 }

//check all switches and give back the number
//of the switch which went from open to close
//blocking mode
// 1== closed; 0==open
unsigned char check_sw(void)
{        
    
    unsigned char status;    
        
    while(1) //wait for switch change
    {    
    //scan all switches, no debounce
    STROBE_1 = 1; STROBE_2=0; STROBE_3=0; 
    __delay_ms(10);    
    //S1 == RET1
    status = RETURN_1;
    if ( debounce(1, status) ) return(1);    
    //S2 == RET2
    status = RETURN_2;
    if ( debounce(2, status) ) return(2);            
    //S5 == RET3
    status = RETURN_3;
    if ( debounce(5, status) ) return(5);    
    
    STROBE_1 = 0; STROBE_2=1; STROBE_3=0; 
    __delay_ms(10);    
    //S3 == RET1
    status = RETURN_1;
    if ( debounce(3, status) ) return(3);    
    //S4 == RET2
    status = RETURN_2;
    if ( debounce(4, status) ) return(4);    
            
    //strobeall zero
    STROBE_1 = 0; STROBE_2=0; STROBE_3=0; 
    }
    
    return 0; //never reached
}

//dip switch TBD
/*
     //S6/Dip1 == RET3
    status = RETURN_3;
    if ( debounce(6, status) ) return(6);        
    STROBE_1 = 0; STROBE_2=0; STROBE_3=1; 
    __delay_ms(10);        
    //S6/Dip2 == RET1
    status = RETURN_1;
    if ( debounce(7, status) ) return(7);    
    
    switchstatus[7] = RETURN_1;
    //S6/Dip3 == RET2
    switchstatus[8] = RETURN_2;
    //S6/Dip4 == RET3
    switchstatus[9] = RETURN_3;

 */    


//refresh display with menu 1..3
void refresh_display( void )
{
    char *entry1,*entry2,*entry3,*entry4;
    
    entry1 = main_menu[cur_menu];
    
    //construct line 3
    sprintf(line3,"from:0x%02x to:0x%02x",select_from[cur_menu], select_to[cur_menu]);
    entry3 = line3;
       
    //construct line 3
    entry4 = line4;
    //  char line4[] = "mode:---- t:1.0sec";
    // char line4[] = "pls:150ms t:1.0sec";

    
    switch(cur_menu)
    {
        case 0: //lamps
            entry2 = lamp_option[cur_option[cur_menu]];
            //sprintf(line4,"mode:%s t:%d sec",mode[cur_mode],cur_t);
            sprintf(line4,"t:0.%d sec",cur_lamp_t);
            entry4 = line4;
            break;
        case 1: //sound
            entry2 = sound_option[cur_option[cur_menu]];            
            sprintf(line4,"t:%d sec",cur_sound_t);
            entry4 = line4;            
            break;
        case 2: //solenoid
            entry2 = solenoid_option[cur_option[cur_menu]];                        
            //sprintf(line4,"puls:%3dms t:%d sec",cur_pulse,cur_t);
            sprintf(line4,"t:%d sec",cur_sol_t);
            entry4 = line4;                        
            break;            
    }
        
//display each line with '> ' heading    
LCD_CMD(LCD_CMD_CLR);
sprintf(line,"> %s",entry1);
LCD_GotoXY(0,0);
LCD_STRING(line);
sprintf(line,"> %s",entry2);
LCD_GotoXY(0,1);
LCD_STRING(line);
sprintf(line,"> %s",entry3);
LCD_GotoXY(0,2);
LCD_STRING(line);
sprintf(line,"> %s",entry4);
LCD_GotoXY(0,3);
LCD_STRING(line);

//cursor position
LCD_GotoXY(0,cur_line);
    
}

/*-----------------------------------
line 3 make selection
from - to
-------------------------------------*/
void select_menu(void)
{
    uint8_t my_switch;
    uint8_t pos[4] = { 9, 10, 17, 18 };
    uint8_t current_pos = 0;

    LCD_GotoXY(pos[current_pos],2);
    
  do
  {
   my_switch = check_sw();
   switch ( my_switch )
   {
       case SW_LEFT:
           if (current_pos > 0) --current_pos;
           else my_switch = SW_OK; //exit to the left
           break;
       case SW_RIGHT:
           if (current_pos < 3) ++current_pos;
           else current_pos = 0; //wrap           
           break;              
       case SW_UP:           
           switch(current_pos)
           {
               case 0:
                   if (( select_from[cur_menu] + 0x10) <= 0xFF) select_from[cur_menu] += 0x10;
                   if (select_from[cur_menu] > select_to[cur_menu]) select_to[cur_menu] = select_from[cur_menu];
                   break;
               case 1:
                   if (( select_from[cur_menu] + 1) <= 0xFF) select_from[cur_menu] += 1;
                   if (select_from[cur_menu] > select_to[cur_menu]) select_to[cur_menu] = select_from[cur_menu];
                   break;
               case 2:
                   if (( select_to[cur_menu] + 0x10) <= 0xFF) select_to[cur_menu] += 0x10;                   
                   break;
               case 3:
                   if (( select_to[cur_menu] + 1) <= 0xFF) select_to[cur_menu] += 1;                   
                   break;               
           }
           break;           
       case SW_DOWN:           
           switch(current_pos)
           {
               case 0:
                   if (( select_from[cur_menu] - 0x10) >= 0) select_from[cur_menu] -= 0x10;
                   break;
               case 1:
                   if (( select_from[cur_menu] - 1) >= 0) select_from[cur_menu] -= 1;
                   break;
               case 2:
                   if (( select_to[cur_menu] - 0x10) >= 0) select_to[cur_menu] -= 0x10;                   
                   if (select_to[cur_menu] < select_from[cur_menu]) select_from[cur_menu] = select_to[cur_menu];                   
                   break;
               case 3:
                   if (( select_to[cur_menu] - 1) >= 0) select_to[cur_menu] -= 1;                   
                   if (select_to[cur_menu] < select_from[cur_menu]) select_from[cur_menu] = select_to[cur_menu];                                      
                   break;               
           }           
           break;                      
   }     
      refresh_display();
      LCD_GotoXY(pos[current_pos],2);
  } while (my_switch != SW_OK);            
    
      LCD_GotoXY(0,2);
}



/* -----------------------------------
line 4
select mode
--------------------------------------*/
void mode_lamp_menu(void)
{
    
    uint8_t my_switch;
      
    LCD_GotoXY(4,3);
          
  do
  {
   my_switch = check_sw();
   switch ( my_switch )
   {
       case SW_LEFT:           
             my_switch = SW_OK; //exit to the left           
           break;
       case SW_RIGHT:
           break;              
       case SW_UP:           
           if (cur_lamp_t <= 8) ++cur_lamp_t;
           break;           
       case SW_DOWN:           
           if (cur_lamp_t >= 1) --cur_lamp_t;           
           break;                      
   }     
      refresh_display();
  } while (my_switch != SW_OK);            
    
      LCD_GotoXY(0,3);
}

void mode_sound_menu(void)
{
    
    uint8_t my_switch;
      
    LCD_GotoXY(4,3);
          
  do
  {
   my_switch = check_sw();
   switch ( my_switch )
   {
       case SW_LEFT:           
             my_switch = SW_OK; //exit to the left           
           break;
       case SW_RIGHT:
           break;              
       case SW_UP:           
           if (cur_sound_t <= 8) ++cur_sound_t;
           break;           
       case SW_DOWN:           
           if (cur_sound_t >= 1) --cur_sound_t;           
           break;                      
   }     
      refresh_display();
  } while (my_switch != SW_OK);            
    
      LCD_GotoXY(0,3);
}

void mode_solenoid_menu(void)
{
    
    uint8_t my_switch;
      
    LCD_GotoXY(4,3);
          
  do
  {
   my_switch = check_sw();
   switch ( my_switch )
   {
       case SW_LEFT:           
             my_switch = SW_OK; //exit to the left           
           break;
       case SW_RIGHT:
           break;              
       case SW_UP:           
           if (cur_sol_t <= 8) ++cur_sol_t;
           break;           
       case SW_DOWN:           
           if (cur_sol_t >= 1) --cur_sol_t;           
           break;                      
   }     
      refresh_display();
  } while (my_switch != SW_OK);            
    
      LCD_GotoXY(0,3);
}
//line 4 select mode
void mode_menu(void)
{
    switch(cur_menu)
    {
        case 0:
            mode_lamp_menu();
            break;
        case 1:
            mode_sound_menu();
            break;
        case 2:
            mode_solenoid_menu();
            break;
    }
    
}

/*----- END Menu entry */


//sound data extended Sounboard
void sound_data_extended(unsigned char value)
{
    bitv_t my_value;    
    
    uint8_t my_DATA_A1, my_DATA_B1, my_DATA_C1, my_DATA_D1;
    uint8_t my_DATA_A2, my_DATA_B2, my_DATA_C2, my_DATA_D2;
                
    //Prepare nybbles to save time after di())
    my_value.byte = value;    
    my_DATA_A1 = my_value.bitv.b0;
    my_DATA_B1 = my_value.bitv.b1;
    my_DATA_C1 = my_value.bitv.b2;
    my_DATA_D1 = my_value.bitv.b3;        
    //Prepare nybble2  to save time after di())
    my_DATA_A2 = my_value.bitv.b4;
    my_DATA_B2 = my_value.bitv.b5;
    my_DATA_C2 = my_value.bitv.b6;
    my_DATA_D2 = my_value.bitv.b7;    
    
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

}

void do_lamp_test(void)
{
        int i;
    
    LCD_CMD(LCD_CMD_CLR); // Display loeschen
    LCD_CMD(LCD_CMD_CUR_BLINK);  //cursor blinkend an
    
    if (cur_lamp_t > 0 )
    {
    LCD_GotoXY(0,0);
    LCD_STRING(" Lamp test running ");   
    LCD_GotoXY(0,1);
    LCD_STRING(" press&hold OK      ");   
    LCD_GotoXY(0,2);
    LCD_STRING(" to cancel test     ");   
    }
    else
    {
    LCD_GotoXY(0,0);
    LCD_STRING(" Lamp test running ");   
    LCD_GotoXY(0,1);
    LCD_STRING(" > or < move lamp  ");   
    LCD_GotoXY(0,2);
    LCD_STRING(" OK to cancel test ");   
    }        
    //set lampdriverboard
    switch(cur_option[cur_menu])
      {     
          case 0: //standard board
              lampboard_active = 0;
              lampdriver_variant = NO_AUX_BOARD;
              break;          
          case 1: //extended boards
              lampboard_active = 1;
              lampdriver_variant = AS_2518_43_12_LAMPS;                      
              break;          
          case 2: //extended boards
              lampboard_active = 1;
              lampdriver_variant = AS_2518_52_28_LAMPS;                      
              break;          
          case 3: //extended boards
              lampboard_active = 1;
              lampdriver_variant = AS_2518_23_60_LAMPS;                      
              break;          
          case 4: //comboboard is main board
              lampboard_active = 0;
              lampdriver_variant = AS_2518_147_60_LAMPS;                      
              break;                        
      }      
      
    //preset
    i = select_from[cur_menu];  
    
  while(1)  //test forever
  {
    if (cur_lamp_t > 0 )
    {
   
     for(i=select_from[cur_menu]; i<=select_to[cur_menu]; i++)
     {
      LCD_GotoXY(0,3);        
      sprintf(line,"lamp  %02x ON          ",i);
      LCD_STRING(line);   
      lamp_set(i-1,1);   //numbering 0..59  
      //wait & check OK switch                   
      if (sleep_n_100ms(cur_lamp_t)) return;      
      
      LCD_GotoXY(0,3);        
      sprintf(line,"lamp  %02x OFF         ",i);
      LCD_STRING(line);   
      lamp_set(i-1,0);   //numbering 0..59  
      //wait & check OK switch                   
      if (sleep_n_100ms(cur_lamp_t)) return;           }                
   }
    else
   {      
      LCD_GotoXY(0,3);        
      sprintf(line,"lamp  %02x ON          ",i);
      LCD_STRING(line);   
      lamp_set(i-1,1);   //numbering 0..59  
           
       switch ( check_sw())
       {
       case SW_OK:           
           lamp_set(i-1,0);   //old lamp OFF
           return;
       case SW_LEFT:       
            if ( i > select_from[cur_menu]) {
                    lamp_set(i-1,0);   //old lamp OFF
                    i = i -1;
                    }                    
                    else
                    {
                        lamp_set(i-1,0);   //old lamp OFF
                        i = select_to[cur_menu]; //wrap
                    }

            break;
       case SW_RIGHT:        
            if ( i < select_to[cur_menu]) {
                    lamp_set(i-1,0);   //old lamp OFF
                    i = i +1;
                    }
                    else
                    {
                        lamp_set(i-1,0);   //old lamp OFF
                        i = select_from[cur_menu]; //wrap
                    }            
            break;            
    }
  } 
 }
}    

void do_sound_test(void)
{
    int i;
    
    LCD_CMD(LCD_CMD_CLR); // Display loeschen
    LCD_CMD(LCD_CMD_CUR_BLINK);  //cursor blinkend an
    
    LCD_GotoXY(0,0);
    LCD_STRING(" Sound test running ");   
    LCD_GotoXY(0,1);
    LCD_STRING(" press&hold OK      ");   
    LCD_GotoXY(0,2);
    LCD_STRING(" to cancel test     ");   
    
    while(1)  //test forever
    {
     for(i=select_from[cur_menu]; i<=select_to[cur_menu]; i++)
     {
      LCD_GotoXY(0,3);        
      sprintf(line," play sound %02x",i);
      LCD_STRING(line);   
      LCD_GotoXY(12,3);        
      //test S&T only
      switch(cur_option[cur_menu])
      {     
          case 0: //chims 
              solenoid_set(i-1);
              __delay_ms(300);
              solenoid_set(15);
              break;
          case 1:     //32         
          case 2:     //50
              sound_data_standard(i);
              break;          
          case 3:   //51
              ext_soundboard_type = EXT_SB_IS_51;
              sound_data_extended(i);
              break;          
          case 4:   //S&T              
          case 5:
          case 6:
          case 7:
          case 8:
              ext_soundboard_type = EXT_SB_IS_SAT;              
              sound_data_extended(i);
              break;          
      }
      if ( (sleep_n_100ms(cur_sound_t*10)) | (cur_sound_t == 0) ) return;
     }
    }
}

void do_solenoid_test(void)
{
        int i;
        int sol;
    
    LCD_CMD(LCD_CMD_CLR); // Display loeschen
    LCD_CMD(LCD_CMD_CUR_BLINK);  //cursor blinkend an
    
    LCD_GotoXY(0,0);
    LCD_STRING(" Solenoid test running ");   
    LCD_GotoXY(0,1);
    LCD_STRING(" press&hold OK      ");   
    LCD_GotoXY(0,2);
    LCD_STRING(" to cancel test     ");   
    
    while(1)  //test forever
    {
     for(i=select_from[cur_menu]; i<=select_to[cur_menu]; i++)
     {
      LCD_GotoXY(0,3);        
      sprintf(line," pulse solenoid %02x",i);
      LCD_STRING(line);   
      LCD_GotoXY(12,3);        
      
      switch(cur_option[cur_menu])
      {     
          case 0: //momentary solenoids
              solenoid_set(i-1);
              __delay_ms(150);
              solenoid_set(15);
              break;          
          case 1: //continous solenoids
              switch(i)
              {
                  case 1: sol = 0b1110; break;
                  case 2: sol = 0b1101; break;
                  case 3: sol = 0b1011; break;
                  case 4: sol = 0b0111; break;
                  default: sol = 0b1111; break;                  
              }
              cont_sol_set(sol);
              __delay_ms(800);
              cont_sol_set(sol);
              break;          
      }
      if (sleep_n_100ms(cur_sol_t*10)) return;
     }
    }

    
}



//execute the test
void start_test(void)
{
    
    LCD_CMD(LCD_CMD_CLR); // Display loeschen
    LCD_CMD(LCD_CMD_CUR_BLINK);  //cursor blinkend an
        
    LCD_GotoXY(0,0);
    LCD_STRING(" press OK to        ");   
    LCD_GotoXY(0,1);
    switch(cur_menu)
    {
        case 0:
            LCD_STRING(" start Lamp Test    ");        
            break;
        case 1:
            LCD_STRING(" start Sound Test   ");                    
            break;
        case 2:
            LCD_STRING(" start Solenoid Test");                    
            break;
        
    }            
    LCD_GotoXY(0,2);
    LCD_STRING(" any other key      ");        
    LCD_GotoXY(0,3);
    LCD_STRING(" to cancel          ");        
    LCD_GotoXY(7,0); //cursor to OK string
    
    
    //check switch
    if ( check_sw() != SW_OK ) 
    {
        refresh_display();
        return;
    }
        
    
    switch(cur_menu)
    {
        case 0:
            do_lamp_test();
            break;
        case 1:
            do_sound_test();
            break;
        case 2:
            do_solenoid_test();
            break;
        
    }            

    refresh_display();    
    
}


/*
 * Main
 */
void main(void) {

    int i;
    
//Clock Source and Value: Internal 16MHz
OSCCONbits.IRCF0 = 1; 
OSCCONbits.IRCF1 = 1;
OSCCONbits.IRCF2 = 1;
//Device is running from the internal oscillator
OSCCONbits.OSTS = 0;
//Primary clock (determined by FOSC<3:0> in CONFIG1H).
OSCCONbits.SCS = 0;
//OSCCONbits.SCS = 3; //RTH: 64MHz still need to be tested
// turn off the PLL 16*4
OSCTUNEbits.PLLEN = 0;      
//OSCTUNEbits.PLLEN = 1;      // turn on the PLL 16*4

// Wait until PLL is ready
//while(!OSCCON2bits.PLLRDY);

/* Port init */
// Port A
ANSELA = 0;   //All I/O pins are configured as digital
LATA = 0;     //clear port
TRISA = 0b00010000;    //RA4 input (Returns)

//---------------------------------------------
// Port B
ANSELB = 0;   //All I/O pins are configured as digital
TRISB = 0b00000010;    // PORTB All Output except RB1 for INT1

//---------------------------------------------
//Port C
ANSELC = 0;   //All I/O pins are configured as digital
LATC = 0;     //clear port
TRISC = 0b11000000; //RC6&7 input (Returns)

//---------------------------------------------
// Port D
ANSELD = 0;   //All I/O pins are configured as digital
TRISD = 0;    // PORTB All Output

//---------------------------------------------
// Port E
ANSELE = 0;   //All I/O pins are configured as digital
TRISE = 0;    // PORTB All Output


//---------------------------------------------
    //interrupt Flags
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
    //timer0
    #define TMR0_VALUE  131         //interrupt after 2 ms
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
INTCON2bits.INTEDG1 = 0; //INT1 default on falling edge
INTCON3bits.INT1IP = 0, //INT1 External Interrupt Priority bit, low prio
INTCON3bits.INT1IF = 0;  //INT1 External Interrupt Flag bit cleared
INTCON3bits.INT1IE = 1; //INT1 External Interrupt Enable bit


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
ZERO_CR_LED = 1; //Zerocross LED initial on

LCD_INIT();

LCD_CMD(LCD_CMD_CLR); // Display loeschen
LCD_CMD(LCD_CMD_CUR_BLINK);  //cursor blinkend an

//initial output
sprintf(line,"Bally Tester V %s",VERSION);
LCD_GotoXY(0,0);
LCD_STRING(line);
LCD_GotoXY(0,1);
LCD_STRING("   www.lisy.dev");
LCD_GotoXY(0,2);
LCD_STRING("    READY");
LCD_GotoXY(0,3);
LCD_STRING(" press OK to start");
LCD_GotoXY(7,3); //cursor to OK string

//wait for OK 
while(check_sw() != SW_OK) { NOP(); } 

//show menu 1
refresh_display();

  while(1)
  {
   switch ( check_sw())
   {
       case SW_OK:           
           start_test();
           break;
       case SW_UP:
           if (cur_line > 0) --cur_line;        
           else cur_line = 3; //wrap
           refresh_display();           
           break;
       case SW_DOWN:
           if (cur_line < 3) ++cur_line;           
           else cur_line = 0; //wrap
           refresh_display();           
           break;
       case SW_LEFT:
           switch(cur_line)
           {
               case 0: //main menu
                    if (cur_menu > 0) --cur_menu;
                    else cur_menu = 2; //wrap           
                    refresh_display();
                    break;
               case 1: //option menu                    
                   if (cur_option[cur_menu] > 0) cur_option[cur_menu] -=1 ;
                   else cur_option[cur_menu] = n_option[cur_menu] -1;//wrap
                    refresh_display();                           
                    break;                   
           }
           break;
       case SW_RIGHT:
           switch(cur_line)
           {
               case 0: //main menu           
                    if (cur_menu < 2) ++cur_menu;
                    else cur_menu = 0; //wrap                      
                    refresh_display();           
                    break;
               case 1: //option menu                    
                    if (cur_option[cur_menu] < n_option[cur_menu]-1) cur_option[cur_menu] += 1;
                    else cur_option[cur_menu] = 0; //wrap
                    refresh_display();                           
                    break;         
               case 2: //select menu 
                    select_menu();
                    break;                                                              
               case 3: //mode menu 
                    mode_menu();
                    break;                                                           
            }         
           break;       
   }
   
  }//endless loop

}//main
   

