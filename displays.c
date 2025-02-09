/* 
 * File:   main.c DISPLAYS
 * Author: th
 * Display routine for System 80/80A
 * Part of LISY80NG
  * PIC18F45K42
 * Version 6.0 11.04.2022
 * READ Port / Write LAT
 * 6.1.1 added 500ms delay at start to give display time to boot
 * 6.1.2 added 10us extra delay in 'push_data'
 */

//no warnings about 
#pragma warning push
//#pragma warning disable 359 //illegal conversion between pointer types
#pragma warning disable 373 //'implicite signed to unsigned conversion'

#include "def_displays.h"  //includes <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "GTB_Symbols.h"
#include "fifo_displays.h"

int lisy80_dp_version = 600;  //LISY80 Display Version
uint8_t lisy80_hw_revision = 2;  //Foreseen LISY80 HW revision (autodetect))

/* global definitions */         
volatile uint8_t player1[7];
volatile uint8_t player2[7];
volatile uint8_t player3[7];
volatile uint8_t player4[7];
volatile uint8_t player5[6]; //80A only (Bonus)
volatile uint8_t player6[6];//80A only (Timer)
volatile uint8_t status[4]; 
volatile uint8_t no_of_bytes;
  

/* globale defs */
uint8_t interpret_mode = 1;
uint8_t picmd;
uint8_t is80B = 0;

//used in int
volatile uint8_t data_slave;


//globale var, used in most routines
    union three {
    unsigned char byte;
    struct {
    unsigned DISPLAY:3, DIGIT:3, IS80B:1, IS_CMD:1;
    //signed b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv;    //80A
    struct {
    unsigned COMMAND_BYTE:7, IS_CMD:1;
    //signed b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv2;    
    struct {
    unsigned ROW:2, POSITION:4, COOKED:1, IS_CMD:1;
    //signed b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv3;   //for 80B
    } pidata;
                     
    
uint8_t get_dip_setting(void)
{
            
    bitv_t dip;
    
    //do init the var
    dip.byte = 0;
    
    //disable interrupts    
    di();
    
    //disable hct595
    HCT595_RESET = 0;

    //at first use STROBE1 (RA1) which is DIP1..3)
    STROBE1 = 1; STROBE2=0;
    //let the value be stable
    __delay_ms(10);
    //RET1=DIP6; RET2=DIP5; RET3=DIP4
    dip.bitv.b2 = ~RETURN1;
    dip.bitv.b1 = ~RETURN2;
    dip.bitv.b0 = ~RETURN3;
    
    //then use STROBE2 (RA2) which is DIP4..6)
    STROBE1 = 0; STROBE2=1;
    //let the value be stable
    __delay_ms(10);
    //RET1=DIP3; RET2=DIP2; RET3=DIP1
    dip.bitv.b5 = ~RETURN1;
    dip.bitv.b4 = ~RETURN2;
    dip.bitv.b3 = ~RETURN3;
        
    //enable hct595 again
    HCT595_RESET = 1;
    
    //enable interrupts again
    ei();

    return dip.byte;    
    
}
    
//for all Groups
//Symbol		a	c	e	g	h	f	d	b
//order high to low -> bit7 first
//


//GROUP  A is Pin1..8 Player1 ; 3 and 5 (Bonus)
set_group_A(uint8_t value)
{
    bitv_t bits;
    
    bits.byte = value;
    PIN1 = bits.bitv.b7; //a
    PIN2 = bits.bitv.b0; //b
    PIN3 = bits.bitv.b6; //c
    PIN4 = bits.bitv.b1; //d
    PIN5 = bits.bitv.b5; //e
    PIN6 = bits.bitv.b2; //f
    PIN7 = bits.bitv.b4; //g
    PIN8 = bits.bitv.b3; //h
    
}

//GROUP  B is Pin9..16 ; Player2 ; 4 and 6 (Timer)
set_group_B(uint8_t value)
{
    bitv_t bits;
    
    bits.byte = value;
    PIN9 = bits.bitv.b7;
    PIN10 = bits.bitv.b0;
    PIN11 = bits.bitv.b6;
    PIN12= bits.bitv.b1;
    PIN13 = bits.bitv.b5;
    PIN14 = bits.bitv.b2;
    PIN15 = bits.bitv.b4;
    PIN16 = bits.bitv.b3;
}

//Group C is Pin17..24 ; Status Display
set_group_C(uint8_t value)
{
    bitv_t bits;
    
    bits.byte = value;
    PIN17 = bits.bitv.b7;
    PIN18 = bits.bitv.b0;
    PIN19 = bits.bitv.b6;
    PIN20= bits.bitv.b1;
    PIN21 = bits.bitv.b5;
    PIN22 = bits.bitv.b2;
    PIN23 = bits.bitv.b4;
    PIN24 = bits.bitv.b3;
}


void reset_10941(void)
{
    int i;
    
    // /RESET on low
    DISP_RESET = 0;        
    
    //put both LDs on zero
    LD1 = LD2 = 0;
    
    //wait for Display to be ready
    //__delay_ms(100); //new compiler -> too large?
    for(i=0; i<10; i++) __delay_ms(10);
    //and put /RESET to High
    DISP_RESET = 1;
    //toggle D7 to let the display know we are in parallel mode
    D7 =1;
    __delay_ms(1);
    D7= 0;
    
}

void __interrupt(irq(default)) DEFAULT_ISR(void)
{
// Unhandled interrupts go here
}

/* interrupt service routine for display refresh */
void __interrupt(irq(TMR0),low_priority) displayIsr(void)    {
    
   static unsigned char digit=15;  //with the first ++ we start with 0   
   int i;
    
   PIR3bits.TMR0IF = 0;  // clear this interrupt condition            
                        
   /* next digit to activate */   
   /* appr. each 16*0,5ms == 8ms */
   digit++;    
   if (digit > 15) { digit = 0; }
    
    //only for 80 & 80A systems
   if(!is80B)
   {     
    /* no digits to select during setting phase*/
    SERIN=0;
    RCLK=0;
    for(i=0; i<=15; i++)
    {
        CLK = 0;
        NOP();
        CLK = 1;        
    }
    RCLK=1;
    
    /* set new digit values */
    //digit 0..5 serves Player1 ; 3 and 5 (Bonus)
    if (digit <= 5)
        {          
          set_group_A( player1[digit]);                    
          set_group_B( player3[digit]);                    
          set_group_C( player5[digit]);          
        }
    //digit 6..10 serves Player2 ; 4 and 6 (Timer)
    if (( digit > 5) && ( digit <= 11))
        {
          set_group_A( player2[digit - 6]);                    
          set_group_B( player4[digit - 6]);                    
          set_group_C( player6[digit - 6]);                              
        }
    if ( digit > 11)
        {
            if ( digit == 12) //7.Digit SYS80A Player 2 and 4
                {                
                set_group_A(player2[6]);                                 
                set_group_B(player4[6]);                
                }
            if ( digit == 15) //7.Digit SYS80A Player 1 and 3
                {                
                set_group_A(player1[6]);                                
                set_group_B(player3[6]);               
                }          
          set_group_C(status[digit - 12]);          
        }
                        
    // set the rigth pin   *****
    SERIN=1;
    CLK = 0;
    RCLK=0;
    NOP();
    CLK = 1;    
    
    // now we have '1' on pin1 -> shift it
    SERIN=0;
    for(i=0; i<digit; i++)
    {
        CLK = 0;
        NOP();
        CLK = 1;        
    }
    RCLK=1;
    
   }// !is80B
   
}//timer-interrupt routine

typedef enum
{
    I2C1_IDLE,
    I2C1_ADDR_TX,
    I2C1_ADDR_RX,
    I2C1_DATA_TX,
    I2C1_DATA_RX
} i2c1_slave_state_t;

static volatile i2c1_slave_state_t i2c1SlaveState = I2C1_IDLE;

void __interrupt(irq(I2C1TX,I2C1RX,I2C1E,I2C1),high_priority) I2C1_ISR(void) {
        
        uint8_t slave_addr; //not used but need the read?!
    
        I2C1PIR = 0x00;  //I2C1_SlaveClearIrq();        
        if(!(I2C1STAT0bits.D)) //if(I2C1_SlaveIsAddr())  //   return 
        {
            if(I2C1STAT0bits.R) //if(I2C1_SlaveIsRead())
            {
                i2c1SlaveState = I2C1_ADDR_TX;
            }
            else
            {
                i2c1SlaveState = I2C1_ADDR_RX;
            }
        }
        else
        {
            if(I2C1STAT0bits.R)  //if(I2C1_SlaveIsRead())
            {
                i2c1SlaveState = I2C1_DATA_TX;
            }
            else
            {
                i2c1SlaveState = I2C1_DATA_RX;
            }
        }

        switch(i2c1SlaveState)
        {
            case I2C1_ADDR_TX:
                //slave_addr = I2C1RXB; //I2C1_SlaveAddrCallBack(); RTH needed ?
                I2C1STAT1bits.CLRBF = 1; //I2C1_SlaveClearBuff();
                if(I2C1STAT1bits.TXBE) //if(I2C1_SlaveIsTxBufEmpty())
                {
                    I2C1TXB = data_slave; //I2C1_SlaveWrCallBack();
                }
                break;
            case I2C1_ADDR_RX:
                //slave_addr = I2C1RXB; //I2C1_SlaveAddrCallBack(); RTH needed ?
                I2C1STAT1bits.CLRBF = 1; //I2C1_SlaveClearBuff();
                break;
            case I2C1_DATA_TX:
                if(I2C1STAT1bits.TXBE) //if(I2C1_SlaveIsTxBufEmpty())
                {
                    I2C1TXB = data_slave; //I2C1_SlaveWrCallBack();
                }
                break;
            case I2C1_DATA_RX:
                if (I2C1STAT1bits.RXBF) //if(I2C1_SlaveIsRxBufFull())
                {
                //put byte from master to Buffer, interpret in main
            	BufferIn ( I2C1RXB ); //I2C1_SlaveRdCallBack();                    
                }
                break;
            default:
                break;
        }
        I2C1CON0bits.CSTR = 0; //I2C1_SlaveReleaseClock();
    }



//set the right segments
//called by execute_cmd in main
set_segments( uint8_t digit, uint8_t display, unsigned char value)
{
    uint8_t segments;
    
    segments = gtb_symbol_for_ascii(value);
    
switch(display) {
            case 0:if (digit>3) break;
                   status[digit] = segments;
                   break;
            case 1:if (digit>6) break;
                   player1[digit] = segments;
                   break;
            case 2:if (digit>6) break;
                   player2[digit] = segments;
                   break;
            case 3:if (digit>6) break;
                   player3[digit] = segments;
                   break;
            case 4:if (digit>6) break;
                   player4[digit] = segments;
                   break;
            case 5:if (digit>5) break;
                   player5[digit] = segments;
                   break;
            case 6:if (digit>5) break;
                   player6[digit] = segments;
                   break;
                        } // switch display               
}

//push a byte to display controller Rockwell 10941
//row is 1 or 2
//use row==3 for pushing to row 1 and 2
void push_data_to_10941(uint8_t value, uint8_t row)
{
    union both8 {
    unsigned char byte;
    struct {
    unsigned b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;    
        } bitv;
    } myvalue;
    
    myvalue.byte = value;
    
    // LD1 and LD2 on zero
    LD1 = LD2 = 0;    

    D0 = myvalue.bitv.b0;
    D1 = myvalue.bitv.b1;        
    D2 = myvalue.bitv.b2;
    D3 = myvalue.bitv.b3;                       
    D4 = myvalue.bitv.b4;
    D5 = myvalue.bitv.b5;    
    D6 = myvalue.bitv.b6;
    D7 = myvalue.bitv.b7;                       
    
    //now push the byte to the rigth row by pulse the LD    
    if ((row == LS80D_B_TOROW1)|(row == LS80D_B_TOROW12))
     {
      
      //set LD1 = 1
      LD1 = 1;
      //wait a bit
      __delay_us(10);      
      //and set back to 0 again
      LD1 = 0;
      //K42 is too fast, add delay
      __delay_us(3);   
     }
    if ((row == LS80D_B_TOROW2)|(row == LS80D_B_TOROW12))
     {
      
      //set LD2 = 1
      LD2 = 1;
      //wait a bit
      __delay_us(10);      
      //and set back to 0 again      
      LD2 = 0;
      //K42 is too fast, add delay
      __delay_us(3);   
     }     
          //K42 is too fast, add delay
      __delay_us(10);   

}


show_string_80B( char string[], uint8_t row)
{
    int i,len;
    len = strlen(string);
    //limit len
    if (len>20) len=20;
    
    for (i=0; i<len; i++)
        push_data_to_10941( string[i],row);
    
}

// init display, show LISY80B and Version
void init_10941(void)
{    
  uint8_t dip;  
  //read the dip settings
  dip = get_dip_setting();
  
  //do a reset first
  reset_10941();
  
  //Digit Counter = 20, both rows
  push_data_to_10941(1, LS80D_B_TOROW12);  
  push_data_to_10941(0x94, LS80D_B_TOROW12);  
  //Digit time 32 cycle per Grid
  push_data_to_10941(1, LS80D_B_TOROW12);
  push_data_to_10941(6, LS80D_B_TOROW12);
  
  //Duty cicle 26/6
  push_data_to_10941(1, LS80D_B_TOROW12);
  push_data_to_10941(0x5c, LS80D_B_TOROW12);  

  //start display refresh
  push_data_to_10941(1, LS80D_B_TOROW1);
  push_data_to_10941(0x0E, LS80D_B_TOROW1);  
    
  //Show Initial Message by using lisy80_char which is not in use for 80B
  sprintf(lisy80_char,"LISY80B GAME NO %02d  ",dip);
  show_string_80B( lisy80_char,1); 
  sprintf(lisy80_char,"WAIT FOR PI          ");
  show_string_80B( lisy80_char,2);
   
}



//Init for system80 & 80A
void init_80(void)
{
  int i,ver,temp;      
  uint8_t dip;  
  //read the dip settings
  dip = get_dip_setting();  

//Load 80&80A display segment table
assign_the_chars();

// Display Init: LISY80 for player 1 
player1[5]=gtb_symbol_for_ascii('L');
player1[4]=gtb_symbol_for_ascii('I');
player1[3]=gtb_symbol_for_ascii('S'); 
player1[2]=gtb_symbol_for_ascii('Y');
player1[1]=gtb_symbol_for_ascii('8');
player1[0]=gtb_symbol_for_ascii('0');
// LISY80A in case of SYS80A
player1[6]=gtb_symbol_for_ascii('A');
// Default values for the other display
for (i=0; i<=6; i++)  player2[i]= gtb_symbol_for_ascii(' ');
for (i=0; i<=6; i++)  player3[i]= gtb_symbol_for_ascii(' ');
for (i=0; i<=6; i++)  player4[i]= gtb_symbol_for_ascii(' ');
for (i=0; i<=5; i++)  player5[i]= gtb_symbol_for_ascii(' ');
for (i=0; i<=5; i++)  player6[i]= gtb_symbol_for_ascii(' ');
//Value of DIP Switch at player2
player2[5]=gtb_symbol_for_ascii('G');
player2[4]=gtb_symbol_for_ascii('A');
player2[3]=gtb_symbol_for_ascii('M');
player2[2]=gtb_symbol_for_ascii('E');
player2[1]=gtb_symbol_for_number(dip/10);
dip = dip - ((dip/10) * 10);
player2[0]=gtb_symbol_for_number(dip);
player2[6]=gtb_symbol_for_ascii(' ');
// player3says 'wait'
player3[5]=gtb_symbol_for_ascii('W');
player3[4]=gtb_symbol_for_ascii('A');
player3[3]=gtb_symbol_for_ascii('I');
player3[2]=gtb_symbol_for_ascii('T');
// player3says 'for PI
player4[5]=gtb_symbol_for_ascii('F');
player4[4]=gtb_symbol_for_ascii('O');
player4[3]=gtb_symbol_for_ascii('R');
player4[2]=gtb_symbol_for_ascii(' ');
player4[1]=gtb_symbol_for_ascii('P');
player4[0]=gtb_symbol_for_ascii('I');

//Software Version at status display
//status[3]=gtb_symbol_for_ascii('V');
//status[2]=gtb_symbol_for_number(lisy80_dp_version/100);
//temp = lisy80_dp_version%100;
//status[1]=gtb_symbol_for_number(temp/10);
//status[0]=gtb_symbol_for_number(temp%10);

PIN1 = PIN2 = PIN3 = PIN4 = PIN5 = PIN6 = PIN7 = PIN8 = 0;
PIN9 = PIN10 = PIN11 = PIN12 = PIN13 = PIN14 = PIN15 = PIN16 = 0;
PIN17 = PIN18 = PIN19 = PIN20 = PIN21 = PIN22 = PIN23 = PIN24 = 0;
//enable outputs of hct595
HCT595_RESET = 1;
OE = 0;


}

//Init for system80B
void init_80B(void)
{
 // sleep(2);   delay at start?
 //No Timer interrupt for System80B
//INTCONbits.T0IE=0; // disable Timer0 interrupt 
//we use timer for eeprom background write
 
 //initialize chip
init_10941();
 
   
}


/*
 * Push the data to the 80B display
 */
void push_data_80B(uint8_t row, uint8_t cooked, uint8_t position, uint8_t dataByte)
{
    //in case 'cooked' mode we interpret the same way as for 80 & 80A
    if (cooked)
    {
        //place buffer pointer
        push_data_to_10941(1, row);
        push_data_to_10941(0xC0 + position, row);  
        //and push the databyte there
        push_data_to_10941(dataByte, row);
    }
    else
    {
        push_data_to_10941(dataByte, row);
    }
    
}

/*
 * Main
 */
void main(void) {

int i;
uint8_t dataByte, display, digit;
uint8_t row,cooked,position;
uint8_t bytes_to_push = 0;
uint8_t eeprom_address, eeprom_data;
uint8_t eeprom_read_mode = 0;
uint8_t eeprom_write_mode = 0;
 
/* Port init */
// Port A
PORTA = 0;
ANSELA = 0;   //All I/O pins are configured as digital
ADCON1=0x6;   // PortA Digital
TRISA = 0b00010011;    //1 RA0 Return1 Input
                       //1 RA1,Return2 Input
                       //0 RA2 Output (LED)
                       //0 RA3 Output (ENABLE_4514)
                       //1 RA4 Return3 Input
                       //0 RA5 output '6'
                       //0 RA6 output ( D1 & Strobe 1))
                       //0 RA7,output ( D1 & Strobe 1))
// Port B
PORTB = 0;
ANSELB = 0;   //All I/O pins are configured as digital
TRISB = 0;    // PORTB All Output
//Port C
PORTC = 0;
ANSELC = 0;   //All I/O pins are configured as digital
//TRISC = 0b00011000; // PORTC All Output except I2C
TRISC = 0; // PORTC All Output
//Port D
PORTD = 0;
ANSELD = 0;   //All I/O pins are configured as digital
TRISD = 0; //all output
//Port E
PORTE = 0;
ANSELE = 0;   //All I/O pins are configured as digital
TRISE = 0b1000; // E3 is input, others output

/* I2C */        
//https://www.reddit.com/r/pic_programming/comments/av5vdg/has_anyone_gotten_an_i2c_slave_to_work_with/
// Setup MSSP in 7 bit I2C Slave mode according to TB3159
//CONFIGURING SCL AND SDA PINS
// Configure the pins as digital
ANSELCbits.ANSELC3 = 0;
ANSELCbits.ANSELC4 = 0;
// PPS Unlock Sequence
PPSLOCK = 0x55;
PPSLOCK = 0xAA;
PPSLOCKbits.PPSLOCKED = 0x00;
// Set RC4 for SDA
RC4PPS = 0x22;
I2C1SDAPPS = 0x14;
// Set RC3 for SCL
RC3PPS = 0x21;
I2C1SCLPPS = 0x13;
// PPS Lock Sequence
PPSLOCK = 0x55;
PPSLOCK = 0xAA;
PPSLOCKbits.PPSLOCKED = 0x01;
// Configure the pins as Open-drain
ODCONCbits.ODCC3 = 1;
ODCONCbits.ODCC4 = 1;
// Set the I2C levels
RC3I2Cbits.TH = 1;
RC4I2Cbits.TH = 1;
// Configure the pins as Outputs
TRISCbits.TRISC3 = 0;
TRISCbits.TRISC4 = 0;

// Slave Address Match
I2C1ADR0 = I2C_ADDR;
I2C1ADR1 = I2C_ADDR;
I2C1ADR2 = I2C_ADDR;
I2C1ADR3 = I2C_ADDR;

// Configure I2C1 module
 I2C1CON1 = 0x00;  // CSD Clock Stretching enabled; ACKDT Acknowledge; ACKCNT Not Acknowledge;  
 I2C1CON2 = 0x08;  // ABD enabled; SDAHT 30 ns; BFRET 8 I2C Clocks; FME disabled;
 I2C1CLK  = 0x00;  // Slave doesn't use I2CCLK
 I2C1CNT  = 0x00;  // Zero the byte counter
 I2C1CON0 = 0x00;  // CSTR enable clocking; S Cleared by hardware after Start; MODE four 7-bit address; 
    
 // Clear interrupt flags
    PIR2bits.I2C1RXIF = 0;
    PIR3bits.I2C1TXIF = 0;
    PIR3bits.I2C1IF = 0;
    I2C1PIRbits.PCIF = 0;
    I2C1PIRbits.ADRIF = 0;
    
    // Enable interrupts
    PIE2bits.I2C1RXIE = 1;
    PIE3bits.I2C1TXIE = 1;
    PIE3bits.I2C1IE = 1;
    I2C1PIEbits.PCIE = 1;
    I2C1PIEbits.ADRIE = 1;
    
    // Enable I2C1
    I2C1CON0bits.EN = 1;     
    
       // ACK for every valid byte (ACKDT = 0)
    // ACK at the end of a Read (ACKCNT = 0)
    // Clock stretching Enabled! (RTH) (CSTRDIS = 0)
    I2C1CON1 = 0;
    
    // Auto-count disabled (ACNT = 0)
    // General Call disabled (GCEN = 0)
    // Fast mode DISabled (FME = 0)
    // ADB0 address buffer used (ADB = 0)
    // SDA Hold time of 300 ns (SDAHT = 0)
    // Bus free time of 8 I2C Clock pulses
    // (BFRET = 1)
    I2C1CON2 = 0x08;
    
    // Slaves don't use the clock
    I2C1CLK = 0x00;
    
    // Zero out the byte count register
    I2C1CNT = 0x00;
    
    // Clear all I2C flags
    PIR3bits.I2C1IF = 0;
    I2C1PIR = 0x00;
    
/* Timer interrupt init, fuer den Strobe brauche wir alle 0,5 ms einen INT */
/* 16Mhz Takt -> ergibt 16/4 gleich 4 MHz fuer Timer0, entspricht alle 0,25us */
/* prescaler 64 entspricht alle 16 us */
// TMR0H = 77 mal 16 us = alle 1,25 ms ein interrupt
//verified with MCC!

//with the below settings one interrupt each 1,25ms ( 16 * 16 us ))
//T0CON1 = 0x66; // T0CS HFINTOSC; T0CKPS 1:64; T0ASYNC synchronised;  
T0CON1bits.CS = 2; //010 = FOSC/4
T0CON1bits.ASYNC = 0; //0 = The input to the TMR0 counter is synchronized to FOSC/4
T0CON1bits.CKPS = 8; //prescaler 1000 = 1:256
TMR0H = 0x23;    // TMR0H 35;     
TMR0L = 0x00;    // TMR0L 0; 
T0CON0bits.EN = 1; //1 = The module is enabled and operating
T0CON0bits.MD16 = 0; //8bit timer mode
//T0CON0 = 0x80; // T0OUTPS 1:1; T0EN enabled; T016BIT 8-bit;  


    
    
    // now enable Interrupts 
    INTCON0bits.GIEH = 1; // Enable high priority interrupts
    INTCON0bits.GIEL = 1; // Enable low priority interrupts
    INTCON0bits.IPEN = 1; // Enable interrupt priority
    
PIR3bits.TMR0IF = 0;   // Clear Interrupt flag before enabling the interrupt    
IPR3bits.TMR0IP = 0; //low priority
PIE3bits.TMR0IE = 1; // Enabling TMR0 interrupt.
T0CON0bits.T0EN = 1;// Start the Timer by writing to TMR0ON bit
    
/* init */     
LED = 1;

//read first 6 bits of dipswitch and see if we are 80/80A or 80B
//all values > 39 are supposed to be 80B    
if ( get_dip_setting() > 39 ) is80B=1; else is80B = 0;

//disable outputs of hct595 and set device to reset state
OE = 1;
HCT595_RESET = 0;

//wait 500ms for boot of display
for(i=0; i<=50; i++) __delay_ms(10);

//Now do init
if (is80B) init_80B(); else init_80();

//enable interrupts
ei();

while( 1 ) { //Loop forever

//check if we have a byte in the Buffer
if ( BufferOut(&pidata.byte) == BUFFER_SUCCESS )
{                 
    //check if we are in assign_mode                                                                                    
   if (interpret_mode == 1)
      {
         //is this an assigment? 
         if ( pidata.bitv.IS_CMD == 0 )
            {          
             //yes, assign data
             if (is80B)
             {
                 row = pidata.bitv3.ROW;
                 cooked = pidata.bitv3.COOKED;
                 position = pidata.bitv3.POSITION;
             }
             else
             {
                display = pidata.bitv.DISPLAY;
                digit = pidata.bitv.DIGIT;                                           
             }
                //next byte not to interpret
                interpret_mode = 0;                           
            } 
         else //no, it is a command, we have to execute
            {             
              switch(pidata.bitv2.COMMAND_BYTE) { 
                      case LS80DPCMD_GET_SW_VERSION_MAIN: //send SW version  
                                data_slave=lisy80_dp_version / 100; //main version number               
                                break;            
                      case LS80DPCMD_GET_SW_VERSION_SUB: //send SW version                 
                                data_slave=lisy80_dp_version % 100; //sub version number
                                break;            
                      case LS80DPCMD_GET_DIPSW_VALUE:    //send value of DIP_SWitch                
                                data_slave= get_dip_setting();
                                break;                       
                      case LS80DPCMD_RESET:    //do a reset (again?))
                                reset_10941();                 
                                break;                                  
                      case LS80DPCMD_INIT:    //do a reset (again?))
                                init_10941();                 
                                break;                                                                  
                      case LS80DPCMD_NEXT20_ROW1:    //next 20Bytes are for row 1
                                interpret_mode = 0;  //next byte not to interpret
                                bytes_to_push = 20;
                                cooked = 0; row=1;
                                break;                                                                                                  
                      case LS80DPCMD_NEXT20_ROW2:    //next 20Bytes are for row 2
                                interpret_mode = 0;  //next byte not to interpret
                                bytes_to_push = 20;
                                cooked = 0; row=2;
                                break;                                                                                                  
//                      case LS80DPCMD_READ_EEPROM:    //read a byte from eeprom
//                                eeprom_read_mode = 1;                 
//                                break;            
//                      case LS80DPCMD_WRITE_EEPROM:    //write a byte to eeprom
//                                eeprom_write_mode = 1;                 
//                                break;            
                      case LS80DPCMD_GET_HW_REV:    //send foreseen HW revision
                          data_slave = lisy80_hw_revision;
                                break;                                                                            
                      }//switch Command_byte   
                   }//command to execute                  
                }
   else //assign mode, we do not interpret data here
               {
                 dataByte = pidata.byte;
                 //signal data input
                 LED = ~LED;
                 //we are in assign mode, push data
                 //together with data we collected earlier                 
                 if (is80B)
                 {
                  push_data_80B(row, cooked, position, dataByte);                       
                 }
                 else
                 {
                  set_segments (digit, display, dataByte);                    
                 }                  
                 //prepare for next byte
                 if (bytes_to_push > 1)
                 {
                     bytes_to_push--;
                     interpret_mode = 0;
                 }
                 else interpret_mode = 1;                
               }                                                  
     } // if Bufferstatus  
 } //while 1, endless loop
} //main      