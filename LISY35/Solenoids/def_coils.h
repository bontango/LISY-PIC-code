/* 
 * File:   def.h
 * Author: th
 * part of LISY35 - Coils
 * Version 100
 * 17.07.2017
 */

#ifndef DEF_H
#define	DEF_H

// PIC18F45K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = ON      // 4X PLL Enable (Oscillator multiplied by 4)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
//#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config PWRTEN = OFF      // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = INTMCLR  // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
//While in Low-Voltage ICSP mode, MCLRis always enabled, regardless of the
//MCLRE bit, and the RE3 pin can no longer be used as a general purpose input.
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


//#define _XTAL_FREQ 64000000      // 4*16 MHz internal Clock // Fosc  frequency for _delay()  library
//real frequ is 16 MHz !
#define _XTAL_FREQ 16000000      // 4*16 MHz internal Clock // Fosc  frequency for _delay()  library

//timer0
#define TMR0_VALUE  131         //interrupt after 2 ms

//Application Definitions
#define I2C_ADDR 0x82          //I2C Bus Adress PIC

//Port Definitions
//PORTA all output
#define SOUNDSELECT LATAbits.LA0
// #define CON_SOL_DATA_5 PORTAbits.RA1   //NU
#define CO3_PB7 LATAbits.LA2   
#define CO1_PB6 LATAbits.LA3   
//#define SOL5 PORTAbits.LA4   //NC
#define CO4_PB5 LATAbits.LA5   
#define MOM_SOL_SOUND_DATA_D LATAbits.LA6   
#define STROBE8 LATAbits.LA6   
#define MOM_SOL_SOUND_DATA_C LATAbits.LA7 
#define STROBE7 LATAbits.LA7 

//PORTB B all output except RB0 & RB3 which are options via k3
//and RB1 which is interupt input for bally zero cross
#define NOINTROVOICE PORTBbits.RB0   //k3 dip 1
#define ZEROCROSS PORTBbits.RB1   //Zero Cross Interrupt
#define LED LATBbits.LB2     //Anschluss LED
#define LED_READ PORTBbits.RB2     //Anschluss LED for read
#define NOVOLOPT PORTBbits.RB3    //k3 dip 2
#define PD0 LATBbits.LB4    //Lamp Data 0
#define PD1 LATBbits.LB5    //Lamp Data 1
#define PD2 LATBbits.LB6    //Lamp Data 2
#define PD3 LATBbits.LB7    //Lamp Data 3
#define STROBE4 LATBbits.LB4    //Strobe for dip switch
#define STROBE3 LATBbits.LB5    //Strobe for dip switch
#define STROBE2 LATBbits.LB6    //Strobe for dip switch
#define STROBE1 LATBbits.LB7    //Strobe for dip switch
#define INHIBIT LATB


//PORTC all outputs excep RC3&4 for I2C
#define RETURN1 PORTCbits.RC0   //Return for dip switches
#define RETURN2 PORTCbits.RC1   //Return for dip switches
#define RETURN3 PORTCbits.RC2   //Return for dip switches
//#define DS03 PORTCbits.RC3   //I2C Clock
//#define DS03 PORTCbits.RC4   //I2C Data
#define RETURN4 PORTCbits.RC5   //Return for dip switches
#define LAMPSTR2 LATCbits.LC6  //Lamp Strobe 2
#define LAMPSTR1 LATCbits.LC7  //Lamp Strobe 1

//PORTD
#define ZERO_CR_LED LATDbits.LD0  //LED for Zerocross Int indication
#define GREENLED LATDbits.LD1  //Bally 'Green LED'
//#define DS02 PORTDbits.RD2   //NC
//#define LD03 PORTDbits.RD3   //NC
#define AD3 LATDbits.LD4   //Lamp Address 3
#define AD2 LATDbits.LD5   //Lamp Address 2
#define AD1 LATDbits.LD6   //Lamp Address 1
#define AD0 LATDbits.LD7   //Lamp Address 0

//PORTE
#define CO2_PB4 LATEbits.LE0 
#define MOM_SOL_SOUND_DATA_A LATEbits.LE1 
#define STROBE5 LATEbits.LE1  //Strobe for dip switch
#define MOM_SOL_SOUND_DATA_B LATEbits.LE2   //Tens Chime (Sol/Sound))
#define STROBE6 LATEbits.LE2  //Strobe for dip switch
#define Extension PORTEbits.RE3      //Input Jumper for future extension

//the commands
#define LS80COILCMD_EXT_CMD_ID 0  //extended command for ver 4
#define LS80COILCMD_GET_SW_VERSION_MAIN 1 
#define LS80COILCMD_GET_SW_VERSION_SUB 2
#define LS35COIL_CONT_SOL 3
#define LS35COIL_MOM_SOL 4
#define LISY35_STANDARD_SOUND 5
#define LISY35_EXTENDED_SOUND 6
#define LS80COILCMD_GET_K3 7

#define LISY_EXT_CMD_EEPROM_READ 0
#define LISY_EXT_CMD_EEPROM_WRITE 1
#define LISY35_EXT_CMD_AUX_BOARD_0 2  // no aux board
#define LISY35_EXT_CMD_AUX_BOARD_1 3  // AS-2518-43 12 lamps
#define LISY35_EXT_CMD_AUX_BOARD_2 4  // AS-2518-52 28 lamps
#define LISY35_EXT_CMD_AUX_BOARD_3 5  // AS-2518-23 60 lamps
#define LISY35_EXT_CMD_SB_IS_51 6  //Soundboard is a 2581-51 (extended mode)
#define LISY35_EXT_CMD_SB_IS_SAT 7  //Soundboard is a S&T (extended mode)
#define LISY35_EXT_CMD_J4PIN5_INPUT 8  // CO2_PB4 LATEbits.LE0   cont Sol 1
#define LISY35_EXT_CMD_J4PIN5_OUTPUT 9  // CO2_PB4 LATEbits.LE0  cont Sol 1
#define LISY35_EXT_CMD_J4PIN8_INPUT 10  //CO3_PB7 LATAbits.LA2   cont Sol 4
#define LISY35_EXT_CMD_J4PIN8_OUTPUT 11  //CO3_PB7 LATAbits.LA2  cont Sol 4 
#define LISY35_EXT_CMD_J1PIN8_INPUT 12  //LAMPSTR2 LATCbits.LC6  Lamp Strobe 2
#define LISY35_EXT_CMD_J1PIN8_OUTPUT 13  //LAMPSTR2 LATCbits.LC6  Lamp Strobe 2
#define LISY35_READ_DIP_SWITCHES 14  //read dip switches and buffer value
#define LISY35_GET_DIP_SWITCHES 15  //dip switches 1...4, need to be called four times

//special coil numbers
#define LISY35_COIL_GREEN_LED 60
#define LISY35_COIL_SOUNDSELECT 61
#define LISY35_COIL_SOUNDRAW 62
#define LISY35_COIL_LAMPBOARD 63

//the aux lampdriverboards variants
#define NO_AUX_BOARD 0
#define AS_2518_43_12_LAMPS 1
#define AS_2518_52_28_LAMPS 2
#define AS_2518_23_60_LAMPS 3

//other
#define SOLENOIDS_SELECTED 0
#define SOUNDS_SELECTED 1
#define EXT_SB_IS_51 0  //Soundboard is a 2581-51 (extended mode)
#define EXT_SB_IS_SAT 1  //Soundboard is a S&T (extended mode)

//typedefs
typedef union typed {
    unsigned char byte;
    struct {    
    unsigned b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv;
    } bitv_t;

#endif	/* DEF_H */

