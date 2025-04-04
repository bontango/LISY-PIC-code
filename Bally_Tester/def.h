/* 
 * File:   def.h
 * Author: th
 * part of LISY80NG - Displays
 * Version 200
 * 24.12.2015
 */

#ifndef DEF_H
#define	DEF_H

// PIC18F45K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF      // 4X PLL disabled
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
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

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
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) write-protected)
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

//functions

#define _XTAL_FREQ 16000000      // 16 MHz internal Clock // Fosc  frequency for _delay()  library
//#define _XTAL_FREQ 64000000      // 4*16 MHz internal Clock // Fosc  frequency for _delay()  library

//Port Definitions
//PORTA 
#define SOUND_DATA_E LATAbits.LA0
#define LAMPSTR2 LATAbits.LA1  //Lamp Strobe 2
#define LAMPSTR1 LATAbits.LA2  //Lamp Strobe 1
#define AD3 LATAbits.LA3   //Lamp Address 3
#define RETURN_1 PORTAbits.RA4
#define AD2 LATAbits.LA5   //Lamp Address 2
#define PD2 LATAbits.LA6    //Lamp Data 2
#define PD1 LATAbits.LA7    //Lamp Data 1

//PortB
#define ZEROCROSS PORTBbits.RB1   //Zero Cross Interrupt
//all other bits on Port B for display

//PortC
#define PD3 LATCbits.LC0    //Lamp Data 3
#define MOM_SOL_SOUND_DATA_D LATCbits.LC1   
#define MOM_SOL_SOUND_DATA_C LATCbits.LC2   
#define MOM_SOL_SOUND_DATA_B LATCbits.LC3   
#define CO1_PB6 LATCbits.LC4
#define CO4_PB5 LATCbits.LC5   
#define RETURN_2 PORTCbits.RC6
#define RETURN_3 PORTCbits.RC7

//PortD
#define MOM_SOL_SOUND_DATA_A LATDbits.LD0
#define CO2_PB4 LATDbits.LD1 
#define SOUNDSELECT LATDbits.LD2
#define CO3_PB7 LATDbits.LD3   
#define STROBE_1 LATDbits.LATD4
#define STROBE_2 LATDbits.LATD5
#define STROBE_3 LATDbits.LATD6
#define ZERO_CR_LED LATDbits.LATD7

//PortE
#define AD1 LATEbits.LE0   //Lamp Address 1
#define AD0 LATEbits.LE1   //Lamp Address 1
#define PD0 LATEbits.LE0    //Lamp Data 0

#endif	/* DEF_H */

