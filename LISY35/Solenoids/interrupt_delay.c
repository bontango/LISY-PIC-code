/*
 * File:   interrupt_delay.c
 * Author: Jakob Foos
 *
 * Created on 9/4/2020 5:45:56 PM UTC
 * "Created in MPLAB Xpress"
 *
 * Using code created by MPLAB Xpress Code Configurator
 *
 */

#include "interrupt_delay.h"

#include "def_coils.h"
#include <stdint.h>
#include <pic18f45k22.h>


static volatile unsigned int TMR2_CountCallBack = 0;


void TMR2_Initialize(void)
{
    // Set TMR2 to the options selected in the User Interface:
    // ~1 ms period
    // Not started yet!

    // PR2 15; 
    PR2 = 0x0F;

    // TMR2 0; 
    TMR2 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR2IF = 0;

    // Enabling TMR2 interrupt.
    PIE1bits.TMR2IE = 1;

    // T2CKPS 1:16; T2OUTPS 1:16; TMR2ON off; 
    T2CON = 0x7A;
    
    // TMRI - low priority
    IPR1bits.TMR2IP = 0;    

}

void TMR2_StartTimer(void)
{
    // Start the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 1;
}

void TMR2_StopTimer(void)
{
    // Stop the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 0;
}

void TMR2_ISR(void)
{
    

    // clear the TMR2 interrupt flag
    PIR1bits.TMR2IF = 0;

    // callback function - called every 18th pass
    if (++TMR2_CountCallBack >= TMR2_INTERRUPT_TICKER_FACTOR)
    {
        // Stop the Timer by writing to TMRxON bit
        T2CONbits.TMR2ON = 0;
        
        // reset ticker counter
        TMR2_CountCallBack = 0;
    }
}


void delay_18ms_interruptable(void)
{
    // The 18 ms are defined in the header
    
    // Reset timer2
    TMR2_CountCallBack = 0;
    TMR2 = 0x00;
    
    TMR2_StartTimer();
    
    while(T2CONbits.TMR2ON);
    
    
}

void interrupt_delay(void)
{
    TMR2_StopTimer();
}
