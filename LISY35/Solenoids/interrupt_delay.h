/* 
 * File:   interrupt_delay.h
 * Author: Jakob Foos
 *
 * Created on 9/4/2020 5:21:08 PM UTC
 * "Created in MPLAB Xpress"
 *
 *
 * Uses code created with the MPLAB Xpress Code Configurator
 *
 */

#ifndef INTERRUPT_DELAY_H
#define INTERRUPT_DELAY_H

// Define the length of the interruptable delay in ms. Default application wants 18 ms.
#define TMR2_INTERRUPT_TICKER_FACTOR    18


// General driver for TMR2 - made with MPLAB
void TMR2_Initialize(void);

void TMR2_StartTimer(void);

void TMR2_StopTimer(void);

void TMR2_ISR(void);


// new functions
void delay_18ms_interruptable(void);

void interrupt_delay(void);



#endif  /* INTERRUPT_DELAY_H */
