/* 
 * File:   eeprom.h
 * Author: th
 *
 * Created on 3. Januar 2018, 15:11
 */

#ifndef EEPROM_H
#define	EEPROM_H

//define some buffer to do eeprom write in the background
uint8_t eeprom_content_current[256];
uint8_t eeprom_content_towrite[256];

unsigned char readEEPROM(unsigned char address)
{
  EEADR = address; //Address to be read

  EECON1bits.EEPGD = 0;//Selecting EEPROM Data Memory
  EECON1bits.CFGS = 0;//Access EEPROM
  EECON1bits.RD = 1; //Initialise read cycle
  return EEDATA; //Returning data
}


void writeEEPROM(unsigned char address, unsigned char datas)
{  
  while(EECON1bits.WR) //Checking for complition of (last) write operation
  {
    asm ("nop"); //do nothing
  }    
  PIR2bits.EEIF = 0; //clear EEIF bit
  
  EEADR = address; //Address to write
  EEDATA = datas; //Data to write
  EECON1bits.EEPGD = 0; //Selecting EEPROM Data Memory
  EECON1bits.CFGS = 0;//Access EEPROM
  EECON1bits.WREN = 1; //Enable writing of EEPROM  
  INTCONbits.GIE   = 0; //Diables the interrupt  
  //required sequence start
  EECON2 = 0x55; //Required sequence for write to internal EEPROM  
  EECON2 = 0x0AA; //Required sequence for write to internal EEPROM
  EECON1bits.WR = 1; //Initialise write cycle    
  // required sequence end
  INTCONbits.GIE   = 1; //Enables Interrupt  
     
  EECON1bits.WREN = 0; //To disable write
  
}



#endif	/* EEPROM_H */

