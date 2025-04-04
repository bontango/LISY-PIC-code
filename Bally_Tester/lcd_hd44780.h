/* 
 * File:   lcd_hd44780.h
 * Author: th
 * Version 100
 * 10.10.2019
 */

#ifndef LCD_44780_H
#define	LCD_44780_H

//connected ports
#define LCD_DATA_4 PORTBbits.RB4
#define LCD_DATA_5 PORTBbits.RB5
#define LCD_DATA_6 PORTBbits.RB6
#define LCD_DATA_7 PORTBbits.RB7
#define LCD_BUSY_BIT LATBbits.LB7
#define LCD_DATA_TRIS_BIT TRISBbits.RB7

//#define LCD_DATA PORTC
//#define LCD_DATA_TRIS TRISC
#define LCD_RS PORTBbits.RB0
#define LCD_RW PORTBbits.RB2
#define LCD_E PORTBbits.RB3

/****************** constants for display commands ************************************/
#define LCD_CMD_CLR 0x01 //Clear display 	0x01
//Return cursor to home, and un-shift display 	0x02
//Entry mode: The following control how the cursor behaves after each character is entered 	
//move cursor right, don?t shift display 	0x04
//move cursor right, do shift display (left) 	0x05
#define LCD_CMD_ENTRY_INC 0x06 //move cursor right, don?t shift display (this is the most common) 	0x06
//move cursor right, do shift display (left) 	0x07
//Display control: The following control display properties 	
//turn display off 	0x08
//display on, cursor off, 	0x0C
//display on, cursor on, steady cursor 	0x0E
#define LCD_CMD_CUR_BLINK 0x0F //display on, cursor on, blinking cursor 	0x0F
//Shift cursor left 	0x10
//Shift cursor right 	0x14
//Shift display left 	0x18
//Shift display right 	0x1C


// Display loeschen
#define LCD_CMD_ON_C 0x0E // Cursor home
#define LCD_CMD_FUNCTION_SET 0x28 // 4 Bit, 2 Zeilen, 5x7
#define LCD_CMD_DISP_ON 0x0C // Display Ein
#define LCD_CMD_DISP_OFF 0x08 // Display Aus


/****************** Funktionsprototypen *************************************************/
void LCD_INIT(void);
void LCD_CHAR(unsigned char mychar);
void LCD_CMD(unsigned char befehl);
void LCD_STRING(const char *pString);
void LCD_GotoXY(uint8_t x,uint8_t y);

//typedefs
typedef union two {
    unsigned char byte;
    struct {
    unsigned FREE:4, DATA_4:1, DATA_5:1, DATA_6:1, DATA_7:1;
    //signed b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
        } bitv;
    } my_byte_t;
    

#endif	/* LCD_44780_H*/