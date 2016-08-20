//
//  LCD_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 8. 17..
//

#ifndef __LCD_CONTROL__
#define __LCD_CONTROL__

void _LCD_Initialize()
{
    #if USE_LCD_DISPLAY
    Serialprintln(F(" *      2. Start LCD Module Initialization   "));
    
    _gLCDHndl.begin(16, 2, 0);                                                 //Initialize the LCD
    _gLCDHndl.backlight();                                                     //Activate backlight
    _gLCDHndl.clear();                                                         //Clear the LCD
    
    _gLCDHndl.setCursor(0,0);                                                  //Set the LCD cursor to position to position 0,0
    _gLCDHndl.print(" Maverick Drone");                                        //Print text to screen
    _gLCDHndl.setCursor(0,1);                                                  //Set the LCD cursor to position to position 0,1
    _gLCDHndl.print("      V1.0");                                             //Print text to screen
    
    delay(1500);                                                               //Delay 1.5 second to display the text
    _gLCDHndl.clear();                                                         //Clear the LCD
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
    #endif
}


void _LCD_Clear()
{
    #if USE_LCD_DISPLAY
    delay(100);                                                               //Delay 1.5 second to display the text
    _gLCDHndl.clear();                                                         //Clear the LCD  
    delay(100);                                                               //Delay 1.5 second to display the text
    #endif
}


#endif /* LCD_Controller_h */




