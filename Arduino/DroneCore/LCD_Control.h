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


void _LCD_DispDissolveClear(const int nLooCnt, const int nOffset)
{
    static int nLastPos = 0;
    const int nLocalDispCnt = nLooCnt - nOffset;
    
    if( 0 == (nLocalDispCnt % 4)){nLastPos = (nLocalDispCnt / 4); _gLCDHndl.setCursor(nLastPos, 0);}
    if( 1 == (nLocalDispCnt % 4))_gLCDHndl.print(" ");
    if( 2 == (nLocalDispCnt % 4))_gLCDHndl.setCursor(nLastPos, 1);
    if( 3 == (nLocalDispCnt % 4))_gLCDHndl.print(" ");
}

                                  
void _LCD_DispRPY(const int nLooCnt, const int nOffset)
{
    static int nVal;
    const int nLocalDispCnt = nLooCnt - nOffset;
    
         if( 0 == nLocalDispCnt){nVal = _gAngleRoll * 10; _gLCDHndl.setCursor(0, 0);}
    else if( 1 == nLocalDispCnt)_gLCDHndl.print("R");
    else if( 2 == nLocalDispCnt){if(nVal < 0) _gLCDHndl.print("-"); else _gLCDHndl.print("+");}
    else if( 3 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if( 4 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if( 5 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if( 6 == nLocalDispCnt)_gLCDHndl.print(".");
    else if( 7 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) % 10);

    else if( 8 == nLocalDispCnt){nVal = _gAnglePitch * 10; _gLCDHndl.setCursor(8, 0);}
    else if( 9 == nLocalDispCnt)_gLCDHndl.print("P");
    else if(10 == nLocalDispCnt){if(nVal < 0) _gLCDHndl.print("-"); else _gLCDHndl.print("+");}
    else if(11 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(12 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(13 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(14 == nLocalDispCnt)_gLCDHndl.print(".");
    else if(15 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) % 10);

    else if(16 == nLocalDispCnt){nVal = _gEstYaw * 10; _gLCDHndl.setCursor(0, 1);}
    else if(17 == nLocalDispCnt)_gLCDHndl.print("Y");
    else if(18 == nLocalDispCnt){if(nVal < 0) _gLCDHndl.print("-"); else _gLCDHndl.print("+");}
    else if(19 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(20 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(21 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(22 == nLocalDispCnt)_gLCDHndl.print(".");
    else if(23 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) % 10);
}


void _LCD_DispThrottle(const int nLooCnt, const int nOffset)
{
    static int nVal;
    const int nLocalDispCnt = nLooCnt - nOffset;
    
         if( 0 == nLocalDispCnt){nVal = _gESCOutput[0]; _gLCDHndl.setCursor(0, 0);}
    else if( 1 == nLocalDispCnt)_gLCDHndl.print("T");
    else if( 2 == nLocalDispCnt)_gLCDHndl.print("0:");
    else if( 3 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if( 4 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if( 5 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if( 6 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) % 10);
    
    else if( 7 == nLocalDispCnt){nVal = _gESCOutput[1]; _gLCDHndl.setCursor(9, 0);}
    else if( 8 == nLocalDispCnt)_gLCDHndl.print("T");
    else if( 9 == nLocalDispCnt)_gLCDHndl.print("1:");
    else if(10 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(11 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(12 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(13 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) % 10);
    
    else if(14 == nLocalDispCnt){nVal = _gESCOutput[2]; _gLCDHndl.setCursor(0, 1);}
    else if(15 == nLocalDispCnt)_gLCDHndl.print("T");
    else if(16 == nLocalDispCnt)_gLCDHndl.print("2:");
    else if(17 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(18 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(19 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(20 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) % 10);

    else if(21 == nLocalDispCnt){nVal = _gESCOutput[3]; _gLCDHndl.setCursor(9, 1);}
    else if(22 == nLocalDispCnt)_gLCDHndl.print("T");
    else if(23 == nLocalDispCnt)_gLCDHndl.print("3:");
    else if(24 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(25 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(26 == nLocalDispCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(27 == nLocalDispCnt)_gLCDHndl.print(abs(nVal) % 10);
}
        

void _LCD_DispInfo()
{
    static int          nDispCnt = 0;
    
    if(180 == nDispCnt)
        nDispCnt = 0;
    
    if((nDispCnt >= 0) && (nDispCnt < 50))
        _LCD_DispRPY(nDispCnt, 0);
    
    if((nDispCnt >= 50) && (nDispCnt < 100))
        _LCD_DispDissolveClear(nDispCnt, 50);
    
    if((nDispCnt >= 100) && (nDispCnt < 150))
        _LCD_DispThrottle(nDispCnt, 100);

    if((nDispCnt >= 150) && (nDispCnt < 200))
        _LCD_DispDissolveClear(nDispCnt, 150);

    nDispCnt++;
}
#endif /* LCD_Controller_h */




