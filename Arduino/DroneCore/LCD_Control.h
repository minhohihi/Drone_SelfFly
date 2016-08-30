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


void _LCD_DispDissolveClear(const int nLoopCnt)
{
    static int nLastPos = 0;
    
    if( 0 == (nLoopCnt % 6)){nLastPos = (nLoopCnt / 6); _gLCDHndl.setCursor(nLastPos, 0);}
    if( 1 == (nLoopCnt % 6))_gLCDHndl.print(" ");
    if( 2 == (nLoopCnt % 6))_gLCDHndl.print(" ");
    if( 3 == (nLoopCnt % 6))_gLCDHndl.setCursor(nLastPos, 1);
    if( 4 == (nLoopCnt % 6))_gLCDHndl.print(" ");
    if( 5 == (nLoopCnt % 6))_gLCDHndl.print(" ");
}

                                  
void _LCD_DispRPY(const int nLoopCnt)
{
    static int nVal;
    
         if( 0 == nLoopCnt){nVal = _gAngleRoll * 10; _gLCDHndl.setCursor(0, 0);}
    else if( 1 == nLoopCnt)_gLCDHndl.print("R");
    else if( 2 == nLoopCnt){if(nVal < 0) _gLCDHndl.print("-"); else _gLCDHndl.print("+");}
    else if( 3 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if( 4 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if( 5 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if( 6 == nLoopCnt)_gLCDHndl.print(".");
    else if( 7 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);

    else if( 8 == nLoopCnt){nVal = _gAnglePitch * 10; _gLCDHndl.setCursor(8, 0);}
    else if( 9 == nLoopCnt)_gLCDHndl.print("P");
    else if(10 == nLoopCnt){if(nVal < 0) _gLCDHndl.print("-"); else _gLCDHndl.print("+");}
    else if(11 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(12 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(13 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(14 == nLoopCnt)_gLCDHndl.print(".");
    else if(15 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);

    else if(16 == nLoopCnt){nVal = _gEstYaw * 10; _gLCDHndl.setCursor(0, 1);}
    else if(17 == nLoopCnt)_gLCDHndl.print("Y");
    else if(18 == nLoopCnt){if(nVal < 0) _gLCDHndl.print("-"); else _gLCDHndl.print("+");}
    else if(19 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(20 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(21 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(22 == nLoopCnt)_gLCDHndl.print(".");
    else if(23 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);

    else if(24 == nLoopCnt)_gLCDHndl.setCursor(9, 1);
    else if(25 == nLoopCnt)_gLCDHndl.print(" ");
    else if(26 == nLoopCnt)_gLCDHndl.print("R");
    else if(27 == nLoopCnt)_gLCDHndl.print("P");
    else if(28 == nLoopCnt)_gLCDHndl.print("Y");
    else if(29 == nLoopCnt)_gLCDHndl.print(" ");
}


void _LCD_DispMag(const int nLoopCnt)
{
    static int nVal;

         if( 0 == nLoopCnt){nVal = (int)(_gRawMag[X_AXIS]); _gLCDHndl.setCursor(0, 0);}
    else if( 1 == nLoopCnt)_gLCDHndl.print("X:");
    else if( 2 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if( 3 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if( 4 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if( 5 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);
    
    else if( 6 == nLoopCnt){nVal = (int)(_gRawMag[Y_AXIS]); _gLCDHndl.setCursor(8, 0);}
    else if( 7 == nLoopCnt)_gLCDHndl.print("Y:");
    else if( 8 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if( 9 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(10 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(11 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);
    
    else if(12 == nLoopCnt){nVal = (int)(_gRawMag[Z_AXIS]); _gLCDHndl.setCursor(0, 1);}
    else if(13 == nLoopCnt)_gLCDHndl.print("Z:");
    else if(14 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(15 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(16 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(17 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);
    
    else if(18 == nLoopCnt)_gLCDHndl.setCursor(9, 1);
    else if(19 == nLoopCnt)_gLCDHndl.print(" ");
    else if(20 == nLoopCnt)_gLCDHndl.print("M");
    else if(21 == nLoopCnt)_gLCDHndl.print("a");
    else if(22 == nLoopCnt)_gLCDHndl.print("g");
    else if(23 == nLoopCnt)_gLCDHndl.print(" ");
}


void _LCD_DispThrottle(const int nLoopCnt)
{
    static int nVal;
    
         if( 0 == nLoopCnt){nVal = _gESCOutput[0]; _gLCDHndl.setCursor(0, 0);}
    else if( 1 == nLoopCnt)_gLCDHndl.print("T");
    else if( 2 == nLoopCnt)_gLCDHndl.print("0:");
    else if( 3 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if( 4 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if( 5 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if( 6 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);
    
    else if( 7 == nLoopCnt){nVal = _gESCOutput[1]; _gLCDHndl.setCursor(9, 0);}
    else if( 8 == nLoopCnt)_gLCDHndl.print("T");
    else if( 9 == nLoopCnt)_gLCDHndl.print("1:");
    else if(10 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(11 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(12 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(13 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);
    
    else if(14 == nLoopCnt){nVal = _gESCOutput[2]; _gLCDHndl.setCursor(0, 1);}
    else if(15 == nLoopCnt)_gLCDHndl.print("T");
    else if(16 == nLoopCnt)_gLCDHndl.print("2:");
    else if(17 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(18 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(19 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(20 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);

    else if(21 == nLoopCnt){nVal = _gESCOutput[3]; _gLCDHndl.setCursor(9, 1);}
    else if(22 == nLoopCnt)_gLCDHndl.print("T");
    else if(23 == nLoopCnt)_gLCDHndl.print("3:");
    else if(24 == nLoopCnt)_gLCDHndl.print(abs(nVal) / 1000);
    else if(25 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 100) % 10);
    else if(26 == nLoopCnt)_gLCDHndl.print((abs(nVal) / 10) % 10);
    else if(27 == nLoopCnt)_gLCDHndl.print(abs(nVal) % 10);
}
        

void _LCD_DispInfo()
{
    static int          nDispCnt = 0;
    
    if(3300 == nDispCnt)
        nDispCnt = 0;
    
    if((nDispCnt >= 0) && (nDispCnt < 1000))
        _LCD_DispRPY(nDispCnt % 30);
    
    if((nDispCnt >= 1000) && (nDispCnt < 1100))
        _LCD_DispDissolveClear((nDispCnt - 1000) % 96);
    
    if((nDispCnt >= 1100) && (nDispCnt < 2100))
        _LCD_DispThrottle((nDispCnt - 1100) % 28);

    if((nDispCnt >= 2100) && (nDispCnt < 2200))
        _LCD_DispDissolveClear((nDispCnt - 2100) % 96);
    
    if((nDispCnt >= 2200) && (nDispCnt < 3200))
        _LCD_DispMag((nDispCnt - 2200) % 30);
    
    if((nDispCnt >= 3200) && (nDispCnt < 3300))
        _LCD_DispDissolveClear((nDispCnt - 3200) % 96);

    nDispCnt++;
}
#endif /* LCD_Controller_h */




