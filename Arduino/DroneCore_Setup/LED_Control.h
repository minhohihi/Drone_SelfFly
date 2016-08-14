//
//  LED_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __LED_CONTROL__
#define __LED_CONTROL__

void _LED_SetColor(int nRed, int nGreen, int nBlue);
void _LED_Blink(int nRed, int nGreen, int nBlue, int32_t nLinkPeriod);

void _LED_Initialize()
{
    int             i = 0;

    Serialprintln(F(" *      2. Start LED Module Initialization   "));
    
    // Set Digital Port 7, 12, and 13 as Output
    DDRB |= B00110000;
    DDRD |= B10000000;

    // Set Value of Digital Port 7, 12, and 13 as Low to Initialize LED
    PORTB &= B11001111;
    PORTD &= B01111111;

    _LED_Blink(1, 1, 1, 0);
    
    delay(1000);

    for(i=0 ; i<3 ; i++)
    {
        _LED_Blink(1, 0, 0, 0);
        delay(200);
        _LED_Blink(0, 1, 0, 0);
        delay(200);
        _LED_Blink(0, 0, 1, 0);
        delay(200);
    }

    // Set RED Led as Init Color
    _LED_Blink(1, 0, 0, 0);

    _gPrevBlinkTime = micros();
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _LED_SetColor(int nRed, int nGreen, int nBlue)
{
    // Digital Port 7
    if(0 != nRed)
        PORTD |= B10000000;
    else
        PORTD &= B01111111;

    // Digital Port 13
    if(0 != nGreen)
        PORTB |= B00100000;
    else
        PORTB &= B11011111;

    // Digital Port 12
    if(0 != nBlue)
        PORTB |= B00010000;
    else
        PORTB &= B11101111;
}


void _LED_Blink(int nRed, int nGreen, int nBlue, int32_t nLinkPeriod)
{
    _gCurrTime = micros();

    if(0 == nLinkPeriod)
        _LED_SetColor(nRed, nGreen, nBlue);
    else if((_gCurrTime - _gPrevBlinkTime) > nLinkPeriod)
    {
        if(0 == _gLED_Status)
            _LED_SetColor(nRed, nGreen, nBlue);
        else
            _LED_SetColor(0, 0, 0);

        _gLED_Status = !(_gLED_Status);

        _gPrevBlinkTime = _gCurrTime;
    }
}

#endif /* LED_Controller_h */



