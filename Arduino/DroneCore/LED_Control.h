//
//  LED_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __LED_CONTROL__
#define __LED_CONTROL__

void _LED_SetColor(const int nRed, const int nGreen, const int nBlue, const int bBackUp);

void _LED_Initialize()
{
    int             i = 0;

    // Set Digital Port 7, 12, and 13 as Output
    DDRB |= B00110000;
    DDRD |= B10000000;
    
    // Set Value of Digital Port 7, 12, and 13 as Low to Initialize LED
    PORTB &= B11001111;
    PORTD &= B01111111;
    
    _LED_SetColor(1, 1, 1, 1);

    delay(1000);
    
    for(i=0 ; i<3 ; i++)
    {
        _LED_SetColor(1, 0, 0, 1);
        delay(500);
        _LED_SetColor(0, 1, 0, 1);
        delay(500);
        _LED_SetColor(0, 0, 1, 1);
        delay(500);
    }

    pSelfFlyHndl->nPrevBlinkTime = micros();  
}

void _LED_SetColor(int nRed, int nGreen, int nBlue, const int bBackUp)
{
    // Digital Port 7
    if(0 != nRed)
        PORTD |= B10000000;
    else
        PORTD &= B01111111;
    
    // Digital Port 12
    if(0 != nGreen)
        PORTB |= B00010000;
    else
        PORTB &= B11101111;
    
    // Digital Port 13
    if(0 != nBlue)
        PORTB |= B00100000;
    else
        PORTB &= B11011111;
    
    // Backup Current LED Values
    if(0 != bBackUp)
    {
        pSelfFlyHndl->nPrevR = nRed;
        pSelfFlyHndl->nPrevG = nGreen;
        pSelfFlyHndl->nPrevB = nBlue;
    }
}


void _LED_Blink()
{
    static int          nLED_Status = 0;
    unsigned long       nCurrTime = micros();
    
    if((nCurrTime - pSelfFlyHndl->nPrevBlinkTime) > LED_BLINK_PERIOD)
    {
        if(0 == nLED_Status)
            _LED_SetColor(pSelfFlyHndl->nPrevR, pSelfFlyHndl->nPrevG, pSelfFlyHndl->nPrevB, 1);
        else
            _LED_SetColor(0, 0, 0, 0);
        
        nLED_Status = !(nLED_Status);
        
        pSelfFlyHndl->nPrevBlinkTime = nCurrTime;
    }
}

#endif /* LED_Controller_h */


