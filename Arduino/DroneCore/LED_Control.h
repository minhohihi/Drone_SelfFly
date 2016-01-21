//
//  LED_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __LED_CONTROL__
#define __LED_CONTROL__

static int      nPrevR = 0;
static int      nPrevG = 0;
static int      nPrevB = 0;

void _LED_SetColor(int nRed, int nGreen, int nBlue);

void _LED_Initialize()
{
    int             i = 0;

    _LED_SetColor(0, 0, 0);

    delay(1000);
    
    for(i=0 ; i<3 ; i++)
    {
        _LED_SetColor(1, 0, 0);
        delay(500);
        _LED_SetColor(0, 1, 0);
        delay(500);
        _LED_SetColor(0, 0, 1);
        delay(500);
    }
}

void _LED_SetColor(int nRed, int nGreen, int nBlue)
{
    // Digital Port 7
    if(0 != nRed)
        PORTD |= B10000000;
    else
        PORTD &= B01111111;
    
    // Digital Port 12
    if(0 != nGreen)
        PORTB |= B01000000;
    else
        PORTB &= B10111111;
    
    // Digital Port 13
    if(0 != nBlue)
        PORTB |= B10000000;
    else
        PORTB &= B01111111;
    
    //analogWrite(PIN_LED_RED, nRed);
    //analogWrite(PIN_LED_GREEN, nGreen);
    //analogWrite(PIN_LED_BLUE, nBlue);
    
    // Backup Current LED Values
    nPrevR = nRed;
    nPrevG = nGreen;
    nPrevB = nBlue;
}


void _LED_Blink()
{
    static int          nLED_Status = 0;
    unsigned long       nCurrTime = micros();
    
    if((nCurrTime - pSelfFlyHndl->nPrevBlinkTime) > LED_BLINK_PERIOD)
    {
        if(0 == nLED_Status)
            _LED_SetColor(nPrevR, nPrevG, nPrevB);
        else
            _LED_SetColor(0, 0, 0);
        
        nLED_Status = !(nLED_Status);
        
        pSelfFlyHndl->nPrevBlinkTime = nCurrTime;
    }
}

#endif /* LED_Controller_h */


