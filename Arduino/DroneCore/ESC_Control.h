//
//  ESC_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __ESC_CONTROL__
#define __ESC_CONTROL__

inline void _UpdateESCs();

void _ESC_Initialize()
{
    unsigned long           *pThrottle = &(pSelfFlyHndl->nThrottle[0]);
    int                     i = 0;     
    
    // Set Digital Port 8, 9, 10, and 11 as Output
    DDRB |= B00001111;
    
    delay(100);

    // Set Value of Digital Port 8, 9, 10, and 11 as Low
    PORTB &= B11110000;
    
    // Set Value of Digital Port 8, 9, 10, and 11 as Minimun ESC to Initialize ESC
    for(i=0 ; i<30 ; i++)
    {
        pThrottle[0] = ESC_MIN;
        pThrottle[1] = ESC_MIN;
        pThrottle[2] = ESC_MIN;
        pThrottle[3] = ESC_MIN;   
        _UpdateESCs();
        delay(10);    
    }    

    // Set Value of Digital Port 8, 9, 10, and 11 as Low
    PORTB &= B11110000;
}

inline void _UpdateESCs()
{
    unsigned long           *pThrottle = &(pSelfFlyHndl->nThrottle[0]);
    int                     i = 0;
    unsigned long           nLoopTimer = 0;
    //while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
    //loop_timer = micros();                                                    //Set the timer for the next loop.
    
    nLoopTimer = micros();
    
    PORTB |= B00001111;                                                         //Set Digital Port 8, 9, 10, and 11 as high.
    
    // Set Relative Throttle Value by Adding Current Time
    for(i=0 ; i<MAX_CH_ESC ; i++)
        pThrottle[i] += nLoopTimer;

    while(PORTB & B00001111)
    {
        const unsigned long     nCurrTime = micros();
        
        if(pThrottle[0] <= nCurrTime)
            PORTB &= B11111110;
        
        if(pThrottle[1] <= nCurrTime)
            PORTB &= B11111101;
        
        if(pThrottle[2] <= nCurrTime)
            PORTB &= B11111011;
        
        if(pThrottle[3] <= nCurrTime)
            PORTB &= B11110111;
    }
}

#endif /* ESC_Controller_h */

