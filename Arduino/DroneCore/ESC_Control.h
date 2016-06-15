//
//  ESC_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __ESC_CONTROL__
#define __ESC_CONTROL__

void _ESC_Initialize()
{
    int                     i = 0;

    Serialprintln(F(" *      3. Start ESC Module Initialization   "));

    // Set Digital Port 8, 9, 10, and 11 as Output
    DDRB |= B00001111;

    delay(100);

    // Set Value of Digital Port 8, 9, 10, and 11 as Low
    PORTB &= B11110000;

    // Set Value of Digital Port 8, 9, 10, and 11 as Minimun ESC to Initialize ESC For Two Seconds
    for(i=0 ; i<500 ; i++)
    {
        // Set Digital Port 8, 9, 10, and 11 as high.
        PORTB |= B00001111;
        delayMicroseconds(1000);

        //Set digital poort 8, 9, 10, and 11 low.
        PORTB &= B11110000;
        delayMicroseconds(3000);
    }
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _UpdateESCs()
{
    unsigned long           nESCOut[4] = {0, };
    int                     i = 0;
    
    // Wait Until Passing 4ms.
    while(micros() - nESCLoopTimer < 4000);

    // Set the timer for the next loop.
    nESCLoopTimer = micros();

    // Set Digital Port 8, 9, 10, and 11 as high.
    PORTB |= B00001111;

    // Set Relative Throttle Value by Adding Current Time
    nESCOut[0] = nESCOutput[0] + nESCLoopTimer;
    nESCOut[1] = nESCOutput[1] + nESCLoopTimer;
    nESCOut[2] = nESCOutput[2] + nESCLoopTimer;
    nESCOut[3] = nESCOutput[3] + nESCLoopTimer;
    
    while(PORTB & B00001111)
    {
        nCurrTime = micros();

        if(nESCOut[0] <= nCurrTime)
            PORTB &= B11111110;

        if(nESCOut[1] <= nCurrTime)
            PORTB &= B11111101;

        if(nESCOut[2] <= nCurrTime)
            PORTB &= B11111011;

        if(nESCOut[3] <= nCurrTime)
            PORTB &= B11110111;
    }
}

#endif /* ESC_Controller_h */

