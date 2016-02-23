//
//  RC_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __RC_CONTROL__
#define __RC_CONTROL__


void _RC_Initialize()
{
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18);
    PCMSK2 |= (1 << PCINT19);
    PCMSK2 |= (1 << PCINT20);
    PCMSK2 |= (1 << PCINT21);
    PCMSK2 |= (1 << PCINT22);
}


ISR(PCINT2_vect)
{
    const unsigned long     nCurrTime = micros();

    if(true == pSelfFlyHndl->nLock)
        return;

    if(PIND & B00000100)
    {
        if(0 == pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_ROLL])
            pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_ROLL] = nCurrTime;
    }
    else if(0 != pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_ROLL])
    {
        pSelfFlyHndl->nCapturedRCVal[CH_TYPE_ROLL] = nCurrTime - pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_ROLL];
        pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_ROLL] = 0;
    }

    if(PIND & B00001000)
    {
        if(0 == pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_PITCH])
            pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_PITCH] = nCurrTime;
    }
    else if(0 != pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_PITCH])
    {
        pSelfFlyHndl->nCapturedRCVal[CH_TYPE_PITCH] = nCurrTime - pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_PITCH];
        pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_PITCH] = 0;
    }

    if(PIND & B00010000)
    {
        if(0 == pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_THROTTLE])
            pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_THROTTLE] = nCurrTime;
    }
    else if(0 != pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_THROTTLE])
    {
        pSelfFlyHndl->nCapturedRCVal[CH_TYPE_THROTTLE] = nCurrTime - pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_THROTTLE];
        pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_THROTTLE] = 0;
    }

    if(PIND & B00100000)
    {
        if(0 == pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_YAW])
            pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_YAW] = nCurrTime;
    }
    else if(0 != pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_YAW])
    {
        pSelfFlyHndl->nCapturedRCVal[CH_TYPE_YAW] = nCurrTime - pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_YAW];
        pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_YAW] = 0;
    }

    if(PIND & B01000000)
    {
        if(0 == pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_TAKE_LAND])
            pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_TAKE_LAND] = nCurrTime;
    }
    else if(0 != pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_TAKE_LAND])
    {
        pSelfFlyHndl->nCapturedRCVal[CH_TYPE_TAKE_LAND] = nCurrTime - pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_TAKE_LAND];
        pSelfFlyHndl->nRCPrevChangeTime[CH_TYPE_TAKE_LAND] = 0;
    }
}


#endif /* RC_Controller_h */

