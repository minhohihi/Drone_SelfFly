#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <memory.h>
#include <pthread.h>
#include "cv.h"
#include "highgui.h"

#include <OpenGL/gl.h> // OpenGL framework on Mac (version 1.5.10.7.0)
//#include <GL/gl.h> // OpenGL library on Windows

#include <GLUT/glut.h> // GLUT framework on Mac (version 3.4.2)
//#include <GL/glut.h> // GLUT library on Windows

#define IMG_WIDTH                       (720)
#define IMG_HEIGHT                      (401)
#define ESTIMATE_ARR_SIZE               (300)
#define AXIS_X_OFFSET                   (5)
#define AXIS_X_START                    (60)
#define AXIS_Y_START                    (200)
#define AXIS_X_LEN                      (600 + AXIS_X_OFFSET)
#define AXIS_Y_LEN                      (180)


int                         nCoeffiPID[3] = {4500, 1000, 1000};
int                         nInitVal = 150;
int                         nTargetVal = 180;
float                       nEstimatedVal[ESTIMATE_ARR_SIZE];
IplImage                    *pImg = NULL;

void _PIDControl_Cb(int pos);
static void _Get_EstimatedVal(float *pDst, const float nTargetVal, const float nInitVal, const float nPCoeffi, const float nICoeffi, const float nDCoeffi);
static void _DrawGraph(IplImage *pImg, float *pDst);
static void _DrawAxis(IplImage *pImg);


int main(int argc, char * argv[])
{
    pImg = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, 3);
    
    cvNamedWindow("PID Control Simulator", 0);
    cvResizeWindow("PID Control Simulator", IMG_WIDTH, IMG_HEIGHT);
    
    cvCreateTrackbar(" P_Coeffi", "PID Control Simulator", &(nCoeffiPID[0]), 10000, _PIDControl_Cb);
    cvCreateTrackbar(" D_Coeffi", "PID Control Simulator", &(nCoeffiPID[2]), 10000, _PIDControl_Cb);
    cvCreateTrackbar(" I_Coeffi", "PID Control Simulator", &(nCoeffiPID[1]), 10000, _PIDControl_Cb);
    cvCreateTrackbar("  InitVal", "PID Control Simulator", &nInitVal, 360, _PIDControl_Cb);
    cvCreateTrackbar("TargetVal", "PID Control Simulator", &nTargetVal, 360, _PIDControl_Cb);
    
    _PIDControl_Cb(0);
    
    cvWaitKey(0);
    
    cvDestroyWindow("PID Control Simulator");
    
    return 0;
}


void _PIDControl_Cb(int pos)
{
    
    _Get_EstimatedVal(&(nEstimatedVal[0]), (const float)(nTargetVal - 180), (const float)(nInitVal - 180),
                      (const float)((float)nCoeffiPID[0] / 5000.0f), (const float)((float)nCoeffiPID[1] / 5000.0f), (const float)((float)nCoeffiPID[2] / 500000.0f));
    
    _DrawAxis(pImg);
    _DrawGraph(pImg, &(nEstimatedVal[0]));
    
    cvShowImage("PID Control Simulator", pImg);
}


static void _Get_EstimatedVal(float *pDst, const float nTargetVal, const float nInitVal, const float nPCoeffi, const float nICoeffi, const float nDCoeffi)
{
    int                         i = 0;
    const float                 nLoopDuration = 0.01f;
    const float                 nFreq = 1.0f / nLoopDuration;
    float                       nCurrVal = nInitVal;
    float                       nPrevErr = 0.0f;
    float                       nCurrErr = nTargetVal - nCurrVal;
    float                       nEstimated[3] = {0, 0, 0};
    
    memset(pDst, 0, ESTIMATE_ARR_SIZE * sizeof(float));

    printf("%03.05f     %03.05f     %03.05f     %03.05f     %03.05f         \n", nTargetVal, nInitVal, nPCoeffi, nICoeffi, nDCoeffi);
    
    pDst[0] = nCurrVal;
    for(i=1 ; i<ESTIMATE_ARR_SIZE ; i++)
    {
        nEstimated[0] = nPCoeffi * nCurrErr;
        
        nEstimated[1] = nEstimated[1] + nICoeffi * nCurrErr * nLoopDuration;
        
        nEstimated[2] = nDCoeffi * (nCurrErr - nPrevErr) * nFreq;
        
        nCurrVal += (nEstimated[0] + nEstimated[1] + nEstimated[2]);
        nPrevErr = nCurrErr;
        nCurrErr = nTargetVal - nCurrVal;
        
        pDst[i] = nCurrVal;
    }
}


static void _DrawGraph(IplImage *pImg, float *pDst)
{
    int                         i = 0;
    CvScalar                    nScalar = {255, 255, 0, 0};
    int                         nScale = 10;
    
    for(i=0 ; i<(ESTIMATE_ARR_SIZE-1)/nScale ; i++)
    {
        CvPoint                 nStartPos = {AXIS_X_START+AXIS_X_OFFSET + (i * 2 * nScale), AXIS_Y_START + (pDst[i] * nScale)};
        CvPoint                 nEndPos = {nStartPos.x + (2 * nScale), AXIS_Y_START + (pDst[i+1] * nScale)};
        
        cvLine(pImg, nStartPos, nEndPos, nScalar, 1, 8, 0);
    }
}


static void _DrawAxis(IplImage *pImg)
{
    CvScalar                nScalar = {255, 255, 255, 255};
    CvPoint                 nStartPos = {0, 0};
    CvPoint                 nEndPos = {0, 0};
    
    memset(pImg->imageData, 0, pImg->imageSize);
    
    // Draw X-Axis
    nStartPos.x = AXIS_X_START;
    nStartPos.y = AXIS_Y_START;
    nEndPos.x = AXIS_X_START+AXIS_X_LEN;
    nEndPos.y = AXIS_Y_START;
    cvLine(pImg, nStartPos, nEndPos, nScalar, 1, 8, 0);

    // Draw Y-Axis
    nStartPos.x = AXIS_X_START + AXIS_X_OFFSET;
    nStartPos.y = AXIS_Y_START;
    nEndPos.x = AXIS_X_START + AXIS_X_OFFSET;
    nEndPos.y = AXIS_Y_START - AXIS_Y_LEN;
    cvLine(pImg, nStartPos, nEndPos, nScalar, 1, 8, 0);
    nEndPos.x = AXIS_X_START + AXIS_X_OFFSET;
    nEndPos.y = AXIS_Y_START + AXIS_Y_LEN;
    cvLine(pImg, nStartPos, nEndPos, nScalar, 1, 8, 0);
}









