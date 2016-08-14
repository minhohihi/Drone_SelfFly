#include <Servo.h>

Servo nServo0;
Servo nServo1;

void setup() {
    Serial.begin(115200);
    
    nServo0.attach(8);
    nServo1.attach(9);
}

unsigned int a = 0;
int nAngleH = 0;
int nAngleV = 0;

void loop() {
    //Serial.write(((a++) % 255));
    //Serial.write("a");

    if(Serial.available())
    {
        nAngleH = Serial.read();
        nAngleV = Serial.read();

        if('\n' == Serial.read())
        {          
            //nServo0.write(nAngleH);
            //nServo1.write(nAngleV);

            nAngleH = (int)(map(nAngleH, -45, 45, 1250, 1750));
            nAngleV = (int)(map(nAngleV, -45, 45, 1250, 1750));
            //nAngleH = (int)(map(nAngleH, -45, 45, 1100, 1900));
            //nAngleV = (int)(map(nAngleV, -45, 45, 1100, 1900));
            nServo0.writeMicroseconds(nAngleH);
            nServo1.writeMicroseconds(nAngleV);
            delay(1);
        }
    }
    
    delay(10);
}


//void serialEvent()
//{
//    int nAngle = Serial.parseInt();
//    
//    //if(Serial.read() == '\n')
//    {
//        nServo0.write(nAngle);
//        delay(1);
//    }
//}

