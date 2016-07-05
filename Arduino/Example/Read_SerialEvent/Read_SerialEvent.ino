#include <Servo.h>

Servo nServo0;

void setup() {
    Serial.begin(9600);
    
    nServo0.attach(8);
}

unsigned int a = 0;
int nAngle = 0;

void loop() {
    //Serial.write(((a++) % 255));
    //Serial.write("a");

    if(Serial.available())
    {
        nAngle = Serial.read();
        nServo0.write(nAngle);
        delay(1);        
    }
    
    delay(5);
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

