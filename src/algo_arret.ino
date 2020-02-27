#include <Servo.h>
#include <stdbool.h>
#include <stdint.h>
#include <Arduino.h>
#include "TeensyThreads.h"


// HARDWARE GLOBALS ------------------------------------------------------------
static Servo esc;
static Servo servo_motor;

volatile int liDARval = 0;
volatile int strength = 0;


// SETUP -----------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  servo_motor.attach(10, 1000, 2000); // pin 10 -> servomotor
  esc.attach(6);                      // pin 6 -> ESC
  Serial.begin(115200);
  Serial.print("Starting...\n");
  calibrage();
}


void calibrage(){
  delay(5000);
  esc.write(2000);
  delay(5000);
  esc.write(0);
  delay(5000);
  for (int i=0;i<100;i++){
    esc.write(1500+i);
    delay(50);
    Serial.println(1500+i);
  }
}



void loop()
{
   delay(10);  // Don't want to read too often as TFmini samples at 100Hz

   if(Serial1.available()>=9) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
    {
      if((0x59 == Serial1.read()) && (0x59 == Serial1.read())) // byte 1 and byte 2
      {
        unsigned int t1 = Serial1.read(); // byte 3 = Dist_L
        unsigned int t2 = Serial1.read(); // byte 4 = Dist_H
        
        t2 <<= 8;
        t2 += t1;
        liDARval = t2;
        
        t1 = Serial1.read(); // byte 5 = Strength_L
        t2 = Serial1.read(); // byte 6 = Strength_H

        t2 <<= 8;
        t2 += t1;
        strength = t2;
        for(int i=0; i<3; i++)Serial1.read(); // byte 7, 8, 9 are ignored
      }
    //  Serial.println("Distance =");
     // Serial.println(liDARval);
      //Serial.println("Strength =");
     // Serial.println(strength);
     if(liDARval<50){
        Serial.println("Arret");
       Serial.println(liDARval);
       esc.write(0);
       }
    }

}
