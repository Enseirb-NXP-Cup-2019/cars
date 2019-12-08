#include <Servo.h>

Servo esc;          
Servo servoMot;     

void calibrateEsc(){             //at beginning esc must not be powered
  esc.write(180);                      //sending max throttle to esc
  if(Serial.available())               //time to power esc then press any key to continue
    esc.write(0);                      //sending min throttle to esc
}


//sending value v between 0 and 180 corresponding to a certain speed value
void modifySpeed(int v){
  // esc.write(0); //sometimes esc needs this step
  esc.write(v);
}


//updating wheels direction to angle d
//d  is value in [0,180], 90 for straight line, parallel to wheels
void modifyDirection( int d){
   servoMot.write(d);
}
