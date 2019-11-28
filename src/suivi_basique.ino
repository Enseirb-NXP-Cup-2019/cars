#include <Servo.h>
#include <Pixy2.h>

Pixy2 pixy;
Servo motA;
Servo servo_mot;
char data;




void setup() {
  
  Serial.begin(9600);
  servo_mot.attach(10, 1000, 2000);
  
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // we need to initialize the pixy object
  pixy.init();
  // Change to line tracking program
  pixy.changeProg("line");

}






void loop() {

  float x0;
  float x1;
  float y0;
  float y1;
  float x2;
  float x3;
  float y2;
  float y3;
  int8_t i;
  char buf[128];
  pixy.line.getAllFeatures();
  
  int nb_vect=min(pixy.line.numVectors,2);
  Serial.println(nb_vect);
  
  switch (nb_vect){
    case 1:
    x0=(float)pixy.line.vectors[0].m_x0;
    x1=(float)pixy.line.vectors[0].m_x1;
    y0=(float)pixy.line.vectors[0].m_y0;
    y1=(float)pixy.line.vectors[0].m_y1;
    
    servo_mot.write(angle_servo(x0,x1,y0,y1));
  
    case 2:
    x0=(float)pixy.line.vectors[0].m_x0;
    x1=(float)pixy.line.vectors[0].m_x1;
    y0=(float)pixy.line.vectors[0].m_y0;
    y1=(float)pixy.line.vectors[0].m_y1;
    x2=(float)pixy.line.vectors[1].m_x0;
    x3=(float)pixy.line.vectors[1].m_x1;
    y2=(float)pixy.line.vectors[1].m_y0;
    y3=(float)pixy.line.vectors[1].m_y1;

    
    float ag1 = angle_servo(x0,x1,y0,y1);
    float ag2 = angle_servo(x2,x3,y2,y3);
    Serial.println(ag1);
    servo_mot.write(ag1);
  }
  
  delay(100);
/*
  Serial.println("yo");
  motA.write(60);
  delay(2000);
  motA.write(90);
  delay(2000);
  motA.write(120);
  delay(2000);
  motA.write(90);
  delay(2000);*/
}









int angle_servo(float x0, float x1, float y0, float y1){
  if (x1-x0==0) {
    return 90;
  }
  else {
    float r=(y1-y0)/(x1-x0);
    
    if (r>=1){
      int a= 30+60*(1-exp(1-r));
      //Serial.println(a); 
      return a;
    }
    
    else if (r<=-1){
      int b=150-60*(1-exp(r+1));
      //Serial.println(b);
      return b;
    }

    else if (0<r) {
      return 30;
    }

    else {
      return 150;
    }
  }
}

int avg(int a, int b){
  return (a+b)/2;
}

void test()
{
    motA.write(100);
    delay(200);
    motA.write(109);
    delay(2000);
    motA.write(0);
}
