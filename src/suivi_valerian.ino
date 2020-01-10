#include <Pixy2.h>
#include <Servo.h>
#include <math.h>
#include <stdint.h>

#define PI 3.14159265

Pixy2 pixy;
Servo motA;
Servo servo_mot;

uint8_t max(uint8_t a, uint8_t b) { return a > b ? a : b; }

#define NB_FRAMES 10
double lastAngles[NB_FRAMES];
uint8_t lastAngle_index = 0;

#define MAX_DEVIATION 90

void setup() {
  
  Serial.begin(9600);
  servo_mot.attach(10, 1000, 2000);
  motA.attach(6);
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // we need to initialize the pixy object
  pixy.init();
  // Change to line tracking program
  pixy.changeProg("line");

  motA.write(0);
  delay(2000);
  motA.write(100);
  for(int i = 0; i < NB_FRAMES; i++){
    lastAngles[i] = 90.0;
  }
}

double amortissement_reponse(double x) { return x * x / 180.0; }

double average(double* array, uint8_t size) {
  double sum = 0;
  for (size_t i = 0; i < size; i++) {
    sum += array[i];
  }
  return sum / size;
}

void loop() {
  pixy.line.getAllFeatures();
  double sumAngles = 0.;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double angle =
        angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                    pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);
    double diff = abs(90 - angle);
    if(diff <= MAX_DEVIATION){
      sumAngles += angle;
    }
    Serial.print("Angle vecteur:");
                  Serial.print(angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                    pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1));
                    Serial.print("\n"); 
                    
  }
  sumAngles /= pixy.line.numVectors;
  Serial.print("Moyenne:");
  Serial.print(sumAngles);
  Serial.print("\n");
  lastAngles[lastAngle_index] = sumAngles;
  lastAngle_index             = (lastAngle_index + 1) % NB_FRAMES;
  servo_mot.write(average(lastAngles, NB_FRAMES));
  //servo_mot.write(90);
  Serial.print("Direction prise:");
  Serial.print(average(lastAngles, NB_FRAMES));
  Serial.print("\n");
  delay(100);
}
/*
// degrees
double angle_servo(int x0, int y0, int x1, int y1) {
  return atan2((y1 - y0) * 78.0 / 51.0, x1 - x0) * 180.0 / PI;
}*/


float coeff_dir(float x0, float x1, float y0, float y1){
  return (y1-y0) * 78.0 / 51.0 /(x1-x0);
}


int angle_servo(int x0, int y0, int x1, int y1){
  if (x1-x0==0) {
    return 90;
  }
  else {
    float r = coeff_dir(x0,x1,y0,y1);
    
    if (r <= -1){
      int a= 10+80*(1-exp(1+r));
      //Serial.println(a); 
      return a;
    }
    
    else if (r >= 1){
      int b=170-80*(1-exp(-r+1));
      //Serial.println(b);
      return b;
    }

    else if (r > 0) {
      return 170;
    }

    else {
      return 10;
    }
  }
  
}
