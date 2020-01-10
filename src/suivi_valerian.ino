#include <Pixy2.h>
#include <Servo.h>
#include <math.h>
#include <stdint.h>

#define PI 3.14159265

Pixy2 pixy;
Servo motA;
Servo servo_mot;
char data;

uint8_t max(uint8_t a, uint8_t b) { return a > b ? a : b; }

#define NB_FRAMES 10
double lastAngles[NB_FRAMES];
uint8_t lastAngle_index = 0;

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
  double sumAngles = 0;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    sumAngles += angle_servo(
        pixy.line.vectors[bestLine].m_x0, pixy.line.vectors[bestLine].m_y0,
        pixy.line.vectors[bestLine].m_x1, pixy.line.vectors[bestLine].m_y1);
  }
  sumAngles /= pixy.line.numVectors;
  lastAngles[lastAngle_index] = sumAngles;
  lastAngle_index             = (lastAngle_index + 1) % NB_FRAMES;
  servo_mot.write(average(lastAngles, NB_FRAMES));
}
// degrees
double angle_servo(int x0, int y0, int x1, int y1) {
  return atan2(y1 - y0, x1 - x0) * 180 / PI;
}
