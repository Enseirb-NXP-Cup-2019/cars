#include <Pixy2.h>
#include <Servo.h>
#include <math.h>
#include <stdint.h>

#define PI 3.14159265

Pixy2 pixy;
Servo motA;
Servo servo_mot;

uint8_t max(uint8_t a, uint8_t b) { return a > b ? a : b; }

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

static double lastAngle = 90.0;

double amortissement_reponse(double x) { return x * x / 180.0; }

void loop() {
  pixy.line.getAllFeatures();
  uint8_t nb_vect  = pixy.line.numVectors;
  uint8_t bestLine = 0;
  uint8_t bestY    = max(pixy.line.vectors[0].m_y0, pixy.line.vectors[0].m_y1);
  for (uint8_t i = 1; i < pixy.line.numVectors; i++) {
    uint8_t maxY = max(pixy.line.vectors[i].m_y0, pixy.line.vectors[i].m_y1);
    int8_t dx    = pixy.line.vectors[i].m_x0 - pixy.line.vectors[i].m_x1;
    int8_t dy    = pixy.line.vectors[i].m_y0 - pixy.line.vectors[i].m_y1;
    uint16_t squared_dist = dx * dx + dy * dy;
    if (maxY > bestY && squared_dist > 100) {
      double angle = angle_servo(
          pixy.line.vectors[bestLine].m_x0, pixy.line.vectors[bestLine].m_y0,
          pixy.line.vectors[bestLine].m_x1, pixy.line.vectors[bestLine].m_y1);
      if (abs(angle - lastAngle) < 30) {
        bestY    = maxY;
        bestLine = i;
      }
    }
  }

  double angle = angle_servo(
      pixy.line.vectors[bestLine].m_x0, pixy.line.vectors[bestLine].m_y0,
      pixy.line.vectors[bestLine].m_x1, pixy.line.vectors[bestLine].m_y1);
  lastAngle = angle;
  servo_mot.write(angle);
}
// degrees
double angle_servo(int x0, int y0, int x1, int y1) {
  return atan2(y1 - y0, x1 - x0) * 180 / PI;
}
