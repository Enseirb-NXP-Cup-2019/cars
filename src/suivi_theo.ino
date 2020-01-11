#include <Pixy2.h>
#include <Servo.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// UTILS -----------------------------------------------------------------------
#define PI 3.14159265

uint8_t max(uint8_t a, uint8_t b) { return a > b ? a : b; }
double average(double* array, uint8_t size) {
  double sum = 0;
  for (size_t i = 0; i < size; i++) {
    sum += array[i];
  }
  return sum / size;
}
double angle(double x0, double y0, double x1, double y1) {
  if (y0 > y1) {
    return angle(x1, y1, x0, y0);
  }
  return atan2(y1 - y0, x1 - x0);
}
double norme(double x0, double y0, double x1, double y1) {
  double dx = (x1 - x0);
  double dy = (y1 - y0);
  return sqrt(dx * dx + dy * dy);
}
// HARDWARE GLOBALS ------------------------------------------------------------
static Pixy2 pixy;
static Servo esc;
static Servo servo_mot;

// FILE GLOBALS ----------------------------------------------------------------
#define NB_FRAMES 10                  // Frames considering 30fps
#define LONG_MIN_LIGNE_DROITE 40      // Pixels
#define MAX_DECALAGE_CENTRE 8         // ??
#define MAX_DEVIATION_LIGNE_DROITE 40 // Degrees
#define MAX_DEVIATION 90              // Degrees

#define SCALEY(X) (X * 78.0 / 51.0)

static double lastAngles[NB_FRAMES];
static uint8_t lastAngle_index = 0;

void setup() {
  Serial.begin(9600);
  servo_mot.attach(10, 1000, 2000); // pin 10 -> servomotor
  esc.attach(6);                    // pin 6 -> ESC
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // we need to initialize the pixy object
  pixy.init();
  // Change to line tracking program
  pixy.changeProg("line");

  esc.write(0);
  delay(2000);
  esc.write(100);
  for (int i = 0; i < NB_FRAMES; i++) {
    lastAngles[i] = 90.0;
  }
}

double amortissement_reponse(double x) { return x * x / 180.0; }
bool estSurLigneDroite() {
  for (uint8_t i = 0; i < pixy.line.vectors; i++) {
    double angle =
        angle(pixy.line.vectors[i].m_x0, SCALEY(pixy.line.vectors[i].m_y0),
              pixy.line.vectors[i].m_x1, SCALEY(pixy.line.vectors[i].m_y1));
    if (abs(angle - 90) > MAX_DEVIATION_LIGNE_DROITE) {
      return false;
    }
  }
  return true;
}

float coeff_dir(float x0, float x1, float y0, float y1) {
  return (y1 - y0) * 78.0 / 51.0 / (x1 - x0);
}

int angle_servo(int x0, int y0, int x1, int y1) {
  if (x1 - x0 == 0) {
    return 90;
  } else {
    float r = coeff_dir(x0, x1, y0, y1);

    if (r <= -1) {
      int a = 10 + 80 * (1 - exp(1 + r));
      // Serial.println(a);
      return a;
    }

    else if (r >= 1) {
      int b = 170 - 80 * (1 - exp(-r + 1));
      // Serial.println(b);
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

int angle_servo_milieu(float x0, float y0, float x1, float y1, float x2,
                       float y2, float x3, float y3) {
  float av_x       = (x0 + x1 + x2 + x3) / 4;
  float dist_ratio = (av_x - 39) / MAX_DECALAGE_CENTRE;
  return 90 * (1 + dist_ratio);
}

double loopLigneDroite() {
  double normeGauche = 0.;
  double normeDroite = 0.;
  for (uint8_t i = 0; i < pixy.line.vectors; i++) {
    double angle =
        angle(pixy.line.vectors[i].m_x0, SCALEY(pixy.line.vectors[i].m_y0),
              pixy.line.vectors[i].m_x1, SCALEY(pixy.line.vectors[i].m_y1));
    double normeVecteur =
        norme(pixy.line.vectors[i].m_x0, SCALEY(pixy.line.vectors[i].m_y0),
              pixy.line.vectors[i].m_x1, SCALEY(pixy.line.vectors[i].m_y1));
    if (angle > 90) {
      normeGauche += normeVecteur;
    } else {
      normeDroite += normeVecteur;
    }
  }
  Serial.print("Norme Gauche:");
  Serial.print(normeGauche);
  Serial.print("\nNorme Droite:");
  Serial.print(normeDroite);
  double ratio = normeDroite / (normeGauche + normeGauche);
  return ratio * 180;
}

double loopVirages() {
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double angle =
        angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                    pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);
    double diff = abs(90 - angle);
    if (diff <= MAX_DEVIATION) {
      sumAngles += angle;
    }
    Serial.print("Angle vecteur:");
    Serial.print(
        angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                    pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1));
    Serial.print("\n");
  }

  sumAngles /= pixy.line.numVectors;
  Serial.print("Moyenne:");
  Serial.print(sumAngles);
  Serial.print("\n");
}

void loop() {
  pixy.line.getAllFeatures();
  double ordre = 0.;

  // Si on est sur une ligne droite ----------------------------------------
  if (estSurLigneDroite()) {
    Serial.print("Ligne droite:\n");
    ordre = loopLigneDroite();
  }
  // Si on est pas sur une ligne droite ------------------------------------
  else {
    Serial.print("Virage:\n");
    ordre = loopVirages();
  }
  Serial.print("\n\tordre:");
  Serial.print(ordre);
  Serial.print("\n");
  //--------------------- après cette ligne on actualise le tableau des 10
  // dernières instructions, et on lance l'instruction

  lastAngles[lastAngle_index] = ordre;
  lastAngle_index             = (lastAngle_index + 1) % NB_FRAMES;
  servo_mot.write(average(lastAngles, NB_FRAMES));
  Serial.print("\tDirection prise:");
  Serial.print(average(lastAngles, NB_FRAMES));
  Serial.print("\n");
  delay(100);
}
