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
double angled(double x0, double y0, double x1, double y1) {
  return angle_servo(x0, y0, x1, y1);
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
#define MAX_DEVIATION 90              // Degrees
#define Y_LIMIT_VIRAGE 25             // pixels
#define DEVIATION_PRE_VIRAGE 60       // degrees

#define ETAT_LIGNE_DROITE 0
#define ETAT_VIRAGE 1
#define ETAT_PRE_VIRAGE 2


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
      return a;
    }

    else if (r >= 1) {
      int b = 170 - 80 * (1 - exp(-r + 1));
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
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double angle =
        angled(pixy.line.vectors[i].m_x0, SCALEY(pixy.line.vectors[i].m_y0),
              pixy.line.vectors[i].m_x1, SCALEY(pixy.line.vectors[i].m_y1));
    double normeVecteur =
        norme(pixy.line.vectors[i].m_x0, SCALEY(pixy.line.vectors[i].m_y0),
              pixy.line.vectors[i].m_x1, SCALEY(pixy.line.vectors[i].m_y1));
    if (angle < 90) {
      normeGauche += normeVecteur;
    } else {
      normeDroite += normeVecteur;
    }
  }
  Serial.print("Norme Gauche:");
  Serial.println(normeGauche);
  Serial.print("Norme Droite:");
  Serial.println(normeDroite);
  double ratio = normeDroite / (normeGauche + normeDroite);
  Serial.print("Ratio:");
  Serial.println(ratio);
  return ratio * 140 + 20;
}

int indiceVecteurPlusGrandeNorme(){
  int index = 0;
  double meilleurNorme = norme(pixy.line.vectors[0].m_x0, SCALEY(pixy.line.vectors[0].m_y0),
              pixy.line.vectors[0].m_x1, SCALEY(pixy.line.vectors[0].m_y1));
   for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double normeVecteur =
        norme(pixy.line.vectors[i].m_x0, SCALEY(pixy.line.vectors[i].m_y0),
              pixy.line.vectors[i].m_x1, SCALEY(pixy.line.vectors[i].m_y1));
    if (normeVecteur > meilleurNorme) {
      meilleurNorme = normeVecteur;
      index = i;
    } 
  }
  return index;
}


double moyenneAngles(){
    double sumAngles = 0.;
    for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
      sumAngles +=
          angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                      pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);
    }
  
    sumAngles /= pixy.line.numVectors;
    return sumAngles;
}

double deviationAngulairePlusImportante(){
  double meilleureDeviation = 0.;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double angle =
          angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                      pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);
    if(abs(angle - 90) > meilleureDeviation){
      meilleureDeviation = abs(angle - 90);
    }
  }
  return meilleureDeviation;
}

double angleDeDeviationPlusFaible(){
  double meilleureDeviation = 90.;
  double angleM = 0.;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double angle =
          angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                      pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);
    if(abs(angle - 90) < meilleureDeviation){
      meilleureDeviation = abs(angle - 90);
      angleM = angle;
    }
  }
  return angleM;
}

int getEtat(){
  int i = indiceVecteurPlusGrandeNorme();
  int angle = angle_servo(pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
                    pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1) ;
  int min_y = min(pixy.line.vectors[i].m_y0, pixy.line.vectors[i].m_y1);
  if(abs(angle - 90) > 75 && min_y > Y_LIMIT_VIRAGE){
    return ETAT_VIRAGE;
  }
  if(deviationAngulairePlusImportante() > DEVIATION_PRE_VIRAGE){
    if(pixy.line.numVectors == 1)
      return ETAT_VIRAGE;
    else
      return ETAT_PRE_VIRAGE;
  }
  return ETAT_LIGNE_DROITE;
}

double loopPreVirage(){
  int max_y = 0;
  int index = 0;
  bool zero = true;
  for(int i = 0; i < pixy.line.numVectors; i++){
    if(pixy.line.vectors[i].m_y0 > max_y){
      max_y = pixy.line.vectors[i].m_y0;
      index = i;
      zero = true;
    }
    if(pixy.line.vectors[i].m_y1 > max_y){
      max_y = pixy.line.vectors[i].m_y1;
      index = i;
      zero = false;
    }
  }
  int x = zero ? pixy.line.vectors[index].m_x0 : pixy.line.vectors[index].m_x1;
  double dx = (x - 39) / 39.0;
  return 90  + 60 * dx;
  //return 90 - (90. - angleDeDeviationPlusFaible());
}

double loopVirages() {
  double sumAngles = moyenneAngles();
  Serial.print("Moyenne:");
  Serial.println(sumAngles);
  return sumAngles;
}

void loop() {
  pixy.line.getAllFeatures();
  double ordre = 0.;
  switch(getEtat()){
    case ETAT_VIRAGE:
     Serial.println("Virages:");
     ordre = loopVirages();
     break;
     case ETAT_PRE_VIRAGE:
     Serial.println("Pre-Virage:");
     ordre = loopPreVirage();
     break;
     default:
     Serial.println("Ligne Droite:");
     ordre = loopLigneDroite();
     break;
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
