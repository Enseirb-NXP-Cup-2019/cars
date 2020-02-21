#include <Pixy2.h>
#include <Servo.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// MATH UTILS ------------------------------------------------------------------
#define PI 3.14159265

uint8_t max(uint8_t a, uint8_t b) { return a > b ? a : b; }
uint8_t min(uint8_t a, uint8_t b) { return a < b ? a : b; }

double average(double* array, uint8_t size) {
  double sum = 0;
  for (size_t i = 0; i < size; i++) {
    sum += array[i];
  }
  return sum / size;
}
double slope(double x0, double x1, double y0, double y1) {
  return (y1 - y0) / (x1 - x0);
}
double norm(double x0, double y0, double x1, double y1) {
  double dx = (x1 - x0);
  double dy = (y1 - y0);
  return sqrt(dx * dx + dy * dy);
}
double to_range(double x, double mini, double maxi){
  return min(maxi, max(x, mini));
}
// HARDWARE GLOBALS ------------------------------------------------------------
static Pixy2 pixy;
static Servo esc;
static Servo servo_motor;
// FILE GLOBALS ----------------------------------------------------------------
#define NB_FRAMES 16             // Frames considering 30fps
#define TURN_Y_LIMIT 20          // pixels
#define TURN_SHIFT 85.0          // degrees
#define PRE_TURN_SHIFT 60.0      // degrees
#define DELAY_BETWEEN_LOOPS 30  // milliseconds
#define STRAIGHT_LINE_ANGLE 90.0 // degrees
#define MAX_ROTATION_ORDER 180.0 // degrees
#define MIN_ROTATION_ORDER 0.0 //degrees
#define TURN_SPEED 1610
#define PRE_TURN_SPEED 1612
#define STRAIGHT_LINE_SPEED 1612
// STATES ------------------------------
#define STATE_STRAIGHT_LINE 0
#define STATE_TURN 1
#define STATE_PRE_TURN 2
// LOGIC GLOBALS ---------------------------------------------------------------
static double last_orders[NB_FRAMES];
static uint8_t index_last_order = 0;
int zebra_cross = 0;
// Scale the Y coordinates to have a 1:1 ratio with the X coordinates
#define SCALE_Y(y) (y * 78.0 / 51.0)
// SETUP -----------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  servo_motor.attach(10, 1000, 2000); // pin 10 -> servomotor
  esc.attach(6);                      // pin 6 -> ESC
  Serial.begin(115200);
  Serial.print("Starting...\n");
  // we need to initialize the pixy object
  pixy.init();
  // Change to line tracking program
  pixy.changeProg("line");

  calibrage();
  // Fill last_orders with STRAIGHT_LINE_ANGLE
  for (int i = 0; i < NB_FRAMES; i++) {
    last_orders[i] = STRAIGHT_LINE_ANGLE;
  }
}


//Calibrage --------------------------------------------------------------------

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

// Vectors Utils ---------------------------------------------------------------
/**
  Returns the angular shift from STRAIGHT_LINE_ANGLE as a positive double.
**/
double angle_shift(double angle) { return abs(STRAIGHT_LINE_ANGLE - angle); }
/**
  Returns the index of the vector with the biggest norm.
**/
uint8_t index_longest_vector() {
  uint8_t index = 0;
  double best_length =
      norm(pixy.line.vectors[0].m_x0, SCALE_Y(pixy.line.vectors[0].m_y0),
           pixy.line.vectors[0].m_x1, SCALE_Y(pixy.line.vectors[0].m_y1));
  for (uint8_t i = 1; i < pixy.line.numVectors; i++) {
    double length =
        norm(pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
             pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
    if (length > best_length) {
      best_length = length;
      index       = i;
    }
  }
  return index;
}
/**
  Returns the angle of the passed vector, the angle is anti-clockwise from the
right.
**/
double vector_angle(double x0, double y0, double x1, double y1) {
  if (abs(x1 - x0) <= 0.001) {
    return 90.0;
  } else {
    double r = slope(x0, x1, y0, y1);
    if (r <= -1.0) {
      return 10 + 80.0 * (1 - exp(1 + r));
    } else if (r >= 1.0) {
      return 170 - 80.0 * (1 - exp(-r + 1));
    } else if (r > 0.0) {
      return 170.0;
    } else {
      return 10.0;
    }
  }
}
double average_vector_angles() {
  double sum_angles = 0.;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    sum_angles += vector_angle(
        pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
        pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
  }

  sum_angles /= pixy.line.numVectors;
  return sum_angles;
}
double biggest_angular_shift() {
  double best_shift = 0.;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double angle = vector_angle(
        pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
        pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
    if (angle_shift(angle) > best_shift) {
      best_shift = angle_shift(angle);
    }
  }
  return best_shift;
}
/**
  Returns the coordinates of the point with the highest Y among all points.
**/
void highest_point(int* x, int* y) {
  int max_y     = 0;
  uint8_t index = 0;
  bool zero     = true;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    if (pixy.line.vectors[i].m_y0 > max_y) {
      max_y = pixy.line.vectors[i].m_y0;
      index = i;
      zero  = true;
    }
    if (pixy.line.vectors[i].m_y1 > max_y) {
      max_y = pixy.line.vectors[i].m_y1;
      index = i;
      zero  = false;
    }
  }
  *y = max_y;
  *x = zero ? pixy.line.vectors[index].m_x0 : pixy.line.vectors[index].m_x1;
}

// STATE -----------------------------------------------------------------------
int get_state() {
  uint8_t i    = index_longest_vector();
  double angle = vector_angle(
      pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
      pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
  int min_y = min(pixy.line.vectors[i].m_y0, pixy.line.vectors[i].m_y1);
  Serial.print("Min y:");
  Serial.println(min_y);
  // STATE_TURN if the shift is big enough and the min y is bigger than the Y
  // limit
  if (angle_shift(angle) > TURN_SHIFT && min_y > TURN_Y_LIMIT) {
    return STATE_TURN;
  }
  // If the biggest angular shift is big enough
  if (biggest_angular_shift() > PRE_TURN_SHIFT) {
    // If we only have 1 vector
    if (pixy.line.numVectors == 1)
      return STATE_TURN;
    else
      return STATE_PRE_TURN;
  }
  // In any other case we are on a straight line
  return STATE_STRAIGHT_LINE;
}

// ORDERS ----------------------------------------------------------------------
double order_straight_line() {
  double left_norm  = 0.;
  double right_norm = 0.;
  for (uint8_t i = 0; i < pixy.line.numVectors; i++) {
    double angle = vector_angle(
        pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
        pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
    double length =
        norm(pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0), 
             pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
    if (angle < 90.0) {
      left_norm += length;
    } else {
      right_norm += length;
    }
  }
  Serial.print("Norme Gauche:");
  Serial.println(left_norm);
  Serial.print("Norme Droite:");
  Serial.println(right_norm);
  double ratio = right_norm / (left_norm + right_norm);
  Serial.print("Ratio:");
  Serial.println(ratio);
  if(zebra_cross == 0){
    esc.write(STRAIGHT_LINE_SPEED);
  }
  return ratio * 135 + 20;
}
double order_turn() { 
  if(zebra_cross == 0){
    esc.write(TURN_SPEED);
    }
  return 0.7 * (average_vector_angles() - 90) + 90; 
}
double order_pre_turn() {
  int x, y;
  highest_point(&x, &y);
  double dx = (x - 39) / 39.0;
  if(zebra_cross == 0){
    esc.write(PRE_TURN_SPEED);
  }
  return STRAIGHT_LINE_ANGLE + 40 * dx;
}


//DETECTING LINE
int depart_line(){
 double vectors_norm[2];
 int iterator = 0;
 int numVectors = pixy.line.numVectors;
 //Serial.println(index_max);
  //Serial.println(index_min);
  for (uint8_t i = 0; i < numVectors; i++) {
      if(abs(SCALE_Y(pixy.line.vectors[i].m_y0) - SCALE_Y(pixy.line.vectors[i].m_y1)) < 0.1){       
        vectors_norm[iterator] = norm(pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
             pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
      
        iterator++;
      }
  }
  if (abs(vectors_norm[0] - vectors_norm[1]) <= 0.1){
        return 1;
  }
  return 0;
}


//GET INDEX OF EXTREM VECTORS STORE IN INDEX_MIN AND INDEX_MAX
void extrem_vectors(uint8_t *index_min, uint8_t *index_max){
  *index_min = 0;
  *index_max = 0;
  int min_x = min(pixy.line.vectors[0].m_x0,pixy.line.vectors[0].m_x1);
  int max_x = max(pixy.line.vectors[0].m_x0,pixy.line.vectors[0].m_x1);
  double test_value;
  for(uint8_t i = 1; i < pixy.line.numVectors; i++){
   // Serial.println(i);
    test_value = min(pixy.line.vectors[i].m_x0,pixy.line.vectors[i].m_x1);
    
    if( test_value < min_x ){
      min_x = test_value;
      *index_min = i;
    }
    else{
      test_value = max(pixy.line.vectors[i].m_x0,pixy.line.vectors[i].m_x1);
      if( test_value > max_x){
        max_x = test_value;
        *index_max = i;
      }
      
    }
  }
}


//DETECT IF THERE IS A ZEBRA CROSS WITH N BANDS
bool detecting_zebra_cross(uint8_t n) { 
  int numVectors = pixy.line.numVectors;
  //Serial.println(numVectors);
  double vectors_angles[n];
  double vectors_norm[n];
  uint8_t iterator = 0;
  uint8_t index_max, index_min; 
  extrem_vectors(&index_min, &index_max);
 //Serial.println(index_max);
  //Serial.println(index_min);
  for (uint8_t i = 0; i < numVectors; i++) {
    if( i != index_max && i != index_min){
    vectors_angles[iterator] = vector_angle(
        pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
        pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
    vectors_norm[iterator] =
        norm(pixy.line.vectors[i].m_x0, SCALE_Y(pixy.line.vectors[i].m_y0),
             pixy.line.vectors[i].m_x1, SCALE_Y(pixy.line.vectors[i].m_y1));
    iterator++;
    }
  }
  //Serial.println(iterator);
  if(iterator != n || !vectors_norm[0])
    return false;
    
  for (uint8_t i = 1; i < n; i++) {
    //Serial.println(i);
    //Serial.println(vectors_norm[i]);
    double diff_norm = abs(vectors_norm[0] - vectors_norm[i]) / vectors_norm[0];
     
    double diff_angle = abs(vectors_angles[0] - vectors_angles[i]);
   Serial.println(diff_angle);
    if(diff_norm > 0.2)// || diff_angle > 20) //0.2 and 15 are chosen values, have to be tested
      return false;
  }
  return true;
}

//SET ZEBRA CROSS AREA
void set_zebra_cross_area(){
  // detection zebra-cross
  if (pixy.line.numVectors >= 5 && detecting_zebra_cross(4)) // verify that beginning of zebra cross is detected with 3
    zebra_cross = 1;
  if (pixy.line.numVectors >= 6 && detecting_zebra_cross(3)) 
    zebra_cross = 0;  // zebra cross area end is detected by the camera but the car is still in the area
}

void zebra_order(){
  set_zebra_cross_area();
  if(zebra_cross == 1)
  //Serial.print("SLOW");
   esc.write(1608);
  if(zebra_cross == 0){
    //Serial.print("SPEED");
    esc.write(1612);
  }
}

// LOOP ------------------------------------------------------------------------
void loop() {
  pixy.line.getAllFeatures();
  double order = 0.;
  zebra_order();
  switch (get_state()) {
  case STATE_TURN:
    Serial.println("[TURN]");
    order = order_turn();
    break;
  case STATE_PRE_TURN:
    Serial.println("[PRE-TURN]");
    order = order_pre_turn();
    break;
  default:
    Serial.println("[STRAIGHT-LINE]");
    order = order_straight_line();
    break;
  }
  Serial.print("-> Order:");
  Serial.println(order);
  //---------------------
  // après cette ligne on actualise le tableau des NB_FRAMES frames
  // dernières instructions, et on lance l'instruction

  last_orders[index_last_order] = order;
  index_last_order              = (index_last_order + 1) % NB_FRAMES;
  double effective_order        = average(last_orders, NB_FRAMES);
  servo_motor.write((int)to_range(effective_order, MIN_ROTATION_ORDER, MAX_ROTATION_ORDER));
  Serial.print("-> Effective Order:");
  Serial.println(effective_order);
  delay(DELAY_BETWEEN_LOOPS);
}
