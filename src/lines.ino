uint8_t min(uint8_t a, uint8_t b) { return a < b ? a : b; }

// LOGIC GLOBALS ---------------------------------------------------------------
int zebra_cross = 0;

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
void zebra_cross_area(){
  // detection zebra-cross
  if (pixy.line.numVectors >= 5 && detecting_zebra_cross(4)) // verify that beginning of zebra cross is detected with 3
    zebra_cross = 1;
  if (pixy.line.numVectors >= 6 && detecting_zebra_cross(3)) 
    zebra_cross = 0;  // zebra cross area end is detected by the camera but the car is still in the area
}
/*
double speed_order(){
  if(zebra_cross == 2)
    return initial_speed/2;
  if(zebra_cross == 4){
    zebra_cross = 0;
    return initial_speed;
  } 
  return 0;
}
  */
