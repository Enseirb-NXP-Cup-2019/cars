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

  //Variables globales
  int FREQUENCY = 100; //rafraichissement de la loop en ms
  
  //Coordonnées des vecteurs
  float result = 90;
  float x0;
  float x1;
  float y0;
  float y1;
  float x2;
  float x3;
  float y2;
  float y3;
  //int8_t i;
  //char buf[128];
  pixy.line.getAllFeatures();
  
  int nb_vect=min(pixy.line.numVectors,2);
  Serial.println(nb_vect);
  
  switch (nb_vect){
    case 1: //si un seul vecteur est détecté
    x0=(float)pixy.line.vectors[0].m_x0;
    x1=(float)pixy.line.vectors[0].m_x1;
    y0=(float)pixy.line.vectors[0].m_y0;
    y1=(float)pixy.line.vectors[0].m_y1;

    result = angle_dir(x0,x1,y0,y1);
  
    case 2://si deux vecteurs sont détectés
    x0=(float)pixy.line.vectors[0].m_x0;
    x1=(float)pixy.line.vectors[0].m_x1;
    y0=(float)pixy.line.vectors[0].m_y0;
    y1=(float)pixy.line.vectors[0].m_y1;
    x2=(float)pixy.line.vectors[1].m_x0;
    x3=(float)pixy.line.vectors[1].m_x1;
    y2=(float)pixy.line.vectors[1].m_y0;
    y3=(float)pixy.line.vectors[1].m_y1;
  
    float ag1 = angle_dir(x0,x1,y0,y1);
    float ag2 = angle_dir(x2,x3,y2,y3);
    float ratio = center_distance_ratio(x0,y0,x1,y1,x2,y2,x3,y3);
    result = angle_pondere(avg(ag1,ag2),ratio);
    
  }
  
  Serial.println(result);
  servo_mot.write(result);
  delay(FREQUENCY);

}






float coeff_dir(float x0, float x1, float y0, float y1){
  return (y1-y0)/(x1-x0);
}


int angle_dir(float x0, float x1, float y0, float y1){
  if (x1-x0==0) {
    return 90;
  }
  else {
    float r = coeff_dir(x0,x1,y0,y1);
    
    if (r>=1){
      int a= 150-60*(1-exp(1-r));
      //Serial.println(a); 
      return a;
    }
    
    else if (r<=-1){
      int b=30+60*(1-exp(r+1));
      //Serial.println(b);
      return b;
    }

    else if (0<r) {
      return 150;
    }

    else {
      return 30;
    }
  }
  
}

float center_distance_ratio(float x0, float x1, float y0, float y1, float x2, float x3, float y2, float y3){
  //return 0 si la voiture est centrée
  //1 si la voiture est à droite de decalage_max pixels ou plus du centre de la piste
  //-1 si la voiture est à gauche de decalage_max pixels ou plus du centre de la piste
 
  int decalage_max = 10;
  float r1 = coeff_dir(x0,x1,y0,y1);
  float r2 = coeff_dir(x2,x3,y2,y3);
  float xc1 = (17-y0)/r1 + x0;//17 est une valeur de y arbitraire, telle que si on trace une ligne horizontale d'eq y=17, elle devrait couper les deux vecteurs détectés par la pixy
  float xc2 = (17-y2)/r2 + x2;//elle peut donc être modifiée si besoin
  return (((xc1+xc2)/2 - 39)/decalage_max); //39 = 78/2, avec 78 la longueur du champ de vision en pixels
}

int angle_pondere(int ag_dir ,float ratio){

  int ret;//l'angle calculé à partir de la distance de la voiture au centre de la piste
  
  if (ratio>=1){
    ret=30;
  }
  else if (ratio>=0){
    ret=30+60*(1-exp(-100+ratio));
  }
  else if (ratio<=-1){
    ret=150;
  }
  else if (ratio<=0){
    ret=150-60*(1-exp(-(100+ratio)));//Cette fonction retourne une moyenne pondéré entre les instructions de direction proposées par deux algos:
    //la première instruction (une valeur entre 0 et 180 donc) est dans ag_dir et dépend du coefficient directeur des lignes qu'il voit
    //la deuxième valeur entre 0 et 180 est dans ret et est calculée à partir de la distance de la voiture au centre de la piste
    //on pourrait donc modifier cette fonction en lui disant de retourner simplement ret par exemple, pour se baser uniquement sur la distance au centre de la piste
  
  }
  
  return (1-ratio)*ag_dir + ratio*ret;
}

int avg(int a, int b){
  return (a+b)/2;
}
