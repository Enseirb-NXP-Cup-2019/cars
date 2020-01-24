#include <Servo.h>

Servo moteur;
char data;
/*TUTO ---------------------------------------------
 * 
 * 1) Brancher la batterie à l'esc
 * 2) Rester appuyé sur le bouton de l'esc, il va devenir vert, puis rouge, relâcher quand il est rouge
 * Normalement il s'éteint ensuite, puis clignote une fois en rouge
 * 3) Entrer 1 dans le serial
 * 4) L'esc devrait, au bout de 3-4 secondes, clignoter deux fois en rouge
 * 5) Dès que c'est fait, envoyer 2 dans le serial
 * 6) Si la led devient verte au bout de quelques secondes, c'est calibré
 * -------------------------------------------------
 */
void setup() {
  Serial.begin(9600);
  moteur.attach(10);
}

void loop() {
  if (Serial.available()){
    data = Serial.read();
    switch (data){
      //0
      case 48 : Serial.println("ça fonctionne frérot");
      //1
      case 49 : envoyer_max_av();
      break;
      //2
      case 50 : envoyer_max_ar();
      break;
      //3
      case 51 : test();
    }
  }
}

void envoyer_max_av(){
  moteur.write(2000);
}

void envoyer_max_ar(){
  moteur.write(0);
}

void test(){
  moteur.write(1600);
}
