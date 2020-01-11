#include <Servo.h>

Servo esc;
char data;

/**
 * Initialisation routine
 */
void setup() {
    Serial.begin(9600);

    esc.attach(6); // 1000 and 2000 are very important ! Values can be different with other ESCs.


    displayInstructions();
}

/**
 * Main function
 */
void loop() {
  
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending 0 throttle");
                      esc.write(0);

            break;

            // 1
            case 49 : Serial.println("Sending 180 throttle");
                      esc.write(180);

            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
    }
}

/**
 * Test function sending angle to the ESCs from 0 to 180 degrees
 */
void test()
{ /*
  esc.write(0);
  delay(200);
  if (Serial.available() > 0)
  {
    int vitesse = Serial.parseInt();
    esc.write(vitesse);
  }
  
  */
  /*  for (int i=0; i<=180; i++) {
        Serial.print("Speed = ");
        Serial.println(i);

        esc.write(i);


        delay(200);
    }

    Serial.println("STOP");*/
        delay(200);
    esc.write(0);
    delay(200);
    esc.write(103);
    delay(3000);
  
    esc.write(0);
    delay(42000);
  
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Sends 0 throttle");
    Serial.println("\t1 : Sends 180 throttle");
    Serial.println("\t2 : Runs test function\n");
}
