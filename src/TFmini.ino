#include <Arduino.h>
#include "TeensyThreads.h"

// Using supplied cable:
// - Black = GND (connected to GND)
// - Red = 5V (4.5 - 6.0V) (connected to Vin on Teensy 3.5, or 5V on Arduino)
// - White = TFmini RX (aka. connect to microcontroller TX, pin1 on Teensy 3.5)
// - Green = TFmini TX (aka. connect to microcontroller RX, pin0 on Teensy 3.5)

// NOTE: for this sketch you need a microcontroller with additional serial ports beyond the one connected to the USB cable
// This includes Arduino MEGA (use Serial1), Teensy (3.x) (use one of the available HW Serial connections)


volatile int liDARval = 0;

void readLiDAR(){
  // Data Format for Benewake TFmini
  // ===============================
  // 9 bytes total per message:
  // 1) 0x59
  // 2) 0x59
  // 3) Dist_L (low 8bit)
  // 4) Dist_H (high 8bit)
  // 5) Strength_L (low 8bit)
  // 6) Strength_H (high 8bit)
  // 7) Reserved bytes
  // 8) Original signal quality degree
  // 9) Checksum parity bit (low 8bit), Checksum = Byte1 + Byte2 +...+Byte8. This is only a low 8bit though

  while(1){ // Keep going for ever
    while(Serial1.available()>=9) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
    {
      if((0x59 == Serial1.read()) && (0x59 == Serial1.read())) // byte 1 and byte 2
      {
        unsigned int t1 = Serial1.read(); // byte 3 = Dist_L
        unsigned int t2 = Serial1.read(); // byte 4 = Dist_H
        
        t2 <<= 8;
        t2 += t1;
        liDARval = t2;
        
        t1 = Serial1.read(); // byte 5 = Strength_L
        t2 = Serial1.read(); // byte 6 = Strength_H

        t2 <<= 8;
        t2 += t1;
        for(int i=0; i<3; i++)Serial1.read(); // byte 7, 8, 9 are ignored
      }
    }
  }
}

void setup()
{
    Serial1.begin(115200); // HW Serial for TFmini
    Serial.begin(115200); // Serial output through USB to computer

    delay (100); // Give a little time for things to start

    // Set to Standard Output mode
    Serial1.write(0x42);
    Serial1.write(0x57);
    Serial1.write(0x02);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x01);
    Serial1.write(0x06);

    // Setup thread for reading serial input from TFmini
    threads.addThread(readLiDAR);
}


void loop()
{
  delay(10);  // Don't want to read too often as TFmini samples at 100Hz
  Serial.println(liDARval);
}
