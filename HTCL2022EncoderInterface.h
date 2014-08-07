/* Copyright (C) 2014 Michael Yip, RobotInventorBlog.com

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

The Robot Inventor's Blog
Web      :  http://myrobotblog.com
e-mail   :  support@myrobotblog.com
 */
#ifndef HTCL2022EncoderInterface_H
#define HTCL2022EncoderInterface_H
#include <arduino.h>
#include <cmath>
#include <stdint.h>
struct Encoder {
  int bit_pins[8];
  int sb_pins[2];
  int reset_pin;
  int oe_pin;
  int rotations;
  long current_count;

  Encoder(int* latch8bitPins, int sel1Pin, int sel2Pin, int oePin, int resetPin) {
    //12 pins total latch8bitPins is first 8 pins for d0-d7, then sel1 sel2, OE, then reset_pin
    //note: set sel2Pin = -1 if you only want 0-65,535
    
    for (int i = 0; i < 8; i++) {
      bit_pins[i] = latch8bitPins[i];
      pinMode(latch8bitPins[i], INPUT);
    }

    sb_pins[0] = sel1Pin;
    pinMode(sb_pins[0], OUTPUT);
    if(sel2Pin < 0){
      sb_pins[1] = -1;
    }
    else{
      sb_pins[1] = sel2Pin;
      pinMode(sb_pins[1], OUTPUT);
    }
    oe_pin = oePin;
    pinMode(reset_pin, OUTPUT);
    pinMode(oe_pin, OUTPUT);
    digitalWrite(oe_pin, HIGH); //free bus
    reset_pin = resetPin;
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, LOW);
    rotations = 0;
    current_count = 0;
    delay(1);
    digitalWrite(reset_pin, HIGH);
  }

  void resetCounter() {
    digitalWrite(reset_pin, LOW);
    delay(10);
    digitalWrite(reset_pin, HIGH);
    delay(10);
  }

  long read() {
    //for some reason the encoder only outputs from 2^18 and cycles around to -2^18
    uint32_t count = 0;

    //enable bus
    digitalWrite(oe_pin, LOW);
    //read bits
    int selections_start = 0;
    if(sb_pins[1] == -1) //don't care about most significant 16bits
         selections_start = 2;
         
    for (int selections = selections_start; selections < 4; selections++) { //only really need the lowest two byte -- so can tie sb_pins[1] to low.
      if (selections == 0) { //MSB
        digitalWrite(sb_pins[0], LOW);
        digitalWrite(sb_pins[1], HIGH);
      }
      else if (selections == 1) {
        digitalWrite(sb_pins[0], HIGH);
        digitalWrite(sb_pins[1], HIGH);
      }
      else if (selections == 2) {
        digitalWrite(sb_pins[0], LOW);
        digitalWrite(sb_pins[1], LOW);
      }
      else if (selections == 3) { //LSB
        digitalWrite(sb_pins[0], HIGH);
        digitalWrite(sb_pins[1], LOW);
      }


      for (int i = 7; i >= 0; i--) {
        count = count << 1;
        if (digitalRead(bit_pins[i]) == LOW) {
          count = count ^ 0x1;
        }
      }
    }

    //release bus
    digitalWrite(oe_pin, HIGH);

    //check overflow
    if(count > 32767)
      count = count - 65535;
      
    current_count = count;
    return current_count;
  }
};

#endif

