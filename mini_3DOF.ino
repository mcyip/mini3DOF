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

//For Sparkfun FreeSixDOF IMU chip
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>
FreeSixIMU my3IMU = FreeSixIMU();

#include "CommunicationUtilsMod.h"
#include "HTCL2022EncoderInterface.h"


Encoder *enc1, *enc2, *enc3;

void setup() {
  //MEGA
  int enc8BitPins[] = {53, 51, 49, 47, 45, 43, 41, 39}; //Arduino MEGA
  const int byteSelectPin = 37;
  const int resetPin = 35;
  int encoderOrderedSelectPins[] = {31, 33, 29};

  //UNO
//  int enc8BitPins[] = {3, 4, 5, 6, 7, 8, 9, 10}; //Arduino UNO
//  const int byteSelectPin = 11;
//  const int resetPin = 12;
//  int encoderOrderedSelectPins [] = {2, 13, A0};

  enc1 = new Encoder(enc8BitPins, byteSelectPin, -1, encoderOrderedSelectPins[0], resetPin);
  enc2 = new Encoder(enc8BitPins, byteSelectPin, -1, encoderOrderedSelectPins[1], resetPin);
  enc3 = new Encoder(enc8BitPins, byteSelectPin, -1, encoderOrderedSelectPins[2], resetPin);

  //Start Serial
  Serial.begin(115200);

  //Start IMU code
  Wire.begin();
  TWBR = 12;  // 400 kHz (maximum)
  delay(5);
  my3IMU.init(true);
  delay(5);
}

const float cableRadius = 0.40;//mm
const float capstanRadius = 3.175;//mm
const float wheelRadius[] = {65, 29, 29};//mm
const float numCPR = 1440.0; //360*4 for quadrature

const float encToAngle0 = 2.*PI / numCPR * (cableRadius + capstanRadius) / (cableRadius + wheelRadius[0]);
const float encToAngle1 = 2.*PI / numCPR * (cableRadius + capstanRadius) / (cableRadius + wheelRadius[1]);
const float encToAngle2 = 2.*PI / numCPR * (cableRadius + capstanRadius) / (cableRadius + wheelRadius[2]);
const float restingTheta[] = {0. / 180.*PI, -45. / 180.*PI, 120. / 180.*PI};
const int restingThetaInt[] = {((int)(restingTheta[0] / encToAngle0)), ((int)(restingTheta[1] / encToAngle1)), ((int)(restingTheta[2] / encToAngle2))};

int iter = 0;
int thetas[3] = {0,0,0};
float quaternion[] = {0, 0, 0, 0}; //hold q values
unsigned long time = 0;
unsigned long prev_time = 0;
void loop() {
  time = millis();
  
  //read angles
  const int usign_offset = 10000; //keeps the thetas positive, so that serial communication can be done on unsigned, lower-than-4 byte ints.
  thetas[0] = enc1->read() + restingThetaInt[0] + usign_offset;
  thetas[1] = enc2->read() + restingThetaInt[1] + usign_offset;
  thetas[2] = enc3->read() + restingThetaInt[2] + usign_offset;

  //get quaternion from IMU
  if((time-prev_time) > 20){ //try to make this lower...
    my3IMU.getQ(quaternion); //turning this on kills the real-time code...
    prev_time = time;
  }
//Serial.print(quaternion[0]);
//Serial.print(",");
//Serial.print(quaternion[1]);
//Serial.print(",");
//Serial.print(quaternion[2]);
//Serial.print(",");
//Serial.println(quaternion[3]);

  //write position to PC (only after it has recieved a reply from PC)
 while (Serial.available() >= 3) { //3 bytes, each byte represents a force level 0-255, where x,y,z, is byte 1,2,3
      int fx = Serial.read();
      int fy = Serial.read();
      int fz = Serial.read();

    printIntArrAsHex(thetas, 3); //print thetas as a shortened hex list
    serialPrintFloatArr(quaternion, 4); //print quaternion as four byte hexes.
    Serial.print("$"); //endSignal
    
  }
iter++;
}


