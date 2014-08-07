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
 
#include "HTCL2022EncoderInterface.h"
#include "CommunicationUtilsMod.h"

Encoder *enc1, *enc2, *enc3;

void setup() {
  const int resetPin = 35;
  int enc8BitPins[] = {53, 51, 49, 47, 45, 43, 41, 39};
  const int byteSelectPin = 37;
  enc1 = new Encoder(enc8BitPins,byteSelectPin,-1,31,resetPin);
  enc2 = new Encoder(enc8BitPins,byteSelectPin,-1,33,resetPin);
  enc3 = new Encoder(enc8BitPins,byteSelectPin,-1,29,resetPin);

  //Start Serial
  Serial.begin(115200);
}



int iter = 0;
float thetas[3];

void loop() {
    const float cableRadius = 0.40;//mm
    const float capstanRadius = 3.175;//mm
    const float wheelRadius[] = {65, 29, 29};//mm
    const float numCPR = 1440.0; //360*4 for quadrature
   
    const float encToAngle0 = 2.*PI/numCPR*(cableRadius+capstanRadius)/(cableRadius+wheelRadius[0]);
    const float encToAngle1 = 2.*PI/numCPR*(cableRadius+capstanRadius)/(cableRadius+wheelRadius[1]);
    const float encToAngle2 = 2.*PI/numCPR*(cableRadius+capstanRadius)/(cableRadius+wheelRadius[2]);

    //read angles
    const float restingTheta[] = {0.0, 0./180.*PI, 120./180.*PI}; 
    thetas[0] = encToAngle0*((float) enc1->read())+restingTheta[0];
    thetas[1] = encToAngle1*((float) enc2->read())+restingTheta[1];
    thetas[2] = encToAngle2*((float) enc3->read())+restingTheta[2];

    //read angles
//    const int usign_offset = 10000; //keeps the thetas positive, so that serial communication can be done on unsigned, lower-than-4 byte ints.
//    const float restingTheta[] = {0.0/encToAngle0, 0./180.*PI/encToAngle1, 120./180.*PI/encToAngle2}; 
//    thetas[0] = enc1->read()+ ((int) restingTheta[0]) + usign_offset;
//    thetas[1] = enc2->read()+ ((int) restingTheta[1]) + usign_offset;
//    thetas[2] = enc3->read()+ ((int) restingTheta[2]) + usign_offset;


   //write position to PC (only after it has recieved a reply from PC)
   while(Serial.available() >= 3){  //3 bytes, each byte represents a force level 0-255, where x,y,z, is byte 1,2,3
     int fx = Serial.read();
     int fy = Serial.read();
     int fz = Serial.read();
   
      float output[7];
      output[0] = thetas[0];
      output[1] = thetas[1];
      output[2] = thetas[2];
      output[3] = fx;
      output[4] = fy;
      output[5] = fz;
      output[6] = 0;
      //printNCharIntArr(output, 3, 4); //print output as 4-byte integers
      serialPrintFloatArr(output,7);
      Serial.print("$"); //endSignal
    }
    delay(30);
    iter++;
}

