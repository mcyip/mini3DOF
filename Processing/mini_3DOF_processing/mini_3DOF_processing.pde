import processing.serial.*;

Serial myPort;  // Create object from Serial class

final String serialPort = "COM6";  // change this to your port on linux. Should be something like "/dev/ttyACM0" on Linux

float [] q = new float [4];
int [] encoders = new int [3];
float [] encAngles = new float [3];
float [] px = new float [3];
float [] hq = null;
float [] Euler = new float [3]; // psi, theta, phi

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600;


void setup() 
{
  size(VIEW_SIZE_X, VIEW_SIZE_Y, P3D);
  myPort = new Serial(this, serialPort, 115200);  

  font = createFont("Courier", 32); 


  /*
  float [] axis = new float[3];
   axis[0] = 0.0;
   axis[1] = 0.0;
   axis[2] = 1.0;
   float angle = PI/2.0;
   
   hq = quatAxisAngle(axis, angle);
   
   hq = new float[4];
   hq[0] = 0.0;
   hq[1] = 0.0;
   hq[2] = 0.0;
   hq[3] = 1.0;
   */

  delay(100);
  myPort.clear();
  while (myPort.available () < 3) {
    myPort.write(1); //send initial force x to start the device
  }
  println("Started Device Call.");
}


float decodeFloat(String inString) {
  byte [] inData = new byte[4];

  if (inString.length() == 8) {
    inData[0] = (byte) unhex(inString.substring(0, 2));
    inData[1] = (byte) unhex(inString.substring(2, 4));
    inData[2] = (byte) unhex(inString.substring(4, 6));
    inData[3] = (byte) unhex(inString.substring(6, 8));
  }

  int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  return Float.intBitsToFloat(intbits);
}

//forward kinematics for the haptic device
void fwdKin(float[] thetas, float length_arm1, float length_arm2) {
  float s0, c0, s1, c1, s2, c2;  
  c0 = cos(encAngles[0]);
  c1 = cos(encAngles[1]);
  c2 = cos(encAngles[2]);
  s0 = sin(encAngles[0]);
  s1 = sin(encAngles[1]);
  s2 = sin(encAngles[2]);

  px[0] = c0*(length_arm1*c1+length_arm2*c1*c2-length_arm2*s1*s2);
  px[1] = s0*(length_arm1*c1+length_arm2*c1*c2-length_arm2*s1*s2);
  px[2] = -length_arm1*s1-length_arm2*s1*c2-length_arm2*c1*s2;
}




//converts an n-byte sequence 2*n hex chars into an int
int decodeInt(String inString) {
  return unhex(inString);
}

float dTavg = 0;
int t1 = 0;

int readFromDevice() {
  int  numCharAvailable = myPort.available();
  //while(myPort.available() < 1); //loop until a full measurement set is received
  if (numCharAvailable < 51) { //originally 18 for 4 floats, but why?
    //println("Number of Characters Available: " + numCharAvailable);
  }
  else {
    //Grabs string.
    // returns null if the character looked for was not found and keeps the buffer going. 
    // otherwise, it returns the string truncated to the special character, clears the buffer up to the character.
    String inputString = myPort.readStringUntil('$'); 
    println(inputString);
    if (inputString != null) {
      String [] inputStringArr = split(inputString, ",");

      //check the format of the input string
      if (inputStringArr.length < 7) return 0;

      //check the format of the input string
      if (inputStringArr[0].length() == 4
        && inputStringArr[1].length() == 4
        && inputStringArr[2].length() == 4
        && inputStringArr[3].length() == 8
        && inputStringArr[4].length() == 8
        && inputStringArr[5].length() == 8
        && inputStringArr[6].length() == 8) { // x,y,z,q1,q2,q3,q4\n so we have 7 elements

        //println("Read Properly");
        myPort.clear();

        //get encoders
        final int usigned_offset = 10000; //should match with device offset
        encoders[0] = decodeInt(inputStringArr[0])-usigned_offset;
        encoders[1] = decodeInt(inputStringArr[1])-usigned_offset;
        encoders[2] = decodeInt(inputStringArr[2])-usigned_offset;

        //get q's
        q[0] = decodeFloat(inputStringArr[3]);
        q[1] = decodeFloat(inputStringArr[4]);
        q[2] = decodeFloat(inputStringArr[5]);
        q[3] = decodeFloat(inputStringArr[6]);

        //convert encoder counts to angles
        final float cableRadius = 0.40;//mm
        final float capstanRadius = 3.175;//mm
        final float wheelRadius[] = {65, 29, 29};//mm
        final float numCPR = 1440.0; //360*4 for quadrature
        final float encToAngle0 = 2.*PI/numCPR*(cableRadius+capstanRadius)/(cableRadius+wheelRadius[0]);
        final float encToAngle1 = 2.*PI/numCPR*(cableRadius+capstanRadius)/(cableRadius+wheelRadius[1]);
        final float encToAngle2 = 2.*PI/numCPR*(cableRadius+capstanRadius)/(cableRadius+wheelRadius[2]);

        encAngles[0] = encToAngle0*((float) encoders[0]);
        encAngles[1] = encToAngle1*((float) encoders[1]);
        encAngles[2] = encToAngle2*((float) encoders[2]);

       //get serial transfer time
        int dT = millis() - t1;
        dTavg = 0.99*dTavg +  0.01*((float) dT);
        text("Serial Transfer Time (ms): " + dTavg + "\n", 20, 20);
        t1 = millis();
        return 1; //read all the bytesShape
      }
      else {
        println("!$");//Found $ sign but did not get a complete message*************************************************************************");
//        for (int i = 0; i < 7; i++) {
//          println(inputStringArr[i]);
//        }
      }
    }
  }
  return 0; //else did not read any bytes
}


void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);

  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);

  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);

  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);

  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);

  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  endShape();
}


void drawCube() {  
  pushMatrix();
  translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 + 50, 0);
  translate(px[1], -px[2], px[0]);
  scale(1, 1, 1);

  // a demonstration of the following is at 
  // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
    rotateZ(-Euler[2]);
    rotateX(-Euler[1]);
    rotateY(-Euler[0]);

  buildBoxShape();

  popMatrix();
}


void draw() {
  background(#000000);
  fill(#ffffff);

  //read the device, if its sending, and then send back force
  //readFromDevice();
  if (readFromDevice() == 1) {
    myPort.write(1);
    myPort.write(2);
    myPort.write(3);
  }

  if (hq != null) { // use home quaternion
    quaternionToEuler(quatProd(hq, q), Euler);
    text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
  }
  else {
    quaternionToEuler(q, Euler);
    text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
  }
  //  
  //  textFont(font, 20);
  //  textAlign(LEFT, TOP);


  //get Position
  float l1 = 100, l2 = 103; // arm length (in mm)
  fwdKin(encAngles, l1, l2); //updates px

  //display
  text("Position (mm):\nX: " + px[0] + "\nY: " + px[1] + "\nZ: " + px[2], 400, 60);
  int roll = (int) (180./PI*Euler[0]);
  int pitch = (int) (180./PI*Euler[1]);
  int yaw =  (int) (180./PI*Euler[2]);
  text("Euler (degrees): \nRoll: " + roll + "\nPitch: " + pitch + "\nYaw: " + yaw, 100, 60);


  drawCube();
}



void keyPressed() {
  if (key == 'h') {
    println("pressed h");

    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    hq = quatConjugate(q);
  }
  else if (key == 'n') {
    println("pressed n");
    hq = null;
  }
}

// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

float [] quatProd(float [] a, float [] b) {
  float [] q = new float[4];

  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

  return q;
}

// returns a quaternion from an axis angle representation
float [] quatAxisAngle(float [] axis, float angle) {
  float [] q = new float[4];

  float halfAngle = angle / 2.0;
  float sinHalfAngle = sin(halfAngle);
  q[0] = cos(halfAngle);
  q[1] = -axis[0] * sinHalfAngle;
  q[2] = -axis[1] * sinHalfAngle;
  q[3] = -axis[2] * sinHalfAngle;

  return q;
}

// return the quaternion conjugate of quat
float [] quatConjugate(float [] quat) {
  float [] conj = new float[4];

  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];

  return conj;
}

