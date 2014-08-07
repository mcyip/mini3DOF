mini3DOF
========

From Sparkfun Comment:
"I have this module talking to an NXP LPC1343 at up to 1600 samples per second. You need the I2C bus running at 400kHz to achieve this sort of sampling rate. I suspect your issue is the Arduino Wire library. For some reason this sets the clock rate to 100kHz. See the following forum post for how to change http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1241668644.

My code also reads all 6 byte blocks from each of the sensors in a single I2C transaction. This is much quicker than 6 I2C calls which is what some of the code I have seen does.

You might also have some issue with serial baud rates. I had to push up to 500000 baud to get enough bandwidth to push all data to the PC.

A CRO is really helping in tracking down what is happening here. Set a digital output at the start of the sensor read routine and clear it at the end. You can then tune the I2C settings to get this to the correct rate."


SOOOOOO go into TWI.h in the Wire folder and change it from 
  #ifndef TWI_FREQ
  #define TWI_FREQ 100000L
  #endif
  
  to 
  
  #ifndef TWI_FREQ
  #define TWI_FREQ 400000L
  #endif
