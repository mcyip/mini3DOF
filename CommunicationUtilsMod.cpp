#include "CommunicationUtilsMod.h"

//New addition: send an int as hex chars, which lets it be less than 32 bits.
void printIntArrAsHex(int intArr[], int numInts){
  for(int i = 0; i < numInts; i++){
    Serial.print(intArr[i],HEX);
    Serial.print(",");
  }
}


