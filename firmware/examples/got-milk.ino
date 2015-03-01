#include "WunderbarBridge.h"

Bridge bridge;
int bigScale = A1;
int smallScale = A2;
int bigVal = 0;
int smallVal = 0;
int bigOffset = 0;
int smallOffset = 0;
uint8_t dataOut[2];

void setup() {
   bridge = Bridge(9600);
   bridge.begin();
   
   delay(500);
   bigOffset = analogRead(bigScale);
   smallOffset = analogRead(smallScale);
}

void loop() {
    bigVal = analogRead(bigScale);
    smallVal = analogRead(smallScale);
    Serial.println();
    
    dataOut[0] = uint8_t(bigVal - bigOffset);
    dataOut[1] = uint8_t(smallVal - smallOffset);
    
    Serial.print("Value big scale: ");
    Serial.println(bigVal, DEC);
    Serial.print(", data: ");
    Serial.println(dataOut[0], DEC);
    
    Serial.print("Value small scale: ");
    Serial.println(smallVal, DEC);
    Serial.print(", data: ");
    Serial.println(dataOut[1], DEC);
   
    dataOut[0] = uint8_t(bigVal - bigOffset);
    dataOut[1] = uint8_t(smallVal - smallOffset);

    bridge.sendData(dataOut, 2);
    delay(1000);
}

/* the serialEvent() handler is called on every received data 
from the serial port. */
void serialEvent(){
	bridge.processSerial();
}