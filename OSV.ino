/*What to do. Change back movement to be slightly longer.
Find out how long it takes to travel to pool. 
*/
#include <Servo.h>
#include <SharpIR.h>
#include "enes100.h"
#include <SoftwareSerial.h>
#include <OneWire.h>
//Servo Init
Servo steeringServo;
Servo speedController;
Servo armServo;
int delayValue = 10; //Delay in ms
int servoMin = 0;
int servoMax = 180;
int powerMin = -100;
int powerMax = 100;
int steerMin = -90;
int steerMax = 90;
int armMin = -90;
int armMax = 90;
int steerNeutral = 5;
int throttleNeutral = 0;
int armNeutral = 1;
float temp=0;

//IR Init
SharpIR sharp(A0, 25, 93, 1080);

//Comm Init
SoftwareSerial mySerial(5,6);
enes100::RfClient<SoftwareSerial> rf(&mySerial);
enes100::Marker marker;

//Temp Init
OneWire ds(2);

int stage;
//Navigation flag
void setup(){
	mySerial.begin(9600);
	Serial.begin(9600);

	//Servo Setup
	steeringServo.attach(9);
 	speedController.attach(10);
 	armServo.attach(11);
  	delay(1000);
	drive(0);
        steer(0);
        moveArm(0);

	//Comm Setup
	rf.resetServer();
	rf.sendMessage("Team H2O Connected");

	//IR Setup
	pinMode(A0, INPUT);

        stage = 0;
}

void loop(){
  rf.receiveMarker(&marker, 136);
  if(stage == 0){
    if(marker.theta <= 1.27 || marker.theta >= 1.87){
      rf.sendMessage("Stage 0: Rotating to 90");
      if(marker.theta >= 1.87){
        turnInPlace(1);
      } else {
        turnInPlace(-1);
      }
      delay(100);
    } else {
      stage = 1;
    }
  } else if(stage == 1){
    if(marker.y <= 0.24 || marker.y >= 0.29){
      rf.sendMessage("Stage 1: Moving to y = 0.26");
      if(marker.y >= 0.29){
        drive(-60);
      } else {
        drive(35);
      }
      delay(400);
      drive(0);
      delay(100);
    } else {
      stage = 2;
    }
  } else if(stage == 2){
    if(marker.theta <= -0.15 || marker.theta >= 0.15){
      rf.sendMessage("Stage 2: Turning to theta = 0");
      if(marker.theta >= 0.0){
        turnInPlace(1);
      } else {
        turnInPlace(-1);
      }
    } else{
      stage = 3;
    }
  } else if(stage == 3){
    rf.sendMessage("Stage 3: Moving to x = 3.18");
    if(marker.theta>0.1){
      steer(7);
    } else if(marker.theta<-0.1){
       steer(-7);
    } else{
      steer(0);
    }
      
    if(marker.x < 3.19){
      drive(55);
      delay(400);
      drive(0);
      delay(200);
    } else if(marker.x > 3.35){
      drive(-60);
      delay(300);
      drive(0);
      delay(200);
    } else {
      steer(0);
      stage = 4;
    }
  } else if(stage == 4){
    rf.sendMessage("Stage 4: Turning to theta = 90");
    if(marker.theta <= 1.20){
      turnInPlace(-1);
      delay(100);
    } else {
      stage = 5;
    }
  } else if(stage == 5){
    rf.sendMessage("Stage 5: Moving to y = 1.3");
    if(marker.y < 1.3){
      drive(55);
      delay(400);
      drive(0);
    } else if (marker.x<3.19 || marker.x>3.35) {
     // rf.sendMessage("Water detected");
      stage = 6;
      drive(0);
    } else {
      stage = 9;
      drive(0);
    }
    
  } else if(stage == 6){
    if(marker.theta <= -0.3 || marker.theta >= 0.3){
      rf.sendMessage("Stage 6: Turning to theta = 0");
      if(marker.theta >= 0.3){
        turnInPlace(1);
      } else {
        turnInPlace(-1);
      }
    } else{
      stage = 7;
    }
  } else if(stage == 7){
    rf.sendMessage("Stage 7: Moving to x = 3.18");
    if(marker.x < 3.19){
      drive(35);
      delay(250);
      drive(0);
    } else if(marker.x > 3.35){
      drive(-40);
      delay(300);
      drive(0);
    } else {
      stage = 8;
    }
  } else if(stage == 8){
    rf.sendMessage("Stage 8: Turning to theta = 90");
    if(marker.theta <= 1.20){
      turnInPlace(-1);
      delay(100);
    }else if(marker.y<1.31){
      drive(30);
      delay(250);  
    } else {
      stage = 9;
      drive(0);
    }
} else if(stage == 9){
    rf.sendMessage("Stage 9: Dropping sensors.");
    moveArm(-45);
    delay(30000);
    char tempVal[10];
    for(int i = 0; i<100; i++){
      rf.sendMessage("Temp: ");   
      dtostrf(readTemp(), 2,2, tempVal);
      rf.sendMessage(tempVal);
      rf.resetServer();
      delay(1000);
    }
    }
}
int readIR(){
	return sharp.distance();
}
//port 7
float readTemp(){
	byte data[12];
	byte addr[8];

	if ( !ds.search(addr)) {
	    //no more sensors on chain, reset search
	    ds.reset_search();
	    return -1000;
	}

	if ( OneWire::crc8( addr, 7) != addr[7]) {
	    Serial.println("CRC is not valid!");
	    return -1000;
	}

	if ( addr[0] != 0x10 && addr[0] != 0x28) {
	    Serial.print("Device is not recognized");
	    return -1000;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44,1); // start conversion, with parasite power on at the end

	byte present = ds.reset();
	ds.select(addr);    
	ds.write(0xBE); // Read Scratchpad

	
	for (int i = 0; i < 9; i++) { // we need 9 bytes
	  data[i] = ds.read();
	}
	
	ds.reset_search();
	
	byte MSB = data[1];
	byte LSB = data[0];

	float tempRead = ((MSB << 8) | LSB); //using two's compliment
	float TemperatureSum = tempRead / 16;
	
	return TemperatureSum;
}
//pin A1
int readSalinity(){
	return analogRead(A1);
}

void printAngle (int angle) {
	Serial.print ("Servo Angle: ");
	Serial.print (angle);
	Serial.println ();
}

void drive (int power) { //accepts values (integers) from -100 to 100
	if (power >= powerMin && power <= powerMax) {
		int throttleValue = map (power, powerMin, powerMax, servoMin, servoMax) + throttleNeutral;
		if (throttleValue < servoMin) {
			throttleValue = servoMin;
		}
    
		if (throttleValue > servoMax) {
			throttleValue = servoMax;
		}
		speedController.write (throttleValue);
	}
}

void steer (int angle) { //accepts angles (integers) from -90 to 90 degrees
	if (angle >= steerMin && angle <= steerMax) {
		int steeringValue = map (angle, steerMin, steerMax, servoMin, servoMax) + steerNeutral;
    
		if (steeringValue < servoMin) {
			steeringValue = servoMin;
		}
    
		if (steeringValue > servoMax) {
			steeringValue = servoMax;
		}
		steeringServo.write (steeringValue);
	}
}

void moveArm (int angle) { //accepts angles (integers) from -90 to 90 degrees
	if (angle >= armMin && angle <= armMax) {
		int armValue = map (angle, steerMin, steerMax, servoMin, servoMax) + armNeutral;
    
		if (armValue < armMin) {
			armValue = servoMin;
		}
    
		if (armValue > servoMax) {
			armValue = servoMax;
		}
	armServo.write (armValue);
	}
}

int getSteeringAngle () { //returns the last position the steering servo was set to
	return map (steeringServo.read(), servoMin, servoMax, steerMin, steerMax);
}

int getThrottlePosition () { //returns the last position the throttle was set to
	return map (speedController.read(), servoMin, servoMax, powerMin, powerMax);
}

int getArmAngle () { //returns the last position the arm servo was set to
	return map (armServo.read(), servoMin, servoMax, armMin, armMax);
}
//right is 1 left is -1
void turnInPlace(int turn){
	if(turn == 1){
		steer(45);
		drive(35);
		delay(300);
                steer(0);
                drive(0);
                delay(300);
		steer(-45);
		drive(-60);
		delay(500);
                steer(0);
                drive(0);
                delay(300);
		steer(45);
		drive(35);
		delay(300);
		steer(0);
		drive(0);
                delay(300);	
	}	
	else{				
		steer(-45);
		drive(35);
		delay(300);
                steer(0);
                drive(0);
                delay(300);
		steer(45);
		drive(-60);
		delay(500);
                steer(0);
                drive(0);
                delay(300);
		steer(-45);
		drive(35);
		delay(300);
		steer(0);
		drive(0);
                delay(300);	
	}
}












