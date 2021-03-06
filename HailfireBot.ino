/*
 Name:		ArduinoControlExploration.ino
 Created:	9/15/2016 12:54:44 PM
 Author:	Jacob Sacco
*/

#pragma GCC optimize ("O2")

bool led;

#include "Drone.h"
#include "Drone_definitions.h"
#include "Comm_definitions.h"
#include <Wire.h>
#include <I2Cdev.h>

Drone drone;
int update_counter = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	Serial1.begin(9600);
	Serial.setTimeout(5);
	Serial1.setTimeout(5);
	pinMode(3, OUTPUT);
	drone.init();
}

// the loop function runs over and over again until power down or reset
void loop() {
	if (drone.is_ready()) {
		drone.update();
		update_counter++;
	}
	else {
		send_message(0b11111111, "Drone not ready");
		delay(20000);
	}
	//delay(5);
}
