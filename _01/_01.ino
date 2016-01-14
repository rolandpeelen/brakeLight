
/*****************************************************************************/
//	Function:    Get the accelemeter of X/Y/Z axis and print out on the 
//					serial monitor.
//  Hardware:    3-Axis Digital Accelerometer(��16g)
//	Arduino IDE: Arduino-1.0
//	Author:	 Frankie.Chu		
//	Date: 	 Jan 11,2013
//	Version: v1.0
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include <Wire.h>
#include <ADXL345.h>

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

//Variables

//Constants
double EARTH_GRAVITY_MS2   = 0.980665; //Gravity divided by 10

void setup(){
  Serial.begin(9600);
  adxl.powerOn();
}

void loop(){
  double ax,ay,az;
	double xyz[3];
  
	adxl.getAcceleration(xyz);
  
	ax = xyz[0];
	ay = xyz[1];
	az = xyz[2] - EARTH_GRAVITY_MS2; //correct for earths gravitational pull

  Serial.print("X=");
  Serial.print(ax);
  Serial.print("Y=");
  Serial.print(ay);
  Serial.print("Z=");
  Serial.print(az);
 
	delay(100);
}
