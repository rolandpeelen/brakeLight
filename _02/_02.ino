//Includes
#include <Wire.h>
#include <math.h>
#include <Accelerometer_Compass_LSM303D.h>
#include <Adafruit_WS2801.h>


//Init LED strip
uint8_t dataPin  = 4;
uint8_t clockPin = 2;
Adafruit_WS2801 strip = Adafruit_WS2801(32, dataPin, clockPin); //Initialize LED STRIP


int leftBlinker[5] = {0, 1, 2, 3, 4}; //blinkerLEDs
int rightBlinker[5] = {27, 28, 29, 30, 31}; //blinkerLEDs
int leftBlinkID = 0; //used to step through blinkers
int rightBlinkID = 0; //used to step through blinkers
int lightID = 0;

int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, tiltHeading, pitch, roll, xh, yh, zh, tiltCorrectedPitch;

//Constants
const int leftBlinkerPin = 8; //Setup pin for left blinker
const int rightBlinkerPin = 7; //Setup pin for left blinker
const int ledDelay = 60; //Delay for the LEDs
const double EARTH_GRAVITY_MS2   = 0.980665; //Gravity divided by 10

void setup() {
  //Setup
  Serial.begin(9600);

  Serial.println("Starting");
  //Set brakelights
  for (int i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, 0, 0, 0, 0);
  }

  //Start up the LED Strip
  strip.show();
  strip.begin();

  //Initialize accelerometer
  char rtn = 0;
  rtn = Lsm303d.initI2C();
  if(rtn != 0)  // Initialize the LSM303, using a SCALE full-scale range
  {
    Serial.println("\r\nLSM303D is not found");
    while(1);
  }
  else
  {
    Serial.println("\r\nLSM303D is found");
  }

  //Set pinmodes to input for the buttons
  pinMode(leftBlinkerPin, INPUT);
  pinMode(rightBlinkerPin, INPUT);

  }

void loop() {
  //read states for the buttons
  int leftBlinkerState = digitalRead(leftBlinkerPin);
  int rightBlinkerState = digitalRead(rightBlinkerPin);
  
  // turn Left blinker on:   
  if (leftBlinkerState == HIGH) {     
      for(leftBlinkID=5; leftBlinkID >= 0; leftBlinkID--){
        strip.setPixelColor(leftBlinker[leftBlinkID], 0, 255, 25, 0);
        strip.show();
        delay(ledDelay);
        strip.setPixelColor(leftBlinker[leftBlinkID], 0, 0, 0, 0);
        strip.show();
      }
  }
  
  // turn right blinker on:
  if(rightBlinkerState == HIGH){
    for(rightBlinkID=0; rightBlinkID < 5; rightBlinkID++){
        strip.setPixelColor(rightBlinker[rightBlinkID], 0, 255, 25, 0);
        strip.show();
        delay(ledDelay);
        strip.setPixelColor(rightBlinker[rightBlinkID], 0, 0, 0, 0);
        strip.show();
      }
  }

  Lsm303d.getAccel(accel);
  while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
  Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag


  
  for (int i=0; i<3; i++)
  {
    realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
  }
  heading = Lsm303d.getHeading(mag);
  tiltHeading = Lsm303d.getTiltHeading(mag, realAccel);

  //prints
  Serial.println("**********");
  Serial.print("MagX: ");
  Serial.print(mag[0]);
  Serial.print("-- Magy: ");
  Serial.print(mag[1]);
  Serial.print("-- MagZ: ");
  Serial.println(mag[2]);

  //prints
  Serial.print("AccX: ");
  Serial.print(realAccel[0]);
  Serial.print("-- AccY: ");
  Serial.print(realAccel[1]);
  Serial.print("-- AccZ: ");
  Serial.println(realAccel[2]);

  // see section 1.2 in app note AN3192
  float pitch = asin(-realAccel[X]);
  float roll = asin(realAccel[Y]/cos(pitch));

  xh = mag[X] * cos(pitch) + mag[Z] * sin(pitch);
  yh = mag[X] * sin(roll) * sin(pitch) + mag[Y] * cos(roll) - mag[Z] * sin(roll) * cos(pitch);
  zh = -mag[X] * cos(roll) * sin(pitch) + mag[Y] * sin(roll) + mag[Z] * cos(roll) * cos(pitch);
  float heading = 180 * atan2(xh, xh)/PI;

  if (heading <0)
  heading += 360;

  Serial.print("-- HEADING: ");
  Serial.println(heading);

  // see appendix A in app note AN3192 
  pitch = asin(-realAccel[X]);
  //pitch = 180 * atan2(realAccel[X], realAccel[Z])/PI;
  roll = asin(realAccel[Y]/cos(pitch));
  //roll = 180 * atan2(realAccel[Y], realAccel[Z])/PI;

  float xh = mag[X] * cos(pitch) + mag[Z] * sin(pitch);
  float yh = mag[X] * sin(roll) * sin(pitch) + mag[Y] * cos(roll) - mag[Z] * sin(roll) * cos(pitch);
  float zh = -mag[X] * cos(roll) * sin(pitch) + mag[Y] * sin(roll) + mag[Z] * cos(roll) * cos(pitch);
  tiltCorrectedPitch = 180 * atan2(yh, xh)/PI;
  if (yh >= 0){
    } else{
      tiltCorrectedPitch = 360 + tiltCorrectedPitch;
    }

  printValues();

  pitch = 180 * pitch / PI;
  float affection = pitch / 90 * 100 * -1; //90 is maximum angle


  float ax = realAccel[X];
  float axx;

  if(axx > 0.85){
    //Set indicatorLights
    for(leftBlinkID=5; leftBlinkID >= 0; leftBlinkID--){
      strip.setPixelColor(leftBlinker[leftBlinkID], 0, 255, 25, 0);
    }
    for(rightBlinkID=0; rightBlinkID < 5; rightBlinkID++){
      strip.setPixelColor(rightBlinker[rightBlinkID], 0, 255, 25, 0);
    }
    strip.show();
    delay(ledDelay * 2);
    for(leftBlinkID=5; leftBlinkID >= 0; leftBlinkID--){
      strip.setPixelColor(leftBlinker[leftBlinkID], 0, 0, 0, 0);
    }
    for(rightBlinkID=0; rightBlinkID < 5; rightBlinkID++){
      strip.setPixelColor(rightBlinker[rightBlinkID], 0, 0, 0, 0);
    }
    strip.show();
    delay(ledDelay * 2);
  }else if(axx > 0.2){
    for (int i=0; i < strip.numPixels(); i++) {
      int intensity = 255 * ax;
      strip.setPixelColor(i, 0, intensity, 0, 0);
    }
    //Set indicatorLights
    for(leftBlinkID=0; leftBlinkID < sizeof(leftBlinker); leftBlinkID++){
        strip.setPixelColor(leftBlinker[leftBlinkID], 0, 0, 0, 0);
    }
    //Set indicatorLights
    for(rightBlinkID=0; rightBlinkID < sizeof(rightBlinker); rightBlinkID++){
        strip.setPixelColor(rightBlinker[rightBlinkID], 0, 0, 0, 0);
    }
    strip.show();
  }else{
    for (int i=0; i < strip.numPixels(); i++) {
      int intensity = 255 * ax;
      strip.setPixelColor(i, 0, 0, 0, 0);
    }
  }
  
  strip.show();
  delay(5000);
}

void printValues()
{  
  
  /*
  Serial.println("Magneto of X,Y,Z is: ");
    Serial.print(mag[X]);
    Serial.print(" - ");
    Serial.print(mag[Y]);
    Serial.print(" - ");
    Serial.println(mag[Z]);

  Serial.println("---");
  Serial.print("The pitch: ");
  Serial.print(pitch); // this only works if the sensor is level
  Serial.println(" radians");

  Serial.println("---");
  Serial.print("The tiltCorrectedPitch: ");
  Serial.print(xh); // this only works if the sensor is level
  Serial.println(" degrees");

  

  Serial.println("---");

  Serial.print("The roll: ");
  Serial.print(roll); // this only works if the sensor is level
  Serial.println(" radians");
  */
}
