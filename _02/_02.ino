//Includes
#include <Wire.h>
#include <math.h>
#include <SPI.h>
#include <ADXL345.h>
#include <Adafruit_WS2801.h>

//Init LED strip
uint8_t dataPin  = 4;
uint8_t clockPin = 2;
Adafruit_WS2801 strip = Adafruit_WS2801(32, dataPin, clockPin); //Initialize LED STRIP

//Init Accelerometer
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

int leftBlinker[5] = {0, 1, 2, 3, 4}; //blinkerLEDs
int rightBlinker[5] = {27, 28, 29, 30, 31}; //blinkerLEDs
int lights[10] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}; //blinkerLEDs
int leftBlinkID = 0; //used to step through blinkers
int rightBlinkID = 0; //used to step through blinkers
int lightID = 0;

//Constants
const int leftBlinkerPin = 8; //Setup pin for left blinker
const int rightBlinkerPin = 7; //Setup pin for left blinker
const int ledDelay = 60; //Delay for the LEDs
const double EARTH_GRAVITY_MS2   = 0.980665; //Gravity divided by 10

void setup() {
  //Setup
  Serial.begin(9600);

  //Set brakelights
  for (int i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, 0, 0, 0, 0);
  }

  for(lightID=0; lightID < 10; lightID++){
        strip.setPixelColor(lights[lightID], 0, 255, 0, 0);
      }

  //Start up the LED Strip
  strip.show();
  strip.begin();

  //Initialize accelerometer
  adxl.powerOn();

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

  //Setup accelerometer variables
  double ax,ay,az;
  double xyz[3];
  
  adxl.getAcceleration(xyz);
  
  ax = xyz[0];
  Serial.println(ax);
    
  if(ax > 0.85){
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
  }else if(ax > 0.1){
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

   for(lightID=0; lightID < 10; lightID++){
        strip.setPixelColor(lights[lightID], 0, 255, 0, 0);
      }
  
  strip.show();
  delay(10);
}
