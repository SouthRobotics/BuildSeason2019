//PixyCam --------------------------------------------------------

#include <Pixy2.h>
Pixy2 pixy; 
int x, y;
int area, tempArea, angle;
byte valueFromRoboRio;
const byte readValue = 0x12; //can be anything just needs to match in RoboRio code

//SparkFun Ultrasonic Sensor --------------------------------------------------------
//https://learn.sparkfun.com/tutorials/qwiic-distance-sensor-vl53l1x-hookup-guide?_ga=2.204939472.1584633807.1548511782-934199408.1548001094

/*#include <Wire.h>
#include "SparkFun_VL53L1X.h"
SFEVL53L1X distanceSensor(Wire);
int distance;
float distanceInches;
float distanceFeet;
const byte ultrasonicValue = 0x13;*/

void setup() {
//PixyCam --------------------------------------------------------

  pixy.init();
  x = -1;
  y = 0;
  area = 0;
  tempArea = 0;
  valueFromRoboRio = 0;
  pixy.setLamp(1, 1); //turns on flashlight top and bottom

//Ultrasonic --------------------------------------------------------

 /* Wire.begin();
  distance = -1;
  distanceInches = -1;
  distanceFeet = -1;
  distanceSensor.init();*/

//All --------------------------------------------------------

  Serial.begin(9600); //connection through usb. 9600 must match number in RoboRio code
}

void loop() {

  if ( Serial.available() )
  {
    valueFromRoboRio = Serial.read();
    
//PixyCam --------------------------------------------------------

    if ( valueFromRoboRio == readValue )
    {
      pixy.ccc.getBlocks(); //blocks are detected objects
      x = -1;
      y = 0;
      area = 0;
      tempArea = 0;
      angle = 0;
      //find largest block (closest to robot), and save x,y,area
      for ( int i = 0; i < pixy.ccc.numBlocks; i++ )
      {
          tempArea = (int)(pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height);
          if ( tempArea > area )
          {
            area = tempArea;
            x = (int)pixy.ccc.blocks[i].m_x;
            y = (int)pixy.ccc.blocks[i].m_y;
            angle = (int)pixy.ccc.blocks[i].m_angle;
          }
      }
      Serial.print(x);
    }
  }

//Ultrasonic --------------------------------------------------------
/*  if ( valueFromRoboRio == ultrasonicValue ){
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    distanceInches = distance * 0.0393701;
    distanceFeet = distanceInches / 12.0;
    //Serial.print(distanceInches);
  }*/

//Both --------------------------------------------------------

  delay(50);  // in ms, gives computer time to process data
}