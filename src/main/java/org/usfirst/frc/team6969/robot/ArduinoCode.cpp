//PixyCam --------------------------------------------------------

#include <Pixy2.h>
Pixy2 pixy; 
int x, y;
int area, tempArea, angle;
byte valueFromRoboRio;
const byte readValue = 0x12; //can be anything just needs to match in RoboRio code

//SparkFun Ultrasonic VL53L1X Sensor --------------------------------------------------------
//https://learn.sparkfun.com/tutorials/qwiic-distance-sensor-vl53l1x-hookup-guide?_ga=2.204939472.1584633807.1548511782-934199408.1548001094

/*#include <Wire.h>
#include "SparkFun_VL53L1X.h"
SFEVL53L1X distanceSensor(Wire);
int distance;
float distanceInches;
float distanceFeet;
const byte ultrasonicValue = 0x13;*/

//SparkFun Ultrasonic HC-SR04 Sensor --------------------------------------------------------


/**
 * HC-SR04 Demo
 * Demonstration of the HC-SR04 Ultrasonic Sensor
 * Date: August 3, 2016
 * 
 * Description:
 *  Connect the ultrasonic sensor to the Arduino as per the
 *  hardware connections below. Run the sketch and open a serial
 *  monitor. The distance read from the sensor will be displayed
 *  in centimeters and inches.
 * 
 * Hardware Connections:
 *  Arduino | HC-SR04 
 *  -------------------
 *    5V    |   VCC     
 *    7     |   Trig     
 *    8     |   Echo     
 *    GND   |   GND
 *  
 * License:
 *  Public Domain
 */

// Pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;


void setup() {
//PixyCam --------------------------------------------------------

  pixy.init();
  x = -1;
  y = 0;
  area = 0;
  tempArea = 0;
  valueFromRoboRio = 0;
  pixy.setLamp(1, 1); //turns on flashlight top and bottom

//SparkFun Ultrasonic VL53L1X Sensor --------------------------------------------------------

 /* Wire.begin();
  distance = -1;
  distanceInches = -1;
  distanceFeet = -1;
  distanceSensor.init();*/

//SparkFun Ultrasonic HC-SR04 Sensor --------------------------------------------------------

// The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

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

//SparkFun Ultrasonic VL53L1X Sensor --------------------------------------------------------
/*  if ( valueFromRoboRio == ultrasonicValue ){
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    distanceInches = distance * 0.0393701;
    distanceFeet = distanceInches / 12.0;
    //Serial.print(distanceInches);
  }*/

//SparkFun Ultrasonic HC-SR04 Sensor --------------------------------------------------------
    unsigned long t1;
    unsigned long t2;
    unsigned long pulse_width;
    float cm;
    float inches;
    String cmString;
  
    // Hold the trigger pin high for at least 10 us
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    // Wait for pulse on echo pin
    while ( digitalRead(ECHO_PIN) == 0 );
  
    // Measure how long the echo pin was held high (pulse width)
    // Note: the micros() counter will overflow after ~70 min
    t1 = micros();
    while ( digitalRead(ECHO_PIN) == 1);
    t2 = micros();
    pulse_width = t2 - t1;
  
    // Calculate distance in centimeters and inches. The constants
    // are found in the datasheet, and calculated from the assumed speed 
    //of sound in air at sea level (~340 m/s).
    cm = pulse_width / 58.0;
    inches = pulse_width / 148.0;
    cmString = cm;
Serial.println("hi");
//Both --------------------------------------------------------
    // Print out results
    if ( pulse_width > MAX_DIST ) {
      Serial.println(x + ",");
    } else {
      Serial.print(x + "," + cmString);
    }
  }
  delay(60);  // in ms, gives computer time to process data
 
  }
}