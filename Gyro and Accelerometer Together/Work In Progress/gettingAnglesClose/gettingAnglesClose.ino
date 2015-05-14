#include <Wire.h> 
#include <Adafruit_L3GD20.h>

// Comment this next line to use SPI
#define USE_I2C

#ifdef USE_I2C
  // The default constructor uses I2C
  Adafruit_L3GD20 gyro;
#else
  // To use SPI, you have to define the pins
  #define GYRO_CS 4 // labeled CS
  #define GYRO_DO 5 // labeled SA0
  #define GYRO_DI 6  // labeled SDA
  #define GYRO_CLK 7 // labeled SCL
  Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif

//gyro stuff
float time = 0;
float oldTime = 0;
int gyroVal;
float angle = 0;
//end gyro stuff

//Analog read pins
const int xPin = 0;
const int yPin = 1;
const int zPin = 2;

//The minimum and maximum values that came from
//the accelerometer while standing still
//You very well may need to change these
//250
int minVal = 265;
int maxVal = 409;


//to hold the caculated values
double x;
double y;
double z;


//to average a level at start of program
float total = 0;
int average = 0;
void setup(){
  
  Serial.begin(9600);
  // Try to initialise and warn if we couldn't detect the chip
   if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
}

void loop(){

   //read the analog values from the accelerometer
  int xRead = analogRead(xPin);
  int yRead = analogRead(yPin);
  int zRead = analogRead(zPin);
  
    //convert read values to degrees -90 to 90 - Needed for atan2
  int xAng = map(xRead, minVal, maxVal, -90, 90);
  int yAng = map(yRead, minVal, maxVal, -90, 90);
  int zAng = map(zRead, minVal, maxVal, -90, 90);
  
    //Caculate 360deg values like so: atan2(-yAng, -zAng)
  //atan2 outputs the value of -π to π (radians)
  //We are then converting the radians to degrees
  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
  
  Serial.print("Y before fix: ");
  Serial.print(y);
  Serial.print("\t");
  
  
 gyro.read();
 
 //store old reading time to get the difference in the times
 oldTime = time;
 //how long the program has been running (in milliseconds - 4000 = 4 seconds)
 time = millis();
 
 float timeDiff = time - oldTime;
 //convert milliseconds
 timeDiff = timeDiff *.001;
 //degrees/second * seconds cancels seconds leaving only degrees
 gyroVal = gyro.data.y * timeDiff;
 

 //track a stable angle
 //angle += gyroVal;
 //Serial.print("Gyro Angle: ");
 //Serial.print(angle);
 
 
  //if statement to fix level error
 /*if(y < 360 && y > 354){
  y = y - 354;
 }
 //if statement for negative lean (leaning to the left)
 if(y < 354 && y > 260){
  y = y-360; 
 }
 Serial.print("Y after fix: ");
 Serial.print(y);
 Serial.print("\t");*/
 
 //gyroVal is the current angle from the gyroscope
 //y is the angle from the accelerometer
 angle = .98 * (angle + gyroVal) + (.02*(y));
 //complementary filter above
 

 
 Serial.print("\tFiltered Angle: ");
 Serial.print(angle);
 Serial.print("\tAccel Angle: ");   
 Serial.println(y);

/*
  I fucked up somewhere and now the angles are all over the place.
  Check the y correction, that's really all I messed with but I thought
  I put it back to where it was...
*/

  delay(100);
}
