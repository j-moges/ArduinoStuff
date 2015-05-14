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

float time = 0;
float oldTime = 0;
int gyroVal;
float angle = 0;

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
 angle += gyroVal;
 
 Serial.println(angle);
  delay(100);
}
