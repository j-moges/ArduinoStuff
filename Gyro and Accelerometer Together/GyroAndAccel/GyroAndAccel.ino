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

/*
  @author Michael Althoff
  From Adafruit ADXL335 accelerometer to arduino
  connect Accelerometer -> Arduino
  Vin -> 5V on Arduino
  GND -> GND
  Xout -> A0
  Yout -> A1
  Zout -> A2
  
  Gyro
  Vin -> 5V
  GND -> GND
  SDA -> SDA
  SCL -> SCL
*/

//Constants for pinouts for Accelerometer
const int xPin = A0;
const int yPin = A1;
const int zPin = A2;

void setup() 
{
  Serial.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
   if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  Serial.println("X\tY\tZ\tXAccel\tYAccel\tZAccel");
}

void loop() 
{
  gyro.read();
  //Serial.print("X: "); 
  Serial.print((int)gyro.data.x);  Serial.print("\t");
  //Serial.print("Y: "); 
  Serial.print((int)gyro.data.y);  Serial.print("\t");
  //Serial.print("Z: "); 
  Serial.print((int)gyro.data.z);  Serial.print("\t");
  delay(100);
  //Serial.print("X Accel: "); 
  Serial.print(analogRead(xPin)); Serial.print("\t");
  //Serial.print("Y Accel: "); 
  Serial.print(analogRead(yPin)); Serial.print("\t");
  //Serial.print("Z Accel: "); 
  Serial.print(analogRead(zPin));
  Serial.println();
  delay(100);
}
