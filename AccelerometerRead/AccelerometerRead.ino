/*
  @author Michael Althoff
  From Adafruit ADXL335 accelerometer to arduino
  connect Accelerometer -> Arduino
  Vin -> 5V on Arduino
  GND -> GND
  Xout -> A0
  Yout -> A1
  Zout -> A2
  
*/

//Constants for pinouts
const int xPin = A0;
const int yPin = A1;
const int zPin = A2;

void setup(){
  //start serial communication
   Serial.begin(9600); 
}

void loop(){
  
  Serial.print("X Accel: ");
  Serial.print(analogRead(xPin));
  // print a tab between values:
  Serial.print("\t");
  Serial.print("Y Accel: ");
  Serial.print(analogRead(yPin));
  // print a tab between values:
  Serial.print("\t");
  Serial.print("Z Accel: ");
  Serial.print(analogRead(zPin));
  Serial.println();
  
  // delay before next reading:
  delay(100);

}
