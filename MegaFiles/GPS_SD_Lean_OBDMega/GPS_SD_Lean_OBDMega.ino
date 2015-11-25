/*
Developed with libraries from:
https://github.com/adafruit
https://github.com/stanleyhuangyc/Freematics/tree/master/libraries


---------------------------------------
          WIRING
---------------------------------------
Switch on shield should be on SoftSerial

SPI - Jump these wires
Pin      Pin
13   ->  52
12   ->  50
11   ->  51
10   ->  53

BNO055     Mega
VIN    ->  5v
GND    ->  GND
SDA    ->  SDA
SCL    ->  SCL

From GPS Logger Shield to Mega
------------------------------
Shield      Mega
TX      ->  RX3
RX      ->  TX3

From OBD2 Adapter to Mega
-----------------------------
Adapter      Mega
White    ->  TX1
Yellow   ->  RX1
Red      ->  5V
Black    ->  GND


---------------------------------------
CSV Format
---------------------------------------
latitude, longitude, hour:minute:seconds.milliseconds, GPS speed (MPH), lean angle, RPM, throttle position
GPS Location, Time, GPS Speed(MPH), Lean Angle, RPM, Throttle Position
*/
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <OBD.h>

#define mySerial Serial3

Adafruit_GPS GPS(&mySerial);

unsigned long delayTimer;  //always use unsigned long for Timer (using millis())

COBD obd;

/* Set the delay between fresh samples - BNO055 */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  true
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Set pin(s) to be used
#define chipSelect 10
#define ledPin 13

//Name of the file
File logfile;

//boolean used to write column headings one time at top of file
boolean firstRun = true;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A') + 10;
}

// blink out an error code
void error(uint8_t errno) {
  while (1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
      delay(200);
    }
  }
}

char filename[15];
void setup()
{
  //Flush all of the serial lines
  Serial.flush();
  Serial1.flush();
  Serial2.flush();
  Serial3.flush();
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Test GPS and SD Logging");

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(53, OUTPUT); //For Arduino Mega

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }

  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if ( ! logfile ) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to ");
  Serial.println(filename);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);


  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  //----------------------------------
  // BNO055 SETUP
  //----------------------------------

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  /*
    OBD SETUP
  */
  //start OBD2 communication with adapter
  obd.begin();
  int obdTries = 0;
  //keep trying to connect until successful, if it takes too long or no connection skip OBD2 logging
  while (!obd.init() && (obdTries < 5)) {
    Serial.print("No OBD2 Connection!  Tries: ");
    Serial.println(obdTries + 1);
    delayTimer = millis();
    if((millis() - delayTimer) > 500){
      obdTries++;
    }
  };
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//          MAIN LOOP
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop()
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (GPS.fix) {
    //open the file
    logfile = SD.open(filename, FILE_WRITE);

    //Print out the column headings only on the first run through so it is
    //the first line of the CSV file
    if (firstRun == true) {
      Serial.println("Latitude Longitude DDM, Hours:Minutes:Seconds, GPS speed, lean angle, RPM, throttle position");
      logfile.println("Latitude Longitude DDM, Hours:Minutes:Seconds, GPS speed, lean angle, RPM, throttle position");
      firstRun = false;
    }
    //DDM GPS conversion
    String lat = String(GPS.latitude, 4);
    lat = lat + (GPS.lat); //tells whether N/S
    String lon = String(GPS.longitude, 4);
    lon = lon + (GPS.lon); //tells whether E/W
    //Serial.print("Lat and Lon: ");
    //Serial.print(lat);
    //Serial.println(lon);
    //3130.1899N 8457.8662W

    //latitude will not exceed 10 characters, longitude can be 10 or 11 characters
    String latFirst = lat.substring(0, 2);
    String latSecond = lat.substring(2, 10);
    lat = latFirst + " " + latSecond;
    //add the + or - for gps interpretation
    if (lat.substring(10) == "N") {
      lat = "+" + lat;
    } else {
      lat = "-" + lat;
    }

    //longitude will be 10 or 11 characters, so if not 10, will be 11
    if (lon.length() == 10) {
      String lonFirst = lon.substring(0, 2);
      String lonSecond = lon.substring(2, 10);
      lon = lonFirst + " " + lonSecond;
      if (lon.substring(10) == "W") {
        lon = "-" + lon;
      } else {
        lon = "+" + lon;
      }

    } else {
      String lonFirst = lon.substring(0, 3);
      String lonSecond = lon.substring(3, 11);
      lon = lonFirst + " " + lonSecond;
      if (lon.substring(11) == "W") {
        lon = "-" + lon;
      } else {
        lon = "+" + lon;
      }
    }

    Serial.print(lat + " " + lon + ", ");
    //Writes data to SD card
    logfile.print(lat + " " + lon + ", ");

    //GPS Time
    Serial.print(GPS.hour, DEC); Serial.print(":");
    Serial.print(GPS.minute, DEC); Serial.print(":");
    Serial.print(GPS.seconds, DEC); Serial.print(", ");
    //Write time to SD card
    logfile.print(GPS.hour, DEC); logfile.print(':');
    logfile.print(GPS.minute, DEC); logfile.print(':');
    logfile.print(GPS.seconds, DEC); logfile.print(", ");

    //GPS Speed
    float currentSpeed = GPS.speed;
    currentSpeed = mphConversion(currentSpeed);
    Serial.print(currentSpeed); Serial.print(", ");
    logfile.print(currentSpeed); logfile.print(", ");


    /* Get a new sensor event - BNO055 */
    sensors_event_t event;
    bno.getEvent(&event);
    //float leanAngle = (event.orientation.y, 4);
    //Convert lean angle to String for logging
    //String leanAngleString = String(leanAngle);
    /* Display the floating point data */
    Serial.print(event.orientation.y, 4); Serial.print(", ");
    //Log the Lean Angle
    logfile.print(event.orientation.y, 4); logfile.print(", ");
    delay(BNO055_SAMPLERATE_DELAY_MS);
    //Log the throttle position and RPM by calling the OBD2 function
    obdInfo();


    //close file after writing
    logfile.close();

  }
}

void obdInfo() {
  int rpmValue, throttlePos;
  if (obd.read(PID_RPM, rpmValue)) {
    if (obd.read(PID_THROTTLE, throttlePos)) {
      Serial.print(rpmValue); Serial.print(", "); Serial.println(throttlePos);
      logfile.print(rpmValue); logfile.print(", "); logfile.println(throttlePos);
    }
  } else {
    Serial.println(",");
    logfile.println(",");
  }
}

//Convert speed in knots to miles per hour
float mphConversion(float knots) {
  float mph = knots * 1.15077945;
  return mph;
}
