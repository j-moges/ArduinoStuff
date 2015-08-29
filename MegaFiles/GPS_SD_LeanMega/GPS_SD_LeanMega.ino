/*
Converts the GPS location so that you can directly input into google maps
*/


// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX3
//   Connect the GPS RX (receive) pin to matching TX3

// If you're using the Adafruit GPS shield, change
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
//SoftwareSerial mySerial(8, 7);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;
#define mySerial Serial3


Adafruit_GPS GPS(&mySerial);


/* Set the delay between fresh samples - BNO055 */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  true
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Set pin(s) to be used
#define chipSelect 10
#define ledPin 13

//Name of the file
File logfile;


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
  /*
  if (SD.errorCode()) {
   putstring("SD error: ");
   Serial.print(card.errorCode(), HEX);
   Serial.print(',');
   Serial.println(card.errorData(), HEX);
   }
   */
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

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Test GPS and SD Logging");

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(53, OUTPUT);

  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect, 11, 12, 13)) {
  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("Card init. failed!");
    error(2);
  }
  //char filename[15];
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
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

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


  //Title line, add or remove as you add and remove logging items
  logfile.println("GPS Location, Time, GPS Speed, Lean Angle");

  //----------------------------------
  // BNO055 SETUP
  //----------------------------------
  Serial.begin(9600);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  //if (GPSECHO)
  //if (c) UDR0 = c;
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
void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    //if (GPSECHO)
    //if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }


  if (GPS.fix) {
    //open the file
    logfile = SD.open(filename, FILE_WRITE);

    String lat = String(GPS.latitude, 4);
    lat = lat + (GPS.lat); //tells whether N/S
    String lon = String(GPS.longitude, 4);
    lon = lon + (GPS.lon); //tells whether E/W
    //Serial.print("Lat and Lon: ");
    //Serial.print(lat);
    //Serial.println(lon);
    //3530.1899N 8257.8662W

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

    //-------------
    //  GPS TIME & Speed (knots)
    //-------------
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print(", ");
    
    logfile.print(GPS.hour, DEC); logfile.print(':');
    logfile.print(GPS.minute, DEC); logfile.print(':');
    logfile.print(GPS.seconds, DEC); logfile.print(", ");
    
    //SPEED (knots)
    //MPH = 1.15077945*knots
    Serial.print(GPS.speed*1.15077945); Serial.print(", ");
    logfile.print(GPS.speed*1.15077945); logfile.print(", ");

    /* Get a new sensor event - BNO055 */
    sensors_event_t event;
    bno.getEvent(&event);
    //float leanAngle = (event.orientation.y, 4);
    //Convert lean angle to String for logging
    //String leanAngleString = String(leanAngle);
    /* Display the floating point data */
    //Serial.print("\tY: ");
    Serial.println(event.orientation.y, 4);// Serial.print("\n");
    //Log the Lean Angle
    logfile.println(event.orientation.y, 4); //logfile.print("\n");
    

    //close file after writing
    logfile.close();
    delay(BNO055_SAMPLERATE_DELAY_MS);
    /*Serial.print("Location (in degrees, works with Google Maps): ");
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(", ");
    Serial.println(GPS.longitudeDegrees, 4);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);*/
  }
  delay(100);
  //}
}
