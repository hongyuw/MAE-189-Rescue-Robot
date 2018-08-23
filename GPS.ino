#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <SoftwareSerial.h>

#include <TinyGPS++.h> // Include the TinyGPS++ library
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
#define GPS_BAUD 9600
#define gpsPort ssGPS
#define SerialMonitor Serial
#define ARDUINO_GPS_RX 9 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 8 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX);

int lati = 34.983322;
int lon = -117.874969;
int NPin = 12;
int SPin = 11;
int WPin = 10;
int EPin = 9;
float Pi = 3.14159;

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");
  pinMode(NPin, OUTPUT);
  pinMode(SPin, OUTPUT);
  pinMode(WPin, OUTPUT);
  pinMode(EPin, OUTPUT);
  gpsPort.begin(GPS_BAUD);
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  printGPSInfo();
  
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  smartDelay(100);
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  
  int destHeading = getHeading(lati, lon);
  if(destHeading-heading<0){
    destHeading = destHeading - heading + 360;
  }
  else{
    destHeading = destHeading - heading;
  }
  Serial.println(destHeading);
  delay(10);
  setPin(destHeading);
}

void printGPSInfo()
{
  // Print latitude, longitude, altitude in feet, course, speed, date, time,
  // and the number of visible satellites.
  SerialMonitor.print("Lat: "); SerialMonitor.println(tinyGPS.location.lat(), 6);
  SerialMonitor.print("Long: "); SerialMonitor.println(tinyGPS.location.lng(), 6);
  SerialMonitor.print("Alt: "); SerialMonitor.println(tinyGPS.altitude.feet());
  SerialMonitor.print("Course: "); SerialMonitor.println(tinyGPS.course.deg());
  SerialMonitor.print("Speed: "); SerialMonitor.println(tinyGPS.speed.mph());
  SerialMonitor.print("Sats: "); SerialMonitor.println(tinyGPS.satellites.value());
  SerialMonitor.println();
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}

int getHeading(int lati, int lon)
{
  int val = atan(lati-tinyGPS.location.lat()/(lon-tinyGPS.location.lng()))*180/Pi;
  if (val < 0)
  {
    val = val+360;
  }
  return val;
}

void setPin(int heading)
{
  if ((0<=heading&&heading<67.5)||(292.5<=heading&&heading<359)){
    digitalWrite(NPin, HIGH);
  }
  else{
    digitalWrite(NPin, LOW);
  }
  if (22.5<=heading&&heading<157.5){
    digitalWrite(EPin, HIGH);
  }
  else{
    digitalWrite(EPin, LOW);
  }
  if (112.5<=heading&&heading<247.5){
    digitalWrite(SPin, HIGH);
  }
  else{
    digitalWrite(SPin, LOW);
  }
  if (202.5<=heading&&heading<337.5){
    digitalWrite(WPin, HIGH);
  }
  else{
    digitalWrite(WPin, LOW);
  }
}
