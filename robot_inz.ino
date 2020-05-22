#include <Arduino.h>
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>

#include <MechaQMC5883.h>

MechaQMC5883 compass;

SoftwareSerial GPS_Robo(10, 11 ); // RX, TX
SoftwareSerial SerialBLE(2, 3); // RX, TX
TinyGPSPlus gps2;
char auth[] = "ok3q9uiJKllXv9dtOR7XyewQq0EL-_H6";
WidgetTerminal terminal(V2); // przypisanie terminala do V2

#define trig 13
#define echo 12

// ---------------------------------------- Motors
int MotorP = 7;
int MotorL = 4;
int spdP = 6;
int spdL = 5;
#define MotorP_OFFSET 0
#define MotorL_OFFSET 15


struct Location //-------------------------Structure 
{
  double lat;
  double lon;  /* data */
}Robo,Phone;


BLYNK_WRITE(V0) //----------------------- Phone GPS
{
  GpsParam gps(param);
  /*
  Serial.println("Data from Phone GPS :");
  Serial.println(gps.getLat(), 5);
  Serial.println(gps.getLon(), 5);
  Serial.println();
 */

  Phone.lat = gps.getLat();
  Phone.lon = gps.getLon();
}

bool feed() {
  while (GPS_Robo.available() > 0) {
    if (gps2.encode(GPS_Robo.read()))
      return true;
  }
  return false;
}

BLYNK_WRITE(V1) // ----------------------Button
{
} 

Location GPS_R()  //------------------- Robo GPS 
{  
 bool newData = false;
 unsigned long start = millis();
   while (millis () -  start < 1000) 
  {
  if (feed())
    newData = true;  
  }
    if (newData)
    {
      gps2.location.isUpdated();
        if(gps2.location.lat() != 0.0 && gps2.location.lng())
      { 
         /*    
        Serial.println("Data from Robo GPS :");
        Serial.println(gps2.location.lat(),5);
        Serial.println(gps2.location.lng(),5);
        Serial.println();
        */

        Robo.lat = gps2.location.lat();
        Robo.lon = gps2.location.lng();
        return Robo;
      }
    }  

}


void listen_data() // ------------------ Switch ports
{
  GPS_Robo.listen(); // Robo
  delay (500);
   if (GPS_Robo.available())
    {
      if (GPS_R != NULL)
      {
      GPS_R();
      }
    }
  SerialBLE.listen(); // Phone
  delay(500);
    if (SerialBLE.available() )
    {
      BLYNK_WRITE(V0);  
    }
}
#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float distance(struct Location &Robo, struct Location &Phone) // ------- Calculate Distance
{
  const float R = 6371000; // km
  float p1 = Robo.lat * 0.0174532925199432957;
  float p2 = Phone.lat * 0.0174532925199432957;
  float dp = (Phone.lat - Robo.lat) * 0.0174532925199432957;
  float dl = (Phone.lon - Robo.lon) * 0.0174532925199432957;
  
  float x = sin(dp/2) * sin(dp/2) + 
            cos(p1) * cos(p2) * 
            sin(dl/2) * sin(dl/2);
                     
  float y = 2 * (atan2(sqrt(x), sqrt(1-x)));
  float z =R*y;
  
    if(z < 70) 
    {
      //Serial.println("Distance:");
      //Blynk.virtualWrite(V2,z);
      //Serial.println(R*y);
    return (z);
    }
      else if (z > 70 )
      {
      Serial.println("Waiting for GPS data ...");
      Blynk.virtualWrite(V2,("Waiting for GPS data ..."));
      }
}

float bearing(struct Location &Robo, struct Location &Phone) // -------- Calculate Bearing
{
  float y = sin(Phone.lon-Robo.lon) * cos(Phone.lat);
  float x = cos(Robo.lat)*sin(Phone.lat) - sin(Robo.lat)*cos(Phone.lat)*cos(Phone.lon-Robo.lon);
  float b =  atan2(y, x) * RADTODEG;
  
  //Serial.println("Bearing : ");
  //Serial.println(b);    
  return b;
}


float heading()
{

const float fullCircle = 360.0;
const float halfCircle = fullCircle / 2.0;
const float rad2deg = 180.0 / PI;
const float magdec = 3.0833; // Plano, TX

int xRoll, yPitch, zYaw, heading;
  compass.read(&xRoll, &yPitch, &zYaw);

  heading = atan2((double)yPitch, (double)xRoll);
  heading += magdec;
  
  if (heading > fullCircle)
  {
    heading -= fullCircle;
  }
  if (heading < 0.0)
  {
    heading += fullCircle;
  }
  float headingDeg = heading * rad2deg;
   
  return headingDeg;
  //Serial.print("Heading=");
  //Serial.println(headingDeg);
}
void motors() // ----------------------------Drive functions
{
   digitalWrite(MotorP, HIGH);
   digitalWrite(MotorL, HIGH);  
   analogWrite (spdP, 200);
   analogWrite (spdL, 200);
   delay (200);   
}

void stop_motors()
{
   digitalWrite(MotorP, LOW);
   digitalWrite(MotorL, LOW);  
}
void turn()
{  
   digitalWrite(MotorP, HIGH);  
   analogWrite (spdP, 200);
   delay (100);   
}

void turn_for_ultra()
{  
   digitalWrite(MotorP, HIGH);  
   analogWrite (spdP, 200);
   delay (200);   
   digitalWrite(MotorP, LOW); 
}

float ultrasonar()
{
  float timee; 
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  timee = pulseIn(echo,HIGH);
  if(timee >= 23200) 
  {
  Serial.println("error");
  return 0;
  }
  else if (timee <= 23200){
  float ti =(timee/58.00);
  return ti;
  delay(50);
  }
}
void fallow() // ------------------------ Fallow GPS
{
  float d = distance(Robo,Phone);
  float b = bearing(Robo,Phone);
  float h = heading();
  float t = b - h;
  float s = ultrasonar();

  Serial.println ("Distance:");
  Serial.println(d);
  Serial.println ("bearing:");
  Serial.println(b);
  Serial.println ("Heading:");
  Serial.println(h);
  Serial.println ("Turn:");
  Serial.println(t);
  
if (distance > 3 && ultrasonar >10)
{
  motors(); 
}
else if (distance < 3 && ultrasonar < 10)
{
  stop_motors();
}
if (t > 10 && t < -10)
  {
  turn();
  }
  else if (t < 10 && t > -10)
  {
    stop_motors();
  }
}

void setup()// -----------------------------Setup
{
  // Debug console
  Serial.begin(9600);
  GPS_Robo.begin(9600); // GPS
  SerialBLE.begin(9600); // BLUETOOTH
  Blynk.begin(SerialBLE, auth); // Connection BT
  terminal.clear();
  
  
  pinMode (MotorP,OUTPUT);
  pinMode (MotorL,OUTPUT);
  pinMode (spdP,OUTPUT);
  pinMode (spdL,OUTPUT);

  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);

  Wire.begin();
  compass.init();
       Robo.lat = 0.0; //------------------- Set beginning value 
       Robo.lon = 0.0;
}
void loop()
{
  Blynk.run();
  listen_data();

/*
  Serial.println("dane globalne Phone");
  Serial.println(Phone.lat,15);
  Serial.println(Phone.lon,15);
         // Blynk.virtualWrite(V2,Phone.lat);
         // Blynk.virtualWrite(V2,Phone.lon);

  Serial.println("dane globalne Robo");
  Serial.println(Robo.lat,15);
  Serial.println(Robo.lon,15);
       // Blynk.virtualWrite(V2,Robo.lat);
       // Blynk.virtualWrite(V2,Robo.lon);
 */   
    distance(Robo,Phone);
    Serial.println("");
    heading();
    bearing(Robo,Phone);
    fallow();
}
