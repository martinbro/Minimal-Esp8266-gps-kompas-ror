
/****************************************************************************************************************************
  ESP8266-Minimal-Client: Minimal ESP8266 Websockets Client

  This sketch:
        1. Connects to a WiFi network
        2. Connects to a Websockets server
           a. Send Calculated val to a Webserver using Websockets
           b. Get Waypoints info& PID values using Websockets
        3. Connects to BNO055
        4. Connects to GPS
        5. Connects to Servo
        6. Connects to Remote Control Device (not implemented)
           Set Rudder Opreration in two MODES MANUAL and AUTO.
        7. Calculate course using datafusion
        8. Calculate position and speed using dead reckoning (not implemented)
        9. Calculate rudder position using PID (not implemented) only in AUTO MODE
        10. Stores Data in External storige (not implementet)


    NOTE:
    The sketch dosen't check or indicate about errors while connecting to
    WiFi or to the websockets server. For full example you might want
    to try the example named "ESP8266-Client".

  Hardware:
        An ESP8266 board.
        GPS
        BNO055
        Servo (rudder)

  Originally Created  : 15/02/2019
  Original Author     : By Gil Maimon
  Original Repository : https://github.com/gilmaimon/ArduinoWebsockets

*****************************************************************************************************************************/
#include "defines.h"
#include <WebSockets2_Generic.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <strings_en.h>
#include <WiFiManager.h>


/*Globale variable************************************************ */
static const int RXPin = 13, TXPin = 12;
static const uint32_t GPSBaud = 9600;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 5;
uint16_t WiFi_DELAY_MS = 100;
static uint32_t t0,t1,tWiFi = 0;


// The TinyGPS++ object /////////////////////////
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
using namespace websockets2_generic;

WebsocketsClient client;
Servo servo;
// BNO055 ///////////////////////////////////////



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
float mx,my,mz = 0.0;
float gx,gy,gz,dt = 0.0;
float rotx,roty,rotz = 0.0;
float ax,ay,az = 0.0;
float dvx,dvy = 0.0;
//float dxRaw,dyRaw,dzRaw = 0.0;
float kurs,kursRaw,roll,rollRaw,pitch,pitchRaw = 0.0;
float K = 0.99;
uint8_t systemC, gyroC, accelC, magC = 0;
float bnoData[10];
float lg[]={-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0} ;
float br[]={-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0} ;
int antalWP =0;
void setup() 
{
  Serial.begin(115200);
  servo.attach(2); //D4
  delay(3000);
  while (!Serial);
  servo.write(45);
  delay(1000);
  servo.write(135);
  delay(1000);
  servo.write(45);
  delay(1000);
  servo.write(135);
  
  //kommunikation mellem GPS og ESP8266
  ss.begin(GPSBaud);

  //etaplering af wifi forbindelsen er flyttet ned i en funktion 
  int ip=-1;
  ip = initConnectToWifi();
  
  // run callback when messages are received
  client.onMessage(onMessageCallback);

  // run callback when events are occuring
  client.onEvent(onEventsCallback);

  // connect to Deno-server
  if(ip==0){ client.connect("192.168.137.1", 8000, "/ws");}//Min laptop Hotspot
  if(ip==1){ client.connect("192.168.87.155", 8000, "/ws");}//Hjemme
  if(ip==2){ client.connect("192.168.43.170", 8000, "/ws");}//HUAWEI
  if(ip==3){ client.connect("192.168.8.220", 8000, "/ws");}//
  // Send a ping
  client.ping("gps");

  initBNO055(bno);
}

int i = 0;
int j = 0;
void loop() 
{  
  getBNO055val();
  //Serial.print(gx);
  
  // Sends information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    if (gps.encode(ss.read())){ 
      i++;
      if (gps.location.isValid()){
        if(i%9==0 ){//Primitiv netværks begrænsning
          sendGPSdata(gps); 
              
  } } } }
  
  client.poll();
}
//**************** FUNKTIONER ************************
//////////////////////Kompas///////////////////////////////
void initBNO055(Adafruit_BNO055 bno){
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1500);
  bno.setExtCrystalUse(true); /* Bruger microprocessorens  clock (jeg bruger en esp88266)- frem for BNO055'ens. Anbefales af alle */ 
}

void getBNO055val(){
  /* Get a new sensor vector*/
  imu::Vector<3> g = bno.getVector( Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> m = bno.getVector( Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> a = bno.getVector( Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
 
  /* Beregner tidsdifferencen dt mellem læsninger - til brug for gyro (jf.: kurs,efter = kurs,før + ROT*dt) */
  t1 = millis(); 
  if(t0==0){t0 = t1;}//initialiserer
  dt =(t1-t0);
  dt =dt/1000.0; /* i sek. */
  t0 = t1;

/* Henter data fra BNO055 */
/* NB! akser fra 'euler'-koordinater til 'fly-koordinater' (dvs. x-akse frem & z-akse NEDAD!) dvs y- og z- akser skifter fortegn) 
 * Årsag: for at få pos ROT om z-akse i samme retn. som kompassets pos. retn)
 * NB! acc er neg da BNO055 måler reaktionskraft.
 */
  rotx = g[0];
  roty = -g[1];
  rotz = -g[2];
  
  ax = -a[0];
  ay = -(-a[1]);
  az = -(-a[2]); 

  mx = m[0];
  my = -m[1];
  mz = -m[2];

  //Beregner gyroens kurs fra Rate Of Turn
  gx = gx + rotx*dt;
  gy = gy + roty*dt;
  gz = gz + rotz*dt;

  // Roll, Pitch og Yaw (kurs) beregnes - bare trigonometri
  float RollRaw = atan2(ay,az); //rollRaw i radianer vinkelrum +-180
  rollRaw = RollRaw*180/PI;
  pitchRaw = atan2(-ax,(ay*sin(RollRaw)+az*cos(RollRaw)))*180/PI; // vinkelrum +-90
  kursRaw = atan2(-my,mx)*180/PI;

  
  //Comperatorfilter på roll, og pitch, 99% gyro, 1% acc
  float k=0.99;//procent gyro
  roll = (roll + rotx*dt)*k + rollRaw*(1-k); 
  pitch = (pitch + roty*dt)*k + pitchRaw*(1-k);

  //'Gyrostabiliserede' værdier
  
  //roll & pitch i radianer
  float Roll = roll*PI/180;
  float Pitch =pitch*PI/180;

  //tilt kompenseret kurs.(Jeg kan vise dig beregningen hvis du er interesseret Jørgen - fås direkte ud fra rotationsmatriserne for roll og pitch)  
  //(Findes også mange steder på nettet! Pas dog på wikipiedia - der har byttet om på roll, pitch og yaw... Hmmm det ligner dem ellers ikke...) 
  // NB! her anvendes de gode! værdier for roll og pitch i radianer!
  float X = mx*cos(Pitch) + mz*sin(Pitch);
  float Y = mx*sin(Roll)*sin(Pitch) + my*cos(Roll) - mz * sin(Roll)*cos(Pitch);
  float kursGyroStabiliseret = (atan2(-Y,X)*180/PI);
  float gyrokurs = kurs +rotz*dt;
  
//Beregner translationen fra accelerometret
  dvx = (ax*cos(Pitch) + az*sin(Pitch))*dt;
  dvy = (ax*sin(Roll)*sin(Pitch) + ay*cos(Roll) - az * sin(Roll)*cos(Pitch))*dt;

  // løser et fjollet diskontinuitetsproblem mellem gyro og mag. (Første del)
  // får mag til at være kontinuært over 180
  if(gyrokurs - kursGyroStabiliseret < -180){
    while (gyrokurs - kursGyroStabiliseret < -180){
      kursGyroStabiliseret = kursGyroStabiliseret -360.0;
    }
  }
  else if(gyrokurs - kursGyroStabiliseret > 180){
    while (gyrokurs - kursGyroStabiliseret > 180){
      kursGyroStabiliseret = kursGyroStabiliseret +360.0;
    }
  }
  kurs = (K*gyrokurs + (1-K)*kursGyroStabiliseret);
  
  //Serial.print(kursGyroStabiliseret); //Serial.print(", ");
  //Serial.print(kurs); //Serial.print(", ");
    //Auto kalibreringens status: (integer) 0=lavest niveau (forkast data), 3=højste niveau (fuldt kalibreret data)
  //uint8_t systemC, gyroC, accelC, magC = 0;
  bno.getCalibration(&systemC, &gyroC, &accelC, &magC);
 
  tWiFi = tWiFi + dt*1000;
  if(tWiFi > WiFi_DELAY_MS){
    tWiFi = 0;
    const int capacity = JSON_OBJECT_SIZE(7+2);
          StaticJsonDocument<capacity> doc;
          doc["name"] = "bno";
          doc["roll"] = roll;
          doc["pitch"] = pitch;
          doc["dt"]= dt;
          doc["kurs"] = kurs;
          doc["rawkurs"] = kursGyroStabiliseret;
          doc["kal"]= gyroC*1000+accelC*100+magC*10+systemC;
          String output = "";
          serializeJson(doc, output);
          client.send( output);
   
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
  //return kurs;
}

////////////////////// GPS ////////////////////////////////
void sendGPSdata(TinyGPSPlus gps){
          const int capacity = JSON_OBJECT_SIZE(10+2);
          StaticJsonDocument<capacity> doc;
          doc["name"] = "navigation";
          doc["lat"] = String(gps.location.lat()*10000000);
          doc["lng"] = String(gps.location.lng()*10000000);
          doc["lolationvalid"]= gps.location.isValid();
          doc["gpscourse"] = gps.course.isValid() ? gps.course.deg() : -1000.0;
          doc["gpsspeed"] = gps.speed.isValid() ? gps.speed.kmph() : -1000.0;
          doc["hdop"]= gps.hdop.hdop();
          doc["sat"] = gps.satellites.value();
          doc["kurs"] = kurs;
          doc["kal"]= gyroC*1000+accelC*100+magC*10+systemC;
          String output = "";
          serializeJson(doc, output);
          client.send( output);
}



////////////////////// websoket2 //////////////////////////
int initConnectToWifi(){
  // ESP8266 Connect to wifi
  delay(5000);
  for (byte j = 0; j < 3; j = i + 1) {
    
    if(j==0){
      WiFi.begin("SKIB1","u530}8T9");
      Serial.println("Prøver at forbinde til LAPTOP-RJIJOOL9 4301");
      }
    if(j==1){
      WiFi.begin("02495500","00809748");
      Serial.println("Prøver at forbinde til privat net");
      }
    if(j==2){
      WiFi.begin("HUAWEI","1q2w3e4r");
      Serial.println("Prøver at forbinde til mobiltelefon");
    }
  

    // Wait some time to connect to wifi
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) 
    {
      Serial.print(".");
      delay(1000);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Succesfuld forbindelse til WiFi");
      return j;
      }
  }
  // Check if connected to wifi
  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.println("No Wifi!");
    return -1;
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  //Serial.print("Connected to Wifi, Connecting to WebSockets Server: ");
  //Serial.println(websockets_server_host);
}
void onMessageCallback(WebsocketsMessage message) 
{
  Serial.print("Got Message: ");
  Serial.println(message.data());
  
  DynamicJsonDocument doc1(1024);
  deserializeJson(doc1, message.data());
  //int nr = doc1["nr"];
  float wp_lg = doc1["lg"];
  //float wp_br = doc1["br"];
  if (wp_lg<0) {
    int i= 0;
    while(i <10){
      lg[i] = -1,0;
      br[i] = -1,0;
      i++;
      antalWP = 0;
    }
  }else{
     lg[antalWP]= doc1["lg"];
     br[antalWP]= doc1["br"];
      antalWP++;
  }
  Serial.print("Tester hvordan den håndterer en ikke kendt værdi: ");
  Serial.println(lg[0]);

  if(doc1["mssg"]<90){
    int udl= doc1["mssg"];
    int udlg = 90 + udl;
    servo.write(udlg);
  }

  String n = doc1["name"];
 // WiFi_DELAY_MS = doc1["rate"];
  WiFi_DELAY_MS = 100;
  K = doc1["k"];
   K=0.99;
 
  
  //Serial.print(n);
 
  //Serial.print(", ");
  i= 0;
  while(i < antalWP ){
   Serial.print(br[i],6);
   Serial.print(", "); 
   Serial.println(lg[i],6); 
   i++; 
  }
  Serial.println();

}

void onEventsCallback(WebsocketsEvent event, String data) 
{
  if (event == WebsocketsEvent::ConnectionOpened) 
  {
    Serial.println("Connnection Opened");
    Serial.println(data);
    
  } 
  else if (event == WebsocketsEvent::ConnectionClosed) 
  {
    Serial.println("Connnection Closed");
  } 
  else if (event == WebsocketsEvent::GotPing) 
  {
    Serial.println("Got a Ping!");
  } 
  else if (event == WebsocketsEvent::GotPong) 
  {
    Serial.println("Got a Pong!");
  }
}
