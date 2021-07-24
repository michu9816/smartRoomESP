
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Hash.h>
#include <ESP8266WebServer.h>
//#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ElegantOTA.h>
#include "Dimmer.h"

// PRZYPISANIE MIEJSC W PAMIĘCI DLA DANYCH

int redEPROM = 1;
int greenEPROM = 2;
int blueEPROM = 3;

int autoEPROM = 4;
int ledsOnEPROM = 5;
int checkLightSensorEPROM = 6;

// DEFINICJE PINÓW

#define LED_RED     15
#define LED_GREEN   13
#define LED_BLUE    12

#define DHTPIN 5     // Digital pin connected to the DHT sensor 

int motionSensorPin = 15;
int lightSensorSensorPin = 4;

// DEFINICJE KOLORÓW

Dimmer red(15);
Dimmer green(13);
Dimmer blue(12);


// USTAWIENIA ODPYTYWANIA

bool useBlynk = false;
bool readTemperature = true; // CZY ODCZYTYWAĆ STAN CZUJNIKA TEMP.
bool checkMotionSensor = false; // CZY ODCZYTYWAĆ STAN CZUJNIKA RUCHU
bool checkLightSensor = false; // CZY ODCZYTYWAĆ STAN CZUJNIKA ŚWIATŁA

// USTAWIENIA WIFI

#ifndef STASSID
#define STASSID "Niknet_001"
#define STAPSK  "Luka2000"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

// USTAWIENIA POBIERANIA CZASU

const long utcOffsetInSeconds = 3600;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


// USTAWIENIA LEDÓW

int lastRed;
int lastGreen;
int lastBlue;

int turnOffDelay = 30;

bool autoMode = false;
bool checkLightSensorInAutoMode = true;
bool ledsOn = false;

int turnOnHour = 6;
int turnOnMinute = 25;
int turnOffHour = 6;
int turnOffMinute = 45;

bool scheduledDays[7] = {false,true,true,true,true,true,false}; // Dni z automatycznym włączaniem światła / nd,pn ...

// POBIERANIE KOLOROW Z CS
bool useCSData = false;

// CZUJNIK RUCHU
bool motion = false;
long lastMotion = 0;

// CZUJNIK ZMIERZCHU
bool lightSensor = false;

// USTAWIENIA CZESTOTLIWOSCI ODPYTYWANIA
long lastRequest = 0;
long lastDHTRequest = 0;
int secDelay = 3;
int secDHTDelay = 20;


// FUNKCJE DLA LEDOW

bool isScheduledDay(){
  Serial.println("Sprawdzam czy dzisiaj jest harmonogramowany / nowy");
  if(scheduledDays[timeClient.getDay()]){
    Serial.println("Ano jest");
    return true;
  }
  return false;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  // BLYNK
  if(useBlynk)
    initializeBlynk();
  
  if(readTemperature){
    initializeSensor();
    readTHSensor();
  }

// PINY LED
  digitalWrite(LED_RED, 0);
  digitalWrite(LED_GREEN, 0);
  digitalWrite(LED_BLUE, 0);


  timeClient.begin();

  red.initialize();
  green.initialize();
  blue.initialize();

  pinMode(motionSensorPin, INPUT); 

  WiFi.mode(WIFI_STA);
  
  WiFiManager wm;

  bool res;

  // STATYCZNE IP
//  wm.setSTAStaticIPConfig(IPAddress(192,168,0,170), IPAddress(192,168,0,1), IPAddress(255,255,255,0)); // KKK
 wm.setSTAStaticIPConfig(IPAddress(192,168,8,41), IPAddress(192,168,8,1), IPAddress(255,255,255,0)); // MMM

  res = wm.autoConnect("NikThinq_LED",""); // password protected ap

  if(!res) {
      Serial.println("Failed to connect");
      // ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
  }

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


// handle index
  webServerInitialize();

  initializeWebsocket();
  EEPROMInitialize();

  loadEPROMValues();
}

void loop() {
  websocketHandler();
  webServerHandler();
  csgoHandle();
  offTimerHandler();
  if(millis() > lastRequest + (secDelay * 1000) && !useCSData){
    timeClient.update();

    if(checkMotionSensor){
      if(digitalRead(motionSensorPin)){ // ODPYTYWANIE CZUJNIKA RUCHU
        motion = true;
        lastMotion = millis();
      }else{
        motion = false;
      }
    }

    if(checkLightSensor){
      if(digitalRead(lightSensorSensorPin)){// ODPYTYWANIE CZUJNIKA ZMIERZCHU
        lightSensor = false;
      }else{
        lightSensor = true;
      }
    }

    // zapalanie ledow wg czujnika ruchu
    if(autoMode && !ledsOn && (checkLightSensorInAutoMode ? !lightSensor : true)){
      if((isScheduledDay() && timeClient.getHours() == turnOnHour && timeClient.getMinutes() == turnOnMinute)){
      // if(motion || (isScheduledDay() && timeClient.getHours() == 18 && timeClient.getMinutes() == 17)){
        Serial.println("Włączono pasek LED");
        fadeIn();
      }

    }else if(autoMode && ledsOn){
      if((isScheduledDay() && timeClient.getHours() == turnOffHour && timeClient.getMinutes() == turnOffMinute)){
      // if((lastMotion != 0 && (lastMotion + turnOffDelay*1000 > millis())) || (isScheduledDay() && timeClient.getHours() == 18 && timeClient.getMinutes() == 18)){
        Serial.println("Wyłączono pasek LED");
        fadeOut();
        lastMotion = 0;
      }
    }else{
      if(lastMotion + turnOffDelay * 1000 > millis())
        lastMotion = 0;
    }

    lastRequest = millis();
    sendStateToClients();
  }
  
  if(millis() > lastDHTRequest + (secDHTDelay * 1000) && readTemperature){
    readTHSensor();
    lastDHTRequest = millis();
    sendStateToClients();
  }
  if(useBlynk)
    blynkHandler();
  // ArduinoOTA.handle();

  red.handler();
  green.handler();
  blue.handler();
}