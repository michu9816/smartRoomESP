
#include <Arduino.h>
#include <ESP8266WebServer.h>
//#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ElegantOTA.h>
#include "Dimmer.h"

extern void textAll(String name);

enum shineMode
{
  STABLE,
  FADE,
  STROBE,
  BREATHING
};

shineMode sMode = STABLE;
int step = 1;
int fadeBrightness = 100;
long nextFadeTime = 0;
long fadeDuration = 1000;
// PRZYPISANIE MIEJSC W PAMIĘCI DLA DANYCH

int redEPROM = 1;
int greenEPROM = 2;
int blueEPROM = 3;

int autoEPROM = 4;
int ledsOnEPROM = 5;
int checkLightSensorEPROM = 6;

// DEFINICJE PINÓW

#define LED_RED 15
#define LED_GREEN 13
#define LED_BLUE 12

#define DHTPIN 5 // Digital pin connected to the DHT sensor

int motionSensorPin = 15;
int lightSensorSensorPin = 4;

// DEFINICJE KOLORÓW

Dimmer red(15);
Dimmer green(13);
Dimmer blue(12);

// USTAWIENIA ODPYTYWANIA

bool useBlynk = false;
bool readTemperature = true;    // CZY ODCZYTYWAĆ STAN CZUJNIKA TEMP.
bool checkMotionSensor = false; // CZY ODCZYTYWAĆ STAN CZUJNIKA RUCHU
bool checkLightSensor = false;  // CZY ODCZYTYWAĆ STAN CZUJNIKA ŚWIATŁA

// USTAWIENIA WIFI

#ifndef STASSID
#define STASSID "Niknet_001"
#define STAPSK "Luka2000"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

// USTAWIENIA POBIERANIA CZASU

const long utcOffsetInSeconds = 3600;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// USTAWIENIA LEDÓW

int lastRed;
int lastGreen;
int lastBlue;

int lightPin = 4;

int turnOffDelay = 30;

bool autoMode = false;
bool checkLightSensorInAutoMode = false;
bool ledsOn = false;

int turnOnHour = 6;
int turnOnMinute = 25;
int turnOffHour = 6;
int turnOffMinute = 45;

bool scheduledDays[7] = {false, true, true, true, true, true, false}; // Dni z automatycznym włączaniem światła / nd,pn ...

// POBIERANIE KOLOROW Z CS
bool useCSData = false;

// CZUJNIK RUCHU
bool motion = false;
long lastMotion = 0;
long lastTimeUpdate = 0;

// CZUJNIK ZMIERZCHU
bool lightSensor = false;

// USTAWIENIA CZESTOTLIWOSCI ODPYTYWANIA
long lastRequest = 0;
long lastDHTRequest = 0;
int secDelay = 20;
int secDHTDelay = 20;

// FUNKCJE DLA LEDOW

bool isScheduledDay()
{
  Serial.println("Sprawdzam czy dzisiaj jest harmonogramowany / nowy");
  textAll("Sprawdzanie czy dzien harmonogramowy");
  if (scheduledDays[timeClient.getDay()])
  {
    textAll("Otóż tak");
    Serial.println("Ano jest");
    return true;
  }
  else
  {
    textAll("Ni hu hu");
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting");

  // BLYNK
  if (useBlynk)
    initializeBlynk();

  if (readTemperature)
  {
    initializeSensor();
    readTHSensor();
  }

  // PINY LED
  digitalWrite(LED_RED, 0);
  digitalWrite(LED_GREEN, 0);
  digitalWrite(LED_BLUE, 0);

  red.initialize();
  green.initialize();
  blue.initialize();

  pinMode(motionSensorPin, INPUT);

  WiFi.mode(WIFI_STA);

  WiFiManager wm;

  bool res;

  // STATYCZNE IP
  //  wm.setSTAStaticIPConfig(IPAddress(192,168,0,170), IPAddress(192,168,0,1), IPAddress(255,255,255,0)); // KKK
  wm.setSTAStaticIPConfig(IPAddress(192, 168, 8, 41), IPAddress(192, 168, 8, 1), IPAddress(255, 255, 255, 0), IPAddress(8, 8, 8, 8)); // MMM

  res = wm.autoConnect("NikThinq_LED", ""); // password protected ap

  if (!res)
  {
    Serial.println("Failed to connect");
    // ESP.restart();
  }
  else
  {
    // if you get here you have connected to the WiFi
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
  timeClient.begin();
  timeClient.setTimeOffset(3600);
}

void loop()
{
  websocketHandler();
  webServerHandler();
  csgoHandle();
  offTimerHandler();
  EEPROMHandler();

  if (millis() > (lastRequest + (secDelay * 1000)) && !useCSData)
  {

    if (checkMotionSensor)
    {
      if (digitalRead(motionSensorPin))
      { // ODPYTYWANIE CZUJNIKA RUCHU
        motion = true;
        lastMotion = millis();
      }
      else
      {
        motion = false;
      }
    }

    if (checkLightSensor)
    {
      if (digitalRead(lightSensorSensorPin))
      { // ODPYTYWANIE CZUJNIKA ZMIERZCHU
        lightSensor = false;
      }
      else
      {
        lightSensor = true;
      }
    }
    refreshlightStatus();
    // zapalanie ledow wg czujnika ruchu
    if (autoMode && !ledsOn && (checkLightSensorInAutoMode ? !lightSensor : true))
    {
      Serial.println(timeClient.getHours());
      Serial.println(turnOnHour);
      Serial.println(timeClient.getMinutes());
      Serial.println(turnOnMinute);
      if ((isScheduledDay() && timeClient.getHours() == turnOnHour && timeClient.getMinutes() == turnOnMinute))
      {
        // if(motion || (isScheduledDay() && timeClient.getHours() == 18 && timeClient.getMinutes() == 17)){
        Serial.println("Włączono pasek LED");

        textAll("Włączam ledzika");
        red.fadeIn();
        green.fadeIn();
        blue.fadeIn();
      }
    }
    else if (autoMode && ledsOn)
    {
      Serial.println(timeClient.getHours());
      Serial.println(turnOnHour);
      Serial.println(timeClient.getMinutes());
      Serial.println(turnOnMinute);
      if ((isScheduledDay() && timeClient.getHours() == turnOffHour && timeClient.getMinutes() == turnOffMinute))
      {
        // if((lastMotion != 0 && (lastMotion + turnOffDelay*1000 > millis())) || (isScheduledDay() && timeClient.getHours() == 18 && timeClient.getMinutes() == 18)){
        Serial.println("Wyłączono pasek LED");
        red.fadeOut();
        green.fadeOut();
        blue.fadeOut();
        lastMotion = 0;
      }
    }
    else
    {
      if (lastMotion + turnOffDelay * 1000 > millis())
        lastMotion = 0;
    }

    lastRequest = millis();
    sendStateToClients();
  }

  if (millis() > lastTimeUpdate + 30000)
  {
    lastTimeUpdate = millis();
    timeClient.update();
  }

  if (millis() > lastDHTRequest + (secDHTDelay * 1000) && readTemperature)
  {
    readTHSensor();
    lastDHTRequest = millis();
    sendStateToClients();
  }
  if (useBlynk)
    blynkHandler();
  // ArduinoOTA.handle();

  red.handler();
  green.handler();
  blue.handler();

  if (sMode == FADE && ledsOn)
  {
    if (millis() > nextFadeTime + fadeDuration)
    {

      switch (step)
      {
      case 1:
        red.brightnessTransition(fadeBrightness, fadeDuration, true);
        green.brightnessTransition(fadeBrightness, fadeDuration, true);
        blue.brightnessTransition(25, fadeDuration, true);
        step++;
        break;
      case 2:
        red.brightnessTransition(25, fadeDuration, true);
        green.brightnessTransition(fadeBrightness, fadeDuration, true);
        blue.brightnessTransition(25, fadeDuration, true);
        step++;
        break;
      case 3:
        red.brightnessTransition(25, fadeDuration, true);
        green.brightnessTransition(fadeBrightness, fadeDuration, true);
        blue.brightnessTransition(fadeBrightness, fadeDuration, true);
        step++;
        break;
      case 4:
        red.brightnessTransition(25, fadeDuration, true);
        green.brightnessTransition(25, fadeDuration, true);
        blue.brightnessTransition(fadeBrightness, fadeDuration, true);
        step++;
        break;
      case 5:
        red.brightnessTransition(fadeBrightness, fadeDuration, true);
        green.brightnessTransition(25, fadeDuration, true);
        blue.brightnessTransition(fadeBrightness, fadeDuration, true);
        step++;
        break;
      default:
        red.brightnessTransition(fadeBrightness, fadeDuration, true);
        green.brightnessTransition(25, fadeDuration, true);
        blue.brightnessTransition(25, fadeDuration, true);
        step = 1;
        break;
      }
      nextFadeTime = millis();
    }
  }
  else if (sMode == STROBE && ledsOn)
  {
    if (millis() > (nextFadeTime + (step == 1 ? 20 : 60)))
    {

      switch (step)
      {
      case 1:
        red.brightnessSet(red.getLastBrightness(), false, false);
        green.brightnessSet(green.getLastBrightness(), false, false);
        blue.brightnessSet(blue.getLastBrightness(), false, false);
        step++;
        break;
      default:
        red.brightnessSet(0, false, false);
        green.brightnessSet(0, false, false);
        blue.brightnessSet(0, false, false);
        step = 1;
        break;
      }
      nextFadeTime = millis();
    }
  }
  else if (sMode == BREATHING && ledsOn)
  {
    red.breathingHandler();
    green.breathingHandler();
    blue.breathingHandler();
  }
}
