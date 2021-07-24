#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"

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
int secDelay = 15;
int secDHTDelay = 20;


// FUNKCJE DLA LEDOW

#line 109 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
bool isScheduledDay();
#line 119 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void setup();
#line 181 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void loop();
#line 7 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sBlynk.ino"
void initializeBlynk();
#line 11 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sBlynk.ino"
void blynkHandler();
#line 15 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sBlynk.ino"
void blynkWriteTH(float humidity, float temperature);
#line 19 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sCSGO.ino"
void csgoHandler(String msg);
#line 65 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sCSGO.ino"
void changeColor(int red, int green, int blue);
#line 72 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sCSGO.ino"
void csgoHandle();
#line 92 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sCSGO.ino"
void loadColorBeforeCS();
#line 96 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sCSGO.ino"
void saveColorBeforeCS();
#line 3 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sEEPROM.ino"
void EEPROMInitialize();
#line 7 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sEEPROM.ino"
void loadEPROMValues();
#line 3 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sLed.ino"
void httpFadeIn();
#line 8 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sLed.ino"
void httpFadeOut();
#line 13 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sLed.ino"
void fadeOut();
#line 19 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sLed.ino"
void fadeIn();
#line 25 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sLed.ino"
void setColor(int redColor, int greenColor, int blueColor);
#line 5 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sOffTimer.ino"
bool getOffTimerStatus();
#line 9 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sOffTimer.ino"
int getOffTimerMinutes();
#line 13 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sOffTimer.ino"
void offTimerSet(bool scheduleOff, int scheduleMinutes);
#line 18 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sOffTimer.ino"
void offTimerHandler();
#line 16 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
void initializeSensor();
#line 20 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
float getHumidity();
#line 23 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
float getTemperature();
#line 27 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
void readTHSensor();
#line 6 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sState.ino"
void generateStateString();
#line 83 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sState.ino"
void sendStateToClients();
#line 94 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sState.ino"
void refreshState();
#line 98 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sState.ino"
void stateHandler();
#line 5 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebServer.ino"
void httpLedStatus();
#line 11 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebServer.ino"
void checkIncommingRequest();
#line 21 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebServer.ino"
void webServerInitialize();
#line 30 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebServer.ino"
void webServerHandler();
#line 6 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebSocket.ino"
void initializeWebsocket();
#line 11 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebSocket.ino"
void websocketHandler();
#line 16 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebSocket.ino"
void textAll(String data);
#line 20 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebSocket.ino"
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
#line 109 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
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
    refreshState();
  }
  
  if(millis() > lastDHTRequest + (secDHTDelay * 1000) && readTemperature){
    readTHSensor();
    lastDHTRequest = millis();
    refreshState();
  }
  if(useBlynk)
    blynkHandler();
  // ArduinoOTA.handle();

  red.handler();
  green.handler();
  blue.handler();
}
#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sBlynk.ino"
#include <BlynkSimpleEsp8266.h>
#define BLYNK_PRINT Serial 

// BLYNK
char auth[] = "azQAhlcdow65Zo_D_X28mYmtX1scdmZ9"; //Enter the Auth code which was send by Blink

void initializeBlynk(){
    Blynk.begin(auth, ssid, password);
}

void blynkHandler(){
    Blynk.run(); // Initiates Blynk
}

void blynkWriteTH(float humidity, float temperature){
    Blynk.virtualWrite(V5, humidity);  //V5 is for Humidity
    Blynk.virtualWrite(V6, temperature);  //V6 is for Temperature
}
#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sCSGO.ino"
int savedRed;
int savedGreen;
int savedBlue;

long lastDataRecieved = 0;
long lastBlinkTime = 0;
int health = 100;
int mvp = 0;
int csgoLastRed;
int csgoLastGreen;
int csgoLastBlue;
bool blinkedRed = false;

extern bool useCSData;
extern int lastRed;
extern int lastGreen;
extern int lastBlue;

void csgoHandler(String msg){
    Serial.println("Dostalem informacje z serwera");
        // textAll("Informacja z CS");
        // Serial.println(msg);
        DynamicJsonDocument postData(2048);
        DeserializationError error = deserializeJson(postData, msg);
        textAll(msg);
        // textAll(error);
        if (error)
            return;
        else{
            if(lastDataRecieved==0){
                saveColorBeforeCS();
            }
            lastDataRecieved = millis();
            health = postData["player"]["state"]["health"];
            // textAll("Dane z cs");
            String roundPhase = postData["round"]["phase"];
            
            String newMvps = postData["player"]["match_stats"];
            int newMvp = newMvps.toInt();
            if(roundPhase == "live"){
                // changeColor(70,70,70);
                Serial.println("Runda LIVE!");
            }else if(roundPhase == "freezetime"){
                mvp = newMvp;
                String playerTeam = postData["player"]["team"];
                if(playerTeam=="CT"){
                    // changeColor(6,48,48);
                }else if(playerTeam=="T"){
                    // changeColor(48,6,6);
                }
              Serial.println("Oczekiwanie na start rundy");
            }else if(roundPhase == "over"){
                String winningTeam = postData["round"]["win_team"];
                if(newMvp > mvp){
                    // changeColor(48,48,0);
                }else if(winningTeam=="CT"){
                    // changeColor(0,32,64);
                }else if(winningTeam=="T"){
                    // changeColor(48,0,0);
                }
            }
        }
}

void changeColor(int red, int green, int blue){
    // transitionToColor(red,green,blue,false);
    csgoLastBlue = blue;
    csgoLastGreen = green;
    csgoLastRed = red;
}

void csgoHandle(){
    if(!useCSData){
        return;
    }
    if(lastDataRecieved != 0 && lastDataRecieved + 60000 < millis()){
        loadColorBeforeCS();
        lastDataRecieved = 0;
    }else{
        if(health<20){
            if(blinkedRed && (lastBlinkTime > millis() + 250)){
                loadColorBeforeCS();
            }else if(lastBlinkTime > millis() + 750){
                // setColor(75,0,0);
                lastBlinkTime = millis();
            }
        }
    }
}


void loadColorBeforeCS(){
//   transitionToColor(savedRed,savedGreen,savedBlue);
}

void saveColorBeforeCS(){
  savedRed = lastRed;
  savedGreen = lastGreen;
  savedBlue = lastBlue;
}
#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sEEPROM.ino"
#include <EEPROM.h>

void EEPROMInitialize(){
    EEPROM.begin(10);
}

void loadEPROMValues(){
  lastRed = EEPROM.read(redEPROM);
  lastGreen = EEPROM.read(greenEPROM);
  lastBlue = EEPROM.read(blueEPROM);
  
  autoMode = EEPROM.read(autoEPROM);
  checkLightSensorInAutoMode = EEPROM.read(checkLightSensorEPROM);
  
  if(EEPROM.read(ledsOnEPROM))
    fadeIn();
  else
    fadeOut();
}

#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sLed.ino"
extern ESP8266WebServer server;

void httpFadeIn(){
  fadeIn();
  server.send(200, "text/plain", "włączono");
}

void httpFadeOut(){
  fadeOut();
  server.send(200, "text/plain", "wyłączono");
}

void fadeOut(){
  red.fadeOut();
  green.fadeOut();
  blue.fadeOut();
};

void fadeIn(){
  red.fadeIn();
  green.fadeIn();
  blue.fadeIn();
};

void setColor(int redColor, int greenColor, int blueColor){
  red.brightnessSet(redColor);
  green.brightnessSet(greenColor);
  blue.brightnessSet(blueColor);
};

#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sOffTimer.ino"
long offTimerLastRequest = 0;
int offTimerMinutes = 0;
bool offTimer = false;

bool getOffTimerStatus(){
	return offTimer;
}

int getOffTimerMinutes(){
	return offTimerMinutes;
}

void offTimerSet(bool scheduleOff, int scheduleMinutes){
	offTimer = scheduleOff;
    offTimerMinutes = scheduleMinutes;
}

void offTimerHandler(){
	if(offTimer){
		if(offTimerLastRequest==0){
			offTimerLastRequest=millis();
		}
		if(millis() > offTimerLastRequest + 60000){
			offTimerMinutes -= 1;
			offTimerLastRequest = millis();
			if(offTimerMinutes == 0){
				Serial.println("Wyłączanie ledów");
				fadeOut();
				offTimer = false;
				offTimerLastRequest = 0;
			}
		}
	}
}
#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
#include <DHT.h>

// CZUJNIK TEMPERATURY I WILGOTNOŚCI
float humidity = 0.0;
float newHumidity = 0.0;
float temperature = 0.0;
float newTemperature = 0.0;
float temperatureCorrection = -2;
int badReadness = 0;

#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

uint32_t delayMS;

void initializeSensor(){
    dht.begin();
}

float getHumidity(){
    return humidity;
}
float getTemperature(){
    return temperature;
}

void readTHSensor(){
    newHumidity = dht.readHumidity();
    newTemperature= dht.readTemperature();
    if((isnan(newHumidity) || isnan(newTemperature) || (temperature - newTemperature > 4)) && badReadness < 5){
      badReadness++;
      newHumidity = -127;
      newTemperature = -127;
    }else{
      humidity = newHumidity;
      temperature = (newTemperature + temperatureCorrection);
      if(isnan(humidity))
        humidity = 0;
      if(isnan(temperature))
        temperature = 0;
      badReadness = 0;
    }
}
#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sState.ino"
String state = "";
String oldState = "";

long lastStateGenerated = 0;

void generateStateString(){
    bool ledsOn;
  if(red.getBrightness() == 0 && green.getBrightness() == 0 && blue.getBrightness() == 0){
    ledsOn = false;
  }else{
    ledsOn = true;
  }

  String newState = "";
  newState += "{\"dhtSensor\":{\"humidity\":";
  newState += getHumidity();
  newState += ",\"temperature\":";
  newState += getTemperature();
  newState += "},";
  newState += "\"motionSensor\":";
  newState += motion;
  newState += ",";
  newState += "\"lightSensor\":";
  newState += lightSensor;
  newState += ",";
  newState += "\"ledsOn\":";
  newState += ledsOn;
  newState += ",";
  newState += "\"autoMode\":";
  newState += autoMode;
  newState += ",";
  newState += "\"light\":{";
  newState += "\"turnOn\":";
  newState += ledsOn;
  newState += ",";
  newState += "\"usingCSData\":";
  newState += useCSData;
  newState += ",";
  newState += "\"checkLightAuto\":";
  newState += checkLightSensorInAutoMode;
  newState += ",";
  newState += "\"turnOnTime\":\"";
  newState += turnOnHour;
  newState += ":";
  newState += turnOnMinute;
  newState += "\",";
  newState += "\"turnOffTime\":\"";
  newState += turnOffHour;
  newState += ":";
  newState += turnOffMinute;
  newState += "\",";
  newState += "\"scheduleOffConfirmed\":";
  newState += getOffTimerStatus();
  newState += ",";
  newState += "\"scheduleOffMinutes\":";
  newState += getOffTimerMinutes();
  newState += ",";
  newState += "\"autoMode\":";
  newState += autoMode;
  newState += ",";
  newState += "\"color\":\"";
  newState += red.getLastBrightness();
  newState += ",";
  newState += green.getLastBrightness();
  newState += ",";
  newState += blue.getLastBrightness();
  newState += "\"";
  newState += ",";
  newState += "\"scheduledDays\":[";
  for(int i = 0;i<7;i++){
    newState += scheduledDays[i];
    if(i!=6)
      newState += ",";
  }
  newState += "]";
  newState += "}";
  newState += "}";

  state = newState;
}


void sendStateToClients(){
  generateStateString();
  if(!oldState.equals(state)){
    textAll(state);
    oldState = state;
  }

  if(useBlynk)
    blynkWriteTH(getTemperature(), getHumidity());
}

void refreshState(){
  lastStateGenerated = millis();
}

void stateHandler(){
  if(lastStateGenerated != 0 && lastStateGenerated + 1000 > millis()){
    lastStateGenerated = 0;
    sendStateToClients();
  }
}
#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebServer.ino"
// HTTP
ESP8266WebServer server(80);


void httpLedStatus(){
  String res = ledsOn ? "1" : "0";
  server.send(200, "text/plain", res);
}


void checkIncommingRequest(){
    if(server.method() != HTTP_POST){
        server.send(200, "text/plain", "Spoczko");
    }else{
      if(useCSData){
        csgoHandler(server.arg("plain"));
      }
    }
}

void webServerInitialize(){
    server.on("/", checkIncommingRequest);
    server.on("/ledOn", httpFadeIn);
    server.on("/ledOff", httpFadeOut);
    server.on("/ledStatus", httpLedStatus);
    server.begin();
    ElegantOTA.begin(&server);    // Start ElegantOTA
}

void webServerHandler(){
    server.handleClient();
}
#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sWebSocket.ino"
#include <WebSocketsServer.h>

// WEBSOCKET
WebSocketsServer webSocket = WebSocketsServer(8080);

void initializeWebsocket(){
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void websocketHandler(){
    webSocket.loop();
}


void textAll(String data){
  webSocket.broadcastTXT(data);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        
              // send message to client
              refreshState();
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);

            if(payload[0] == '#') {
                // we get RGB data

                // decode rgb data
                uint32_t rgb = (uint32_t) strtol((const char *) &payload[1], NULL, 16);

                Serial.print(lastRed);
                Serial.print(",");
                Serial.print(lastGreen);
                Serial.print(",");
                Serial.println(lastBlue);
                if(lastRed==0 && lastBlue == 0 && lastGreen == 0){
                  ledsOn = false;
                  lastRed = 30;
                  lastGreen = 30;
                  lastBlue = 30;

                  EEPROM.write(redEPROM, lastRed);
                  EEPROM.write(greenEPROM, lastGreen);
                  EEPROM.write(blueEPROM, lastBlue);
                }
                // transitionToColor(((rgb >> 16) & 0xFF) * 4, ((rgb >> 8) & 0xFF) * 4, ((rgb >> 0) & 0xFF) * 4);
                setColor(((rgb >> 16) & 0xFF), ((rgb >> 8) & 0xFF), ((rgb >> 0) & 0xFF));
            }else if(payload[0]=='i'){
              fadeIn();
            }else if(payload[0]=='o'){
              fadeOut();
            }else if(payload[0]=='x'){
              digitalWrite(LED_RED,0);
              digitalWrite(LED_GREEN,0);
              digitalWrite(LED_BLUE,0);
            }else if(payload[0]=='y'){
              analogWrite(LED_RED,0);
              analogWrite(LED_GREEN,0);
              analogWrite(LED_BLUE,0);
            }else if(payload[0]=='c'){
              useCSData = !useCSData;
            }else if(payload[0]=='a'){
              autoMode = !autoMode;
              EEPROM.write(autoEPROM, autoMode);
              lastMotion = 0;
              EEPROM.commit();
            }else if(payload[0]=='b'){
              checkLightSensorInAutoMode = !checkLightSensorInAutoMode;
              EEPROM.write(checkLightSensorEPROM, checkLightSensorInAutoMode);
              lastMotion = 0;
              EEPROM.commit();
            }else{
              DynamicJsonDocument doc(1024);
              DeserializationError error = deserializeJson(doc, payload);
              if (error)
                return;
              else{
                int newScheduledDays[7] = {doc["scheduled"][0],doc["scheduled"][1],doc["scheduled"][2],doc["scheduled"][3],doc["scheduled"][4],doc["scheduled"][5],doc["scheduled"][6]};
                int newTurnOnHour = doc["turnOnHour"];
                int newTurnOnMinute = doc["turnOnMinute"];
                int newTurnOffHour = doc["turnOffHour"];
                int newTurnOffMinute = doc["turnOffMinute"];
                turnOnHour = newTurnOnHour;
                turnOnMinute = newTurnOnMinute;
                turnOffHour = newTurnOffHour;
                turnOffMinute = newTurnOffMinute;
                for(int i = 0;i<7;i++){
                  scheduledDays[i] = newScheduledDays[i];
                }

                bool scheduleOff = doc["confirmed"];
                int scheduleMinutes = doc["minutes"];
                offTimerSet(scheduleOff,scheduleMinutes);
              }
            }
            break;
        case WStype_BIN:
            Serial.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);

            // send message to client
            // webSocket.sendBIN(num, payload, length);
            break;
    }

}

