#line 1 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <Hash.h>
//#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <EEPROM.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ElegantOTA.h>

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

long transitionDuration = 1500;
long scheduledOffLastRequest = 0;

int lastRed;
int lastGreen;
int lastBlue;

float fadeInDuration = 0.7;
float fadeOutDuration = 1;

int turnOffDelay = 30;
int scheduledOffMinutes = 0;

bool autoMode = false;
bool checkLightSensorInAutoMode = true;
bool ledsOn = false;
bool isFadingOut = false;
bool isFadingIn = false;
bool scheduledOff = false;

int turnOnHour = 6;
int turnOnMinute = 25;
int turnOffHour = 6;
int turnOffMinute = 45;

bool scheduledDays[7] = {false,true,true,true,true,true,false}; // Dni z automatycznym włączaniem światła / nd,pn ...


// WEBSOCKET
WebSocketsServer webSocket = WebSocketsServer(8080);

// HTTP
ESP8266WebServer server(80);

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

String state = "";
String oldState = "";

// FUNKCJE DLA LEDOW


#line 124 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void httpFadeIn();
#line 129 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void httpFadeOut();
#line 134 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void fadeOut();
#line 140 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void fadeIn();
#line 146 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void httpLedStatus();
#line 151 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void transitionToColor(int r,int g,int b);
#line 155 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void transitionToColor(int r,int g,int b,bool saveNewColor);
#line 290 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void setColor(int r, int g, int b);
#line 296 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void setColorStronger(int r, int g, int b);
#line 302 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
#line 412 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void generateStateString();
#line 481 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void sendStateToClients();
#line 491 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void textAll(String data);
#line 495 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
bool isScheduledDay();
#line 504 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void checkIncommingRequest();
#line 514 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void setup();
#line 586 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void loop();
#line 659 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void loadEPROMValues();
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
#line 16 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
void initializeSensor();
#line 20 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
float getHumidity();
#line 23 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
float getTemperature();
#line 27 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\sSensors.ino"
void readTHSensor();
#line 124 "c:\\Users\\Misiek\\Desktop\\SRProgram\\SmartRoomProgram_DS_CS\\SmartRoomProgram_DS_CS.ino"
void httpFadeIn(){
  fadeIn();
  server.send(200, "text/plain", "włączono");
}

void httpFadeOut(){
  fadeOut();
  server.send(200, "text/plain", "wyłączono");
}

void fadeOut(){
  transitionDuration = 5000;
  isFadingOut = true;
  transitionToColor(0,0,0);
};

void fadeIn(){
  transitionDuration = 3000;
  isFadingIn = true;
  transitionToColor(lastRed,lastGreen,lastBlue);
};

void httpLedStatus(){
  String res = ledsOn ? "1" : "0";
  server.send(200, "text/plain", res);
}

void transitionToColor(int r,int g,int b){
  transitionToColor(r,g,b,true);
}

void transitionToColor(int r,int g,int b,bool saveNewColor){

  int valuesOnStart[3] = {lastRed,lastBlue,lastGreen};

  int red = r;
  int green = g;
  int blue = b;

  int newRed = lastRed;
  int newGreen = lastGreen;
  int newBlue = lastBlue;

  if(isFadingIn || !ledsOn){
    newRed = 0;
    newGreen = 0;
    newBlue = 0;
  }

  if(r==newRed && g==newGreen && b==newBlue)
    return;

  String redDecision = red < newRed ? "decrease" : "increase";
  String greenDecision = green < newGreen ? "decrease" : "increase";
  String blueDecision = blue < newBlue ? "decrease" : "increase";

  if(isFadingIn){
    redDecision = "increase";
    greenDecision = "increase";
    blueDecision = "increase";
  }else if(isFadingOut){
    redDecision = "decrease";
    greenDecision = "decrease";
    blueDecision = "decrease";
  }

  int blueDiff = blue - newBlue;
  if(blueDiff<0)
    blueDiff = -blueDiff;
  int redDiff = red - newRed;
  if(redDiff<0)
    redDiff = -redDiff;
  int greenDiff = green - newGreen;
  if(greenDiff<0)
    greenDiff = -greenDiff;

  int maxDiff = blueDiff;
  if(redDiff>maxDiff)
    maxDiff = redDiff;  
  if(greenDiff>maxDiff)
    maxDiff = greenDiff;  

  if(maxDiff<1)
    maxDiff = 1;
  if(isFadingIn){
    maxDiff = lastRed ? lastRed : (lastBlue ? lastBlue : lastGreen);
  }
  int transitionDelay = transitionDuration / maxDiff;
  if(!transitionDelay)
    transitionDelay = 1;

  int redStep = 1;
  int greenStep = 1;
  int blueStep = 1;

  bool finishLoop = false;
  while(!finishLoop){

    // CZERWONY
    if(redDecision=="increase"){
      newRed += redStep;
      if(red<newRed)
        newRed = red;
    }else{
      newRed -= redStep;
      if(newRed<red)
        newRed = red;
    }

    // ZIELONY
    if(greenDecision=="increase"){
      newGreen += greenStep;
      if(green<newGreen)
        newGreen = green;
    }else{
      newGreen -= greenStep;
      if(newGreen<green)
        newGreen = green;
    }

    // NIEBIESKI
    if(blueDecision=="increase"){
      newBlue += blueStep;
      if(blue<newBlue)
        newBlue = blue;
    }else{
      newBlue -= blueStep;
      if(newBlue<blue)
        newBlue = blue;
    }

    setColorStronger(newRed,newGreen,newBlue);

    if(red==newRed && blue==newBlue && green==newGreen)
      finishLoop = true;
    else
      delay(transitionDelay);
  }
  
  if(isFadingIn){
    ledsOn = true;
    EEPROM.write(ledsOnEPROM, 1);
  }else if(isFadingOut){
    ledsOn = false;
    EEPROM.write(ledsOnEPROM, 0);
  }

  if(!isFadingOut || !saveNewColor){
    ledsOn = true;
    EEPROM.write(ledsOnEPROM, 1);
    lastRed = newRed;
    lastGreen = newGreen;
    lastBlue = newBlue;
    EEPROM.write(redEPROM,lastRed);
    EEPROM.write(greenEPROM,lastGreen);
    EEPROM.write(blueEPROM,lastBlue);
  }
  EEPROM.commit();

  // USTAWIANIE DOMYŚLNYCH USTAWIEŃ
  isFadingOut = false;
  isFadingIn = false;
  transitionDuration = 750;
};


void setColor(int r, int g, int b){
  analogWrite(LED_RED,r*4);
  analogWrite(LED_GREEN,g*4);
  analogWrite(LED_BLUE,b*4);
}

void setColorStronger(int r, int g, int b){
  analogWrite(LED_RED,r);
  analogWrite(LED_GREEN,g);
  analogWrite(LED_BLUE,b);
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
              generateStateString();
              webSocket.sendTXT(num, state);
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);

            if(payload[0] == '#') {
                // we get RGB data

                // decode rgb data
                uint32_t rgb = (uint32_t) strtol((const char *) &payload[1], NULL, 16);
                // lastRed = ((rgb >> 16) & 0xFF);
                // lastGreen = ((rgb >> 8) & 0xFF);
                // lastBlue = ((rgb >> 0) & 0xFF);

                // EEPROM.write(redEPROM, lastRed);
                // EEPROM.write(greenEPROM, lastGreen);
                // EEPROM.write(blueEPROM, lastBlue);

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
                scheduledOff = scheduleOff;
                scheduledOffMinutes = scheduleMinutes;
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

void generateStateString(){
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
  newState += scheduledOff;
  newState += ",";
  newState += "\"scheduleOffMinutes\":";
  newState += scheduledOffMinutes;
  newState += ",";
  newState += "\"autoMode\":";
  newState += autoMode;
  newState += ",";
  newState += "\"color\":\"";
  newState += lastRed / 4;
  newState += ",";
  newState += lastGreen / 4;
  newState += ",";
  newState += lastBlue / 4;
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

  blynkWriteTH(getTemperature(), getHumidity());
}

void textAll(String data){
  webSocket.broadcastTXT(data);
}

bool isScheduledDay(){
  Serial.println("Sprawdzam czy dzisiaj jest harmonogramowany / nowy");
  if(scheduledDays[timeClient.getDay()]){
    Serial.println("Ano jest");
    return true;
  }
  return false;
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

  timeClient.begin();
  
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, 1);
  digitalWrite(LED_GREEN, 1);
  digitalWrite(LED_BLUE, 1);

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
  server.on("/", checkIncommingRequest);
  server.on("/ledOn", httpFadeIn);
  server.on("/ledOff", httpFadeOut);
  server.on("/ledStatus", httpLedStatus);
  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  // PINY LED
  digitalWrite(LED_RED, 0);
  digitalWrite(LED_GREEN, 0);
  digitalWrite(LED_BLUE, 0);

  EEPROM.begin(10);

  loadEPROMValues();
}

void loop() {
  webSocket.loop();
  server.handleClient();
  csgoHandle();
  if(scheduledOff){
    if(scheduledOffLastRequest==0){
      scheduledOffLastRequest=millis();
    }
    if(millis() > scheduledOffLastRequest + 60000){
      scheduledOffMinutes -= 1;
      scheduledOffLastRequest = millis();
      if(scheduledOffMinutes == 0){
        Serial.println("Wyłączanie ledów");
        fadeOut();
        scheduledOff = false;
        scheduledOffLastRequest = 0;
      }
    }
  }
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
                changeColor(70,70,70);
                Serial.println("Runda LIVE!");
            }else if(roundPhase == "freezetime"){
                mvp = newMvp;
                String playerTeam = postData["player"]["team"];
                if(playerTeam=="CT"){
                    changeColor(6,48,48);
                }else if(playerTeam=="T"){
                    changeColor(48,6,6);
                }
              Serial.println("Oczekiwanie na start rundy");
            }else if(roundPhase == "over"){
                String winningTeam = postData["round"]["win_team"];
                if(newMvp > mvp){
                    changeColor(48,48,0);
                }else if(winningTeam=="CT"){
                    changeColor(0,32,64);
                }else if(winningTeam=="T"){
                    changeColor(48,0,0);
                }
            }
        }
}

void changeColor(int red, int green, int blue){
    transitionToColor(red,green,blue,false);
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
                setColor(75,0,0);
                lastBlinkTime = millis();
            }
        }
    }
}


void loadColorBeforeCS(){
  transitionToColor(savedRed,savedGreen,savedBlue);
}

void saveColorBeforeCS(){
  savedRed = lastRed;
  savedGreen = lastGreen;
  savedBlue = lastBlue;
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
