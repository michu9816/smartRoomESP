#include <WebSocketsServer.h>

extern String state;
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
              generateStateString();
              // sendStateToClients();
              webSocket.sendTXT(num, state);
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
              saveEEPROM();
            }else if(payload[0]=='b'){
              checkLightSensorInAutoMode = !checkLightSensorInAutoMode;
              EEPROM.write(checkLightSensorEPROM, checkLightSensorInAutoMode);
              lastMotion = 0;
              saveEEPROM();
            }else{
              DynamicJsonDocument doc(1024);
              DeserializationError error = deserializeJson(doc, payload);
              if (error)
                return;
              else{
                if(doc.containsKey("scheduled")){
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
                }else if(doc.containsKey("confirmed")){

                  bool scheduleOff = doc["confirmed"];
                  int scheduleMinutes = doc["minutes"];
                  offTimerSet(scheduleOff,scheduleMinutes);
                }else if(doc.containsKey("mode")){
                  String mode = doc["mode"];
                  int speed = doc["speed"];
                  double frequency = doc["frequency"];
                  int brightness = doc["brightness"];
                  // textAll(mode);
                
                  if(mode == "default"){
                    sMode = STABLE;
                    red.fadeIn();
                    green.fadeIn();
                    blue.fadeIn();
                  }else if(mode == "fade"){
                    sMode = FADE;
                    if(doc.containsKey("speed"))
                      fadeDuration = speed;
                    if(doc.containsKey("brightness"))
                      fadeBrightness = brightness;
                  }else if(mode == "strobe"){
                    sMode = STROBE;
                  }else if(mode == "breathing"){
                    sMode = BREATHING;
                    if(doc.containsKey("frequency")){
                      red.setBreathFrequency(frequency);
                      green.setBreathFrequency(frequency);
                      blue.setBreathFrequency(frequency);
                    }
                  }
                }else if(doc.containsKey("transition")){
                  String pay = doc["transition"];
                  uint32_t rgb = (uint32_t) strtol((const char *) &pay[1], NULL, 16);
                  setColorTransition(((rgb >> 16) & 0xFF), ((rgb >> 8) & 0xFF), ((rgb >> 0) & 0xFF));
                }
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
