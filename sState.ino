String state = "";
String oldState = "";

void refreshlightStatus(){
  if(red.getBrightness() == 0 && green.getBrightness() == 0 && blue.getBrightness() == 0){
    ledsOn = false;
  }else{
    ledsOn = true;
  }
}

void generateStateString(){
refreshlightStatus();
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