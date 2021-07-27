#include <EEPROM.h>
long saveEEPROMTime = 0;

void saveEEPROM(){
  saveEEPROMTime = millis();
}

void EEPROMInitialize(){
    EEPROM.begin(10);
}

void loadEPROMValues(){
  lastRed = EEPROM.read(redEPROM);
  lastGreen = EEPROM.read(greenEPROM);
  lastBlue = EEPROM.read(blueEPROM);
  
  autoMode = EEPROM.read(autoEPROM);
  // checkLightSensorInAutoMode = EEPROM.read(checkLightSensorEPROM);
  
  // if(EEPROM.read(ledsOnEPROM))
  //   fadeIn();
  // else
  //   fadeOut();
}

void EEPROMHandler(){
  if(saveEEPROMTime + 50000 > millis() && saveEEPROMTime!=0){
    saveEEPROMTime = 0;
    Serial.println("Zapisane do pamięci EEPROM");
    EEPROM.commit();
  }
}