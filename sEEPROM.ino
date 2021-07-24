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
