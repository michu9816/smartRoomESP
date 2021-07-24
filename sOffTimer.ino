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