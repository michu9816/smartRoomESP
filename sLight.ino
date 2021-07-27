void initializeLight(int pin){
    pinMode(pin, OUTPUT);
	digitalWrite(pin,LOW);
}

void turnOnLight(){
    digitalWrite(lightPin,HIGH);
    ledsOn = true;
}

void turnOffLight(){
    digitalWrite(lightPin,LOW);
    ledsOn = false;
}