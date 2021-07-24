#include "Dimmer.h"

extern void sendStateToClients();

int brightnessList[] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 10, 11, 12, 14, 15, 16, 18,
    19, 21, 22, 24, 25, 27, 29, 30,
    32, 34, 35, 37, 39, 41, 43, 44,
    46, 48, 50, 52, 54, 56, 58, 61,
    63, 65, 67, 69, 72, 74, 76, 78,
    81, 83, 86, 88, 91, 93, 96, 98,
    101, 103, 106, 109, 111, 114, 117, 120,
    122, 125, 128, 131, 134, 137, 140, 143,
    146, 149, 152, 155, 158, 161, 164, 168,
    171, 174, 177, 181, 184, 187, 191, 194,
    198, 201, 205, 208, 212, 215, 219, 222,
    226, 230, 233, 237, 241, 244, 248, 252,
    256, 260, 263, 267, 271, 275, 279, 283,
    287, 291, 295, 299, 303, 307, 312, 316,
    320, 324, 328, 333, 337, 341, 345, 350,
    354, 358, 363, 367, 372, 376, 381, 385,
    390, 394, 399, 403, 408, 412, 417, 422,
    426, 431, 436, 440, 445, 450, 455, 459,
    464, 469, 474, 479, 484, 489, 493, 498,
    503, 508, 513, 518, 523, 528, 533, 538,
    543, 548, 554, 559, 564, 569, 574, 579,
    584, 590, 595, 600, 605, 610, 616, 621,
    626, 632, 637, 642, 647, 653, 658, 664,
    669, 674, 680, 685, 690, 696, 701, 707,
    712, 718, 723, 729, 734, 740, 745, 751,
    756, 762, 767, 773, 778, 784, 790, 795,
    801, 806, 812, 818, 823, 829, 834, 840,
    846, 851, 857, 863, 868, 874, 880, 885,
    891, 897, 902, 908, 914, 920, 925, 931,
    937, 942, 948, 954, 960, 965, 971, 977,
    982, 988, 994, 1000, 1005, 1011, 1017, 1023, 1024
};

int Dimmer::devicesNumber = 0;

Dimmer::Dimmer(int outputPin) {
	devicesNumber++;
	this->id = devicesNumber;
	this->outputPin = outputPin;
}

void Dimmer::initialize(){
	// DEFINIOWANIE PINÓW WYJŚCIOWYCH
	pinMode(outputPin, OUTPUT);
	digitalWrite(outputPin,LOW);

	// DEFINIOWANIE PINÓW WEJŚCIOWYCH
	pinMode(inputPin, INPUT_PULLUP);
}


void Dimmer::brightnessTransition(int brightnessRequired){
	brightnessTransition(brightnessRequired,transitionDuration);
}

void Dimmer::brightnessTransition(int brightnessRequired,long transitionDuration){
	this->brightnessRequired = brightnessRequired;
	if(brightnessRequired<0){
		brightnessRequired = 0;
	}else if(brightnessRequired>255){
		brightnessRequired = 255;
	}
	brightnessStep = 1;
	while(brightnessRequired > transitionDuration * brightnessStep){
		brightnessStep++;
	}
	int brightnessDifference = (brightnessRequired - brightness) / brightnessStep;
	if(brightnessDifference > 0){
		transitionDecision = "increase";
	}else if(brightnessDifference < 0){
		transitionDecision = "decrease";
		brightnessDifference = -brightnessDifference;
	}
	if(brightnessDifference>0){
		nextTransitionMillisStep = transitionDuration / brightnessDifference;
	}
	nextTransitionMillis = millis();
}

void Dimmer::brightnessSet(int brightness){
	brightnessSet(brightness,false,true);
}

void Dimmer::brightnessSet(int brightness,bool transition, bool saveLastColor){
	if(brightness<0){
		brightness = 0;
		turnedOn = false;
		// buttonBrightnessDecision = "increase";
	}else if(brightness>255){
		brightness = 255;
	}

	if(brightness==0){
		turnedOn = false;
	}else{
		turnedOn = true;
	}
	if(saveLastColor){
		lastBrightness = brightness;
	}

	this->brightness = brightness;
	if(!transition){
		brightnessRequired = brightness;
	}
	// int calculatedBrightness = brightness * (int)sqrt(brightness) / 4;
	// analogWrite(outputPin,calculatedBrightness);
	analogWrite(outputPin,brightnessList[brightness]);
	Serial.printf("Current brightness: %d \n",brightness);
	lastWebsocketRefresh = millis();
}

void Dimmer::fadeIn(){
	brightnessTransition(lastBrightness);
}

void Dimmer::fadeOut(){
	saveLastColor = false;
	brightnessTransition(0);
}

void Dimmer::handler(){
	if(brightness!=brightnessRequired){
		// Serial.printf("Current brightness: %d \n",brightness);
		// Serial.printf("Required brightness: %d \n",brightnessRequired);
		if(nextTransitionMillis<millis()){
			if(transitionDecision=="increase"){
				brightnessSet(brightness + brightnessStep,true,saveLastColor);
				// Serial.println("Increasing \n");
			}else if(transitionDecision=="decrease"){
				brightnessSet(brightness - brightnessStep,true,saveLastColor);
				// Serial.println("Decreasing \n");
			}
			nextTransitionMillis = millis() + nextTransitionMillisStep;
		}
	}else{
		saveLastColor = true;
	}
	refreshWebsocketState();
	// buttonsHandler();
}

void Dimmer::buttonsHandler(){
	
	bool buttonState = digitalRead(inputPin);
	bool buttonDebounceState = digitalRead(inputPin);
	if(buttonDebounceState != buttonDebounceLastState){
		lastDebounceTime = millis();
		buttonDebounceLastState = buttonDebounceState;
	}
	if(millis() > lastDebounceTime + debounceTime){
		if(buttonState != buttonLastState){
			if(buttonState==LOW){
				buttonPressed = true;
			}else{
				if(buttonPressed && !buttonLongPressed){
					if(turnedOn){
						// brightnessSet(0);
						brightnessTransition(0,100);
						saveLastColor = false;
						buttonBrightnessDecision="increase";
					}else{
						// brightnessSet(lastBrightness);
						saveLastColor = false;
						brightnessTransition(lastBrightness,250);
					}
				}
				if(buttonLongPressed){
					Serial.println("Button pressed");
					if(buttonBrightnessDecision=="increase"){
						buttonBrightnessDecision="decrease";
					}else{
						buttonBrightnessDecision="increase";
					}
				}
				buttonPressed = false;
			}
			buttonLastState = buttonState;
			buttonLastPress = millis();
			buttonLongPressed = false;
		}else{
			if(buttonState==LOW){
				if(buttonPressed && buttonLongPressed && (millis() > buttonLastPress + buttonHoldNextStepDuration)){
					buttonLastPress = millis();
					buttonLongPressed = true;
					if(buttonBrightnessDecision=="increase"){
						brightnessSet(brightness + buttonBrightnessStep);
					}else{
						brightnessSet(brightness - buttonBrightnessStep);
					}
					Serial.println("Holding button");
				}else if(buttonPressed && !buttonLongPressed && (millis() > buttonLastPress + buttonHoldDuration)){
					buttonLastPress = millis();
					buttonLongPressed = true;
					Serial.println("Holding button");
				}
				buttonPressed = true;
			}
		}
	}
}

int Dimmer::getBrightness(){
	return brightness;
}

int Dimmer::getLastBrightness(){
	return lastBrightness;
}

void Dimmer::refreshWebsocketState(){
	if(lastWebsocketRefresh!=0 && (millis() > lastWebsocketRefresh + websocketRefreshDelay)){
		Serial.println("zmieniam stan websocket");
		sendStateToClients();
		lastWebsocketRefresh = 0;
	}
}
