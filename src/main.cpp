#include <Arduino.h>
#include <wifi.h>
#include <ntp.h>
#include <dhtSensor.h>
#include <sinric.h>
#include <rgbDimmer.h>
#include <csgo.h>

Match match;

void setup()
{
	Serial.begin(115200);
	initializeWiFi();
	setupSinricPro();
	initializeNTP();
	initializeDHTSensor();
	initializeRGBDimmer();
}

void loop()
{
	if (!match.usingCSGOData)
	{
		sinricHandler();
		handlerDHTSensor();
	}
	handlerRGBDimmer();
	match.handle();
}