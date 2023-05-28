#include "dhtSensor.h"
#include <sinric.h>

void initializeDHTSensor()
{
  dht.begin();
}

void handlerDHTSensor()
{
  if (millis() > lastDHTRequest + (10000))
  {
    newHumidity = dht.readHumidity();
    newTemperature = dht.readTemperature();
    lastDHTRequest = millis();
    if ((isnan(newHumidity) || isnan(newTemperature) || (temperature - newTemperature > 4)) && badReadness < 5)
    {
      badReadness++;
      newHumidity = -127;
      newTemperature = -127;
    }
    else
    {
      humidity = newHumidity;
      temperature = (newTemperature + temperatureCorrection);
      if (isnan(humidity))
        humidity = 0;
      if (isnan(temperature))
        temperature = 0;
      badReadness = 0;
    }
    setTemperature(temperature, humidity);
    Serial.print("Reading temperature ");
    Serial.println(newTemperature);
  }
}

float getHumidity()
{
  return humidity;
}

float getTemperature()
{
  return temperature;
}