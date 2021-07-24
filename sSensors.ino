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