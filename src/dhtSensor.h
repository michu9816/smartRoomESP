#include <DHT.h>

static float humidity = 0.0;
static float newHumidity = 0.0;
static float temperature = 0.0;
static float newTemperature = 0.0;
static float temperatureCorrection = -2;
static int badReadness = 0;
static long lastDHTRequest = 0;

#define DHTTYPE DHT22            // DHT 22  (AM2302)
#define DHTPIN 5                 // Digital pin connected to the DHT sensor
static DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

static uint32_t delayMS;

void initializeDHTSensor();
void handlerDHTSensor();
float getHumidity();
float getTemperature();