#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#include "SinricPro.h"
#include "SinricProLight.h"
#include "SinricProTemperaturesensor.h"

#define APP_KEY "7f3c4343-9d39-4c52-8f7e-9c7b4554eac7"                                         // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET "3b21a884-90e2-4031-8b8b-ed15183584d6-0b81378e-f179-4fd2-9868-45d0751ed83b" // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define LIGHT_ID "631c40a036b44d06d4b7ce20"                                                    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define TEMP_SENSOR_ID "63653643b8a7fefbd6317956"

// define array of supported color temperatures
int colorTemperatureArray[] = {2200, 2700, 4000, 5500, 7000};
int max_color_temperatures = sizeof(colorTemperatureArray) / sizeof(colorTemperatureArray[0]); // calculates how many elements are stored in colorTemperature array

// a map used to convert a given color temperature into color temperature index (used for colorTemperatureArray)
std::map<int, int> colorTemperatureIndex;

// initializes the map above with color temperatures and index values
// so that the map can be used to do a reverse search like
// int index = colorTemperateIndex[4000]; <- will result in index == 2
void setupColorTemperatureIndex()
{
    Serial.printf("Setup color temperature lookup table\r\n");
    for (int i = 0; i < max_color_temperatures; i++)
    {
        colorTemperatureIndex[colorTemperatureArray[i]] = i;
        Serial.printf("colorTemperatureIndex[%i] = %i\r\n", colorTemperatureArray[i], colorTemperatureIndex[colorTemperatureArray[i]]);
    }
}

// we use a struct to store all states and values for our light
struct
{
    bool powerState = ledsOn;
    int brightness = 0;
    struct
    {
        byte r = 0;
        byte g = 0;
        byte b = 0;
    } color;
    int colorTemperature = colorTemperatureArray[0]; // set colorTemperature to first element in colorTemperatureArray array
} device_state;

bool onPowerState(const String &deviceId, bool &state)
{
    Serial.printf("Device %s power turned %s \r\n", deviceId.c_str(), state ? "on" : "off");
    if (state)
    {
        fadeIn();
    }
    else
    {
        fadeOut();
    }
    // device_state.powerState = state;
    return true; // request handled properly
}

bool onBrightness(const String &deviceId, int &brightness)
{
    device_state.brightness = brightness;
    red.brightnessTransition(brightness, 500, false);
    green.brightnessTransition(brightness, 500, false);
    blue.brightnessTransition(brightness, 500, false);
    Serial.printf("Device %s brightness level changed to %d\r\n", deviceId.c_str(), brightness);
    return true;
}

bool onAdjustBrightness(const String &deviceId, int brightnessDelta)
{
    device_state.brightness += brightnessDelta;
    Serial.printf("Device %s brightness level changed about %i to %d\r\n", deviceId.c_str(), brightnessDelta, device_state.brightness);
    brightnessDelta = device_state.brightness;
    return true;
}

bool onColor(const String &deviceId, byte &r, byte &g, byte &b)
{
    red.brightnessTransition(r, 500, false);
    green.brightnessTransition(g, 500, false);
    blue.brightnessTransition(b, 500, false);
    device_state.color.r = r;
    device_state.color.g = g;
    device_state.color.b = b;
    Serial.printf("Device %s color changed to %d, %d, %d (RGB)\r\n", deviceId.c_str(), device_state.color.r, device_state.color.g, device_state.color.b);
    return true;
}

bool onColorTemperature(const String &deviceId, int &colorTemperature)
{
    device_state.colorTemperature = colorTemperature;
    Serial.printf("Device %s color temperature changed to %d\r\n", deviceId.c_str(), device_state.colorTemperature);
    return true;
}

bool onIncreaseColorTemperature(const String &deviceId, int &colorTemperature)
{
    int index = colorTemperatureIndex[device_state.colorTemperature]; // get index of stored colorTemperature
    index++;                                                          // do the increase
    if (index < 0)
        index = 0; // make sure that index stays within array boundaries
    if (index > max_color_temperatures - 1)
        index = max_color_temperatures - 1;                       // make sure that index stays within array boundaries
    device_state.colorTemperature = colorTemperatureArray[index]; // get the color temperature value
    Serial.printf("Device %s increased color temperature to %d\r\n", deviceId.c_str(), device_state.colorTemperature);
    colorTemperature = device_state.colorTemperature; // return current color temperature value
    return true;
}

bool onDecreaseColorTemperature(const String &deviceId, int &colorTemperature)
{
    int index = colorTemperatureIndex[device_state.colorTemperature]; // get index of stored colorTemperature
    index--;                                                          // do the decrease
    if (index < 0)
        index = 0; // make sure that index stays within array boundaries
    if (index > max_color_temperatures - 1)
        index = max_color_temperatures - 1;                       // make sure that index stays within array boundaries
    device_state.colorTemperature = colorTemperatureArray[index]; // get the color temperature value
    Serial.printf("Device %s decreased color temperature to %d\r\n", deviceId.c_str(), device_state.colorTemperature);
    colorTemperature = device_state.colorTemperature; // return current color temperature value
    return true;
}

void setTemperature(float temperature, float humidity)
{
    SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];    // get temperaturesensor device
    bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event
    if (success)
    { // if event was sent successfuly, print temperature and humidity to serial
        Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
    }
    else
    { // if sending event failed, print error message
        Serial.printf("Something went wrong...could not send Event to server!\r\n");
    }
}

void setupSinricPro()
{
    // get a new Light device from SinricPro
    SinricProLight &myLight = SinricPro[LIGHT_ID];

    // set callback function to device
    myLight.onPowerState(onPowerState);
    myLight.onBrightness(onBrightness);
    myLight.onAdjustBrightness(onAdjustBrightness);
    myLight.onColor(onColor);
    myLight.onColorTemperature(onColorTemperature);
    myLight.onIncreaseColorTemperature(onIncreaseColorTemperature);
    myLight.onDecreaseColorTemperature(onDecreaseColorTemperature);

    // setup SinricPro
    SinricPro.onConnected([]()
                          { Serial.printf("Connected to SinricPro\r\n"); });
    SinricPro.onDisconnected([]()
                             { Serial.printf("Disconnected from SinricPro\r\n"); });
    SinricPro.begin(APP_KEY, APP_SECRET);
}

// main setup function

void sinricHandler()
{
    SinricPro.handle();
}