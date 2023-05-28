#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#define APP_KEY "7f3c4343-9d39-4c52-8f7e-9c7b4554eac7"                                         // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET "3b21a884-90e2-4031-8b8b-ed15183584d6-0b81378e-f179-4fd2-9868-45d0751ed83b" // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define LIGHT_ID "631c40a036b44d06d4b7ce20"                                                    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define TEMP_SENSOR_ID "63653643b8a7fefbd6317956"

void setupColorTemperatureIndex();
bool onPowerState(const String &, bool &);
bool onBrightness(const String &, int &);
bool onAdjustBrightness(const String &, int);
bool onColor(const String &, byte &, byte &, byte &);
bool onColorTemperature(const String &, int &);
bool onIncreaseColorTemperature(const String &, int &);
bool onDecreaseColorTemperature(const String &, int &);
void setTemperature(float, float);
void setupSinricPro();
void sinricHandler();