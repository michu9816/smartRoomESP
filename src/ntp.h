#include <NTPClient.h>
#include <WiFiUdp.h>

static WiFiUDP ntpUDP;
static NTPClient timeClient(ntpUDP, 7200);

void initializeNTP();
String getNTPTime();
String getNTPDate();