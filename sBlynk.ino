#include <BlynkSimpleEsp8266.h>
#define BLYNK_PRINT Serial 

// BLYNK
char auth[] = "azQAhlcdow65Zo_D_X28mYmtX1scdmZ9"; //Enter the Auth code which was send by Blink

void initializeBlynk(){
    Blynk.begin(auth, ssid, password);
}

void blynkHandler(){
    Blynk.run(); // Initiates Blynk
}

void blynkWriteTH(float humidity, float temperature){
    Blynk.virtualWrite(V5, humidity);  //V5 is for Humidity
    Blynk.virtualWrite(V6, temperature);  //V6 is for Temperature
}