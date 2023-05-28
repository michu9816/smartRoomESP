#include "ntp.h"

void initializeNTP(){
    timeClient.begin();
}

String getNTPTime(){
    timeClient.update();

    Serial.println(timeClient.getFormattedTime());
    return timeClient.getFormattedTime();
}

String getNTPDate(){
    time_t epochTime = timeClient.getEpochTime();
    //Get a time structure
    struct tm *ptm = gmtime ((time_t *)&epochTime); 
    
    int monthDay = ptm->tm_mday;  
    int currentMonth = ptm->tm_mon+1;    
    int currentYear = ptm->tm_year+1900;

    //Print complete date:
    String currentDate = String(monthDay) + "." + String(currentMonth) + "." + String(currentYear);
    return currentDate;
}