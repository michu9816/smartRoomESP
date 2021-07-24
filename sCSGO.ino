int savedRed;
int savedGreen;
int savedBlue;

long lastDataRecieved = 0;
long lastBlinkTime = 0;
int health = 100;
int mvp = 0;
int csgoLastRed;
int csgoLastGreen;
int csgoLastBlue;
bool blinkedRed = false;

extern bool useCSData;
extern int lastRed;
extern int lastGreen;
extern int lastBlue;

void csgoHandler(String msg){
    Serial.println("Dostalem informacje z serwera");
        // textAll("Informacja z CS");
        // Serial.println(msg);
        DynamicJsonDocument postData(2048);
        DeserializationError error = deserializeJson(postData, msg);
        textAll(msg);
        // textAll(error);
        if (error)
            return;
        else{
            if(lastDataRecieved==0){
                saveColorBeforeCS();
            }
            lastDataRecieved = millis();
            health = postData["player"]["state"]["health"];
            // textAll("Dane z cs");
            String roundPhase = postData["round"]["phase"];
            
            String newMvps = postData["player"]["match_stats"];
            int newMvp = newMvps.toInt();
            if(roundPhase == "live"){
                changeColor(70,70,70);
                Serial.println("Runda LIVE!");
            }else if(roundPhase == "freezetime"){
                mvp = newMvp;
                String playerTeam = postData["player"]["team"];
                if(playerTeam=="CT"){
                    changeColor(6,48,48);
                }else if(playerTeam=="T"){
                    changeColor(48,6,6);
                }
              Serial.println("Oczekiwanie na start rundy");
            }else if(roundPhase == "over"){
                String winningTeam = postData["round"]["win_team"];
                if(newMvp > mvp){
                    changeColor(48,48,0);
                }else if(winningTeam=="CT"){
                    changeColor(0,32,64);
                }else if(winningTeam=="T"){
                    changeColor(48,0,0);
                }
            }
        }
}

void changeColor(int red, int green, int blue){
    transitionToColor(red,green,blue,false);
    csgoLastBlue = blue;
    csgoLastGreen = green;
    csgoLastRed = red;
}

void csgoHandle(){
    if(!useCSData){
        return;
    }
    if(lastDataRecieved != 0 && lastDataRecieved + 60000 < millis()){
        loadColorBeforeCS();
        lastDataRecieved = 0;
    }else{
        if(health<20){
            if(blinkedRed && (lastBlinkTime > millis() + 250)){
                loadColorBeforeCS();
            }else if(lastBlinkTime > millis() + 750){
                setColor(75,0,0);
                lastBlinkTime = millis();
            }
        }
    }
}


void loadColorBeforeCS(){
  transitionToColor(savedRed,savedGreen,savedBlue);
}

void saveColorBeforeCS(){
  savedRed = lastRed;
  savedGreen = lastGreen;
  savedBlue = lastBlue;
}