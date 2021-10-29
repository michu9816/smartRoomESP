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
bool blinkedRed = true;
String matchRoundPhase;
String playerTeam;

extern bool useCSData;

void csgoHandler(String msg){
    Serial.println("Dostalem informacje z serwera");
        textAll("Informacja z CS");
        Serial.println(msg);
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
            matchRoundPhase = roundPhase;
            String team = postData["player"]["team"];
            playerTeam = team;
            String newMvps = postData["player"]["match_stats"];
            int newMvp = newMvps.toInt();
            if(roundPhase == "freezetime"){
                mvp = newMvp;
                String playerTeam = postData["player"]["team"];
                if(playerTeam=="CT"){
                    setColorTransition(6,48,48);
                }else if(playerTeam=="T"){
                    setColorTransition(48,48,6);
                }
              Serial.println("Oczekiwanie na start rundy");
            }else if(roundPhase == "over"){
                String winningTeam = postData["round"]["win_team"];
                if(newMvp > mvp){
                    setColorTransition(48,48,0);
                }else if(winningTeam=="CT"){
                    setColorTransition(0,32,64);
                }else if(winningTeam=="T"){
                    setColorTransition(64,32,0);
                }
            }
        }
}

void csgoHandle(){
    if(!useCSData){
        return;
    }
    if(lastDataRecieved != 0 && lastDataRecieved + 60000 < millis()){
        loadColorBeforeCS();
        lastDataRecieved = 0;
    }else{
        if(matchRoundPhase=="live"){
            if(health==0){
                setColorTransition(30,30,30);
            }else if(health<20){
                if(blinkedRed && (lastBlinkTime < millis() + 250)){
                    // loadColorBeforeCS();
                    setColor(75,255,0);
                }else if(lastBlinkTime < millis() + 750){
                    setColor(75,0,0);
                    lastBlinkTime = millis();
                }
            }else{
                if(playerTeam=="CT"){
                    setColorTransition(70,200,200);
                }else if(playerTeam=="T"){
                    setColorTransition(200,200,70);
                }
            }
        }
    }
}


void loadColorBeforeCS(){
  setColorTransition(savedRed,savedGreen,savedBlue);
}

void saveColorBeforeCS(){
  savedRed = red.getLastBrightness();
  savedGreen = green.getLastBrightness();
  savedBlue = blue.getLastBrightness();
}