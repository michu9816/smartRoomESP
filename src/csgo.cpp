#include <csgo.h>;
#include <ArduinoJson.h>;
#include <rgbDimmer.h>;
Match::Match()
{
    health = 0;
}

void Match::setHealth(int health)
{
    Serial.print("Changed health to ");
    Serial.println(health);
    if (this->health != health)
    {
        this->health = health;
        this->refreshState = true;
    }
}

void Match::setKills(int kills)
{
    Serial.print("Changed kills to ");
    Serial.println(kills);
    if (this->kills != kills)
    {
        this->kills = kills;
        this->refreshState = true;
    }
}
void Match::setFlash(int flash)
{
    Serial.print("Changed flash to ");
    Serial.println(flash);
    if (this->flash != flash)
    {
        this->flash = flash;
        this->refreshState = true;
    }
}
void Match::setSmoke(int smoke)
{
    Serial.print("Changed kills to ");
    Serial.println(smoke);
    if (this->smoke != smoke)
    {
        this->smoke = smoke;
        this->refreshState = true;
    }
}
void Match::setBurn(int burn)
{
    Serial.print("Changed burn to ");
    Serial.println(burn);
    if (this->burn != burn)
    {
        this->burn = burn;
        this->refreshState = true;
    }
}

void Match::setTeam(String team)
{
    Serial.print("Changed team to ");
    Serial.println(team);
    if (this->team != team)
    {
        this->team = team;
        this->refreshState = true;
    }
}
void Match::setPhase(String phase)
{
    Serial.print("Changed phase to ");
    Serial.println(phase);
    if (this->phase != phase)
    {
        this->phase = phase;
        this->refreshState = true;
    }
    changeToBreathing(false);
}
void Match::setWinner(String winner)
{
    Serial.print("Changed winner to ");
    Serial.println(winner);
    if (this->winner != winner)
    {
        this->winner = winner;
        this->refreshState = true;
    }
}
void Match::setBomb(String bomb)
{
    Serial.print("Changed winner to ");
    Serial.println(bomb);
    if (this->bomb != bomb)
    {
        this->bomb = bomb;
        this->refreshState = true;
    }
}

void Match::handle()
{
    if (millis() > lastRequest + 120000 && lastRequest != 0)
    {
        usingCSGOData = false;
        loadSavedColors();
        lastRequest = 0;
    }
    if (refreshState)
    {
        if (lastRequest == 0)
        {
            saveCurrentColors();
        }
        Serial.println("Refreshing state");
        usingCSGOData = true;
        lastRequest = millis();
        if (phase == "freezetime")
        {
            // Serial.println("Handling freezetime");
            //  mvp = newMvp;
            if (team == "CT")
            {
                setColorTransition(0, 100, 100, 500);
            }
            else if (team == "T")
            {
                setColorTransition(100, 70, 0, 500);
            }
        }
        else if (phase == "over")
        {
            // Serial.println("Handling round over");
            //  String winningTeam = postData["round"]["win_team"];
            //  if (newMvp > mvp)
            //  {
            //  setColorTransition(48, 48, 0);
            //  }
            bool myTeamWin = winner == team;
            if (myTeamWin)
            {
                setColorTransition(10, 70, 10);
            }
            else
            {
                setColorTransition(70, 10, 10);
            }
        }
        else if (phase == "live")
        {
            if (kills > lastKills)
            {
                setColor(125, 125, 125);
            }
            else if (kills < lastKills)
            {
                lastKills = kills;
            }

            if (burn > 0)
            {
                setColorTransition(70, 30, 5, 100);
            }
            else if (smoke > 60)
            {
                setColorTransition(30, 30, 30, 400);
            }
            else if (flash > 60)
            {
                setColorTransition(60, 60, 60, 50);
            }
            else if (health < 20)
            {
                setColorTransition(64, 0, 64, 100);
            }
            else if (bomb == "planted")
            {
                setColor(80, 20, 20);
                changeToBreathing(true);
            }
            else if (team == "CT")
            {
                setColorTransition(20, 40, 40, 700);
            }
            else if (team == "T")
            {
                setColorTransition(40, 40, 20, 700);
            }
            lastKills = kills;
        }
        this->refreshState = false;
    }
};