#include <WString.h>;

class Match
{
public:
    Match();
    void setHealth(int);
    void setKills(int);
    void setFlash(int);
    void setSmoke(int);
    void setBurn(int);
    void setBomb(String);
    void setPhase(String);
    void setTeam(String);
    void setWinner(String);
    void handle();
    bool usingCSGOData = false;

private:
    int health;
    int kills;
    int flash = 0;
    int smoke = 0;
    int burn = 0;
    int lastKills;
    String team;
    String phase;
    String winner;
    String bomb;
    bool refreshState = false;
    long lastRequest = 0;
};