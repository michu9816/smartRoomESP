#include <DNSServer.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>    //Local WebServer used to serve the configuration portal
#include <ESPAsync_WiFiManager.h> //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <AsyncElegantOTA.h>
#include <csgo.h>
#include <ArduinoJson.h>

extern Match match;

static AsyncWebServer server(80);
static DNSServer dns;

#include <wifi.h>

const char *PARAM_MESSAGE = "body";

void initializeWiFi()
{
    ESPAsync_WiFiManager wifiManager(&server, &dns);

    wifiManager.setSTAStaticIPConfig(IPAddress(192, 168, 8, 41), IPAddress(192, 168, 8, 1), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 8, 1), IPAddress(8, 8, 8, 8));
    wifiManager.setConfigPortalTimeout(120);
    bool connected = wifiManager.autoConnect("NikThinq_LED");
    if (!connected && loadTries == 0)
    {
        Serial.println("Failed to connect with network");
        loadTries++;
        delay(1000);
        initializeWiFi();
        return;
    }
    else
    {
        Serial.println("Ready");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
    // AsyncElegantOTA.begin(&server); // Start ElegantOTA
    server.on("", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message); });

    server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
                         {
        Serial.println("Running");
        if (request->url() == "/csgo") {
            // Serial.println((const char*)data);
            // handleIncommingState((const char*)data);

        DynamicJsonDocument postData(8192);
        DeserializationError error = deserializeJson(postData, (const char*)data);
        if (postData.containsKey("player")) {
            String player = postData["player"];
            Serial.println(player); // Hello
            match.setHealth(postData["player"]["state"]["health"]);
            match.setKills(postData["player"]["state"]["round_kills"]);
            match.setFlash(postData["player"]["state"]["flashed"]);
            match.setSmoke(postData["player"]["state"]["smoked"]);
            match.setBurn(postData["player"]["state"]["burning"]);
            String Jteam = postData["player"]["team"];
            match.setTeam(Jteam);
        }
        if (postData.containsKey("round")) {
            String round = postData["round"];
            Serial.println(round); // Hello
            String Jphase = postData["round"]["phase"];
            match.setPhase(Jphase);
            String Jwinner = postData["round"]["win_team"];
            match.setWinner(Jwinner);
            String Jbomb = postData["round"]["bomb"];
            match.setBomb(Jbomb);
        }

        // Serial.println((const char*)data);
        //   DynamicJsonBuffer jsonBuffer;
        //   JsonObject& root = jsonBuffer.parseObject((const char*)data);
        //   if (root.success()) {
        // if (root.containsKey("command")) {
        //   Serial.println(root["command"].asString()); // Hello
        // }
     //   }
      request->send(200, "text/plain", "end");
    } });
    server.begin();
}