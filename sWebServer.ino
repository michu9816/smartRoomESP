// HTTP
ESP8266WebServer server(80);


void httpLedStatus(){
  String res = ledsOn ? "1" : "0";
  server.send(200, "text/plain", res);
}


void checkIncommingRequest(){
    if(server.method() != HTTP_POST){
        server.send(200, "text/plain", "Spoczko");
    }else{
      if(useCSData){
        csgoHandler(server.arg("plain"));
      }
    }
}

void webServerInitialize(){
    server.on("/", checkIncommingRequest);
    server.on("/ledOn", httpFadeIn);
    server.on("/ledOff", httpFadeOut);
    server.on("/ledStatus", httpLedStatus);
    server.begin();
    ElegantOTA.begin(&server);    // Start ElegantOTA
}

void webServerHandler(){
    server.handleClient();
}