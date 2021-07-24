// HTTP
AsyncWebServer server(80);
extern AsyncWebSocket webSocket;

// void httpLedStatus(){
//   String res = ledsOn ? "1" : "0";
//   server.send(200, "text/plain", res);
// }


// void checkIncommingRequest(){
//     if(server.method() != HTTP_POST){
//         server.send(200, "text/plain", "Spoczko");
//     }else{
//       if(useCSData){
//         csgoHandler(server.arg("plain"));
//       }
//     }
// }

void webServerInitialize(){
//     server.on("/", checkIncommingRequest);
//     server.on("/ledOn", httpFadeIn);
//     server.on("/ledOff", httpFadeOut);
//     server.on("/ledStatus", httpLedStatus);
//     server.begin();
server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });
    AsyncElegantOTA.begin(&server);
    server.addHandler(&webSocket);
    server.begin();
    // ElegantOTA.begin(&server);    // Start ElegantOTA
}

// void webServerHandler(){
//     server.handleClient();
// }