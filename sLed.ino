extern ESP8266WebServer server;

void httpFadeIn(){
  fadeIn();
  server.send(200, "text/plain", "włączono");
}

void httpFadeOut(){
  fadeOut();
  server.send(200, "text/plain", "wyłączono");
}

void fadeOut(){
  turnOffLight();
};

void fadeIn(){
  turnOnLight();
};

void setColor(int redColor, int greenColor, int blueColor){
  red.brightnessSet(redColor);
  green.brightnessSet(greenColor);
  blue.brightnessSet(blueColor);
};
