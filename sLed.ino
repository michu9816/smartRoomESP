extern AsyncWebServer server;

void httpFadeIn(){
  fadeIn();
//   server.send(200, "text/plain", "włączono");
}

void httpFadeOut(){
  fadeOut();
//   server.send(200, "text/plain", "wyłączono");
}

void fadeOut(){
  red.fadeOut();
  green.fadeOut();
  blue.fadeOut();
};

void fadeIn(){
  red.fadeIn();
  green.fadeIn();
  blue.fadeIn();
};

void setColor(int redColor, int greenColor, int blueColor){
  red.brightnessSet(redColor);
  green.brightnessSet(greenColor);
  blue.brightnessSet(blueColor);
};
