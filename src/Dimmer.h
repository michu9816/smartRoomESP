#include <WebSocketsServer.h>

class Dimmer{
    public:
        static int devicesNumber;
        Dimmer();
        Dimmer(int);
        int getBrightness();
        int getLastBrightness();
        void initialize();
        void brightnessTransition(int);
        void brightnessTransition(int,long);
        void brightnessTransition(int,long,bool);
        void brightnessSet(int);
        void brightnessSet(int,bool,bool);
        void handler();
        void fadeIn();
        void fadeOut();
        void breathingHandler();
        void setBreathFrequency(double);
    private:
        int id;
        int brightness = 0;
        int lastBrightness = 100;
        int brightnessRequired;
        int brightnessStep;
        int outputPin;
        int inputPin;
        int nextTransitionMillisStep;
        int breathStepTime = 20;
        long nextTransitionMillis;
        long lastBreathMillis = 0;
        long lastBreathStepMillis = 0;
        long transitionDuration = 750;
        double breathFrequency = 3;
        double breathMultipler = 1.0;
        bool turnedOn = false;
        bool saveLastColor = true;
        String breathDecision = "increase";
        String transitionDecision;
        
        // USTAWIENIA DLA PRZYCISKÓW
        int buttonBrightnessStep = 1;
        int buttonHoldDuration = 200;
        int buttonHoldNextStepDuration = 15;
        long buttonLastPress; 
        long debounceTime = 50; 
        long lastDebounceTime; 
        long lastWebsocketRefresh = 0; 
        long websocketRefreshDelay = 500;
        bool buttonLastState = HIGH;
        bool buttonState = HIGH;
        bool buttonPressed = false;
        bool buttonLongPressed = false;
        bool buttonDebounceState = HIGH;
        bool buttonDebounceLastState = HIGH;
        String buttonBrightnessDecision = "increase";

        void buttonsHandler();
        void refreshWebsocketState();
};
