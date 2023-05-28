#include <rgbDimmer.h>

String httpFadeIn()
{
    fadeIn();
    return "włączono";
}

String httpFadeOut()
{
    fadeOut();
    return "wyłączono";
}

void initializeRGBDimmer()
{
    digitalWrite(LED_RED, 0);
    digitalWrite(LED_GREEN, 0);
    digitalWrite(LED_BLUE, 0);

    red.initialize();
    green.initialize();
    blue.initialize();
}

void handlerRGBDimmer()
{

    red.handler();
    green.handler();
    blue.handler();

    // if (sMode == FADE && ledsOn)
    // {
    //     if (millis() > nextFadeTime + fadeDuration)
    //     {

    //         switch (step)
    //         {
    //         case 1:
    //             red.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             green.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             blue.brightnessTransition(25, fadeDuration, true);
    //             step++;
    //             break;
    //         case 2:
    //             red.brightnessTransition(25, fadeDuration, true);
    //             green.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             blue.brightnessTransition(25, fadeDuration, true);
    //             step++;
    //             break;
    //         case 3:
    //             red.brightnessTransition(25, fadeDuration, true);
    //             green.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             blue.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             step++;
    //             break;
    //         case 4:
    //             red.brightnessTransition(25, fadeDuration, true);
    //             green.brightnessTransition(25, fadeDuration, true);
    //             blue.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             step++;
    //             break;
    //         case 5:
    //             red.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             green.brightnessTransition(25, fadeDuration, true);
    //             blue.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             step++;
    //             break;
    //         default:
    //             red.brightnessTransition(fadeBrightness, fadeDuration, true);
    //             green.brightnessTransition(25, fadeDuration, true);
    //             blue.brightnessTransition(25, fadeDuration, true);
    //             step = 1;
    //             break;
    //         }
    //         nextFadeTime = millis();
    //     }
    // }
    // else if (sMode == STROBE && ledsOn)
    // {
    //     if (millis() > (nextFadeTime + (step == 1 ? 20 : 60)))
    //     {

    //         switch (step)
    //         {
    //         case 1:
    //             red.brightnessSet(red.getLastBrightness(), false, false);
    //             green.brightnessSet(green.getLastBrightness(), false, false);
    //             blue.brightnessSet(blue.getLastBrightness(), false, false);
    //             step++;
    //             break;
    //         default:
    //             red.brightnessSet(0, false, false);
    //             green.brightnessSet(0, false, false);
    //             blue.brightnessSet(0, false, false);
    //             step = 1;
    //             break;
    //         }
    //         nextFadeTime = millis();
    //     }
    // }
    // else if (sMode == BREATHING && ledsOn)
    // {
    //     red.breathingHandler();
    //     green.breathingHandler();
    //     blue.breathingHandler();
    // }
    if (sMode == BREATHING)
    {
        red.breathingHandler();
        green.breathingHandler();
        blue.breathingHandler();
    }
}

void fadeOut()
{
    red.fadeOut();
    green.fadeOut();
    blue.fadeOut();
};

void fadeIn()
{
    red.fadeIn();
    green.fadeIn();
    blue.fadeIn();
};

void changeToBreathing(bool yes)
{
    if (yes)
    {
        sMode = BREATHING;
    }
    else
    {
        sMode = STABLE;
    }
    Serial.println("Changing to BRETHING");
}

void saveCurrentColors()
{
    savedColorRed = red.getBrightness();
    savedColorGreen = green.getBrightness();
    savedColorBlue = blue.getBrightness();
}

void loadSavedColors()
{
    red.brightnessSet(savedColorRed);
    green.brightnessSet(savedColorGreen);
    blue.brightnessSet(savedColorBlue);
}

void setColor(int redColor, int greenColor, int blueColor)
{
    red.brightnessSet(redColor, false, true);
    green.brightnessSet(greenColor, false, true);
    blue.brightnessSet(blueColor, false, true);
};
void setColorTransition(int redColor, int greenColor, int blueColor)
{
    setColorTransition(redColor, greenColor, blueColor, 50);
};
void setColorTransition(int redColor, int greenColor, int blueColor, long duration)
{
    red.brightnessTransition(redColor, duration, false);
    green.brightnessTransition(greenColor, duration, false);
    blue.brightnessTransition(blueColor, duration, false);
    Serial.print("Changing color to ");
    Serial.print(redColor);
    Serial.print(", ");
    Serial.print(greenColor);
    Serial.print(", ");
    Serial.println(blueColor);
};
