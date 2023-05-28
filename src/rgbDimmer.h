#include <Dimmer.h>

#define LED_RED 15
#define LED_GREEN 13
#define LED_BLUE 12

static Dimmer red(LED_RED);
static Dimmer green(LED_GREEN);
static Dimmer blue(LED_BLUE);

static int savedColorRed;
static int savedColorGreen;
static int savedColorBlue;

enum shineMode
{
    STABLE,
    FADE,
    STROBE,
    BREATHING
};

static shineMode sMode = STABLE;

String httpFadeIn();
String httpFadeOut();

void initializeRGBDimmer();
void handlerRGBDimmer();
void fadeOut();
void fadeIn();
void changeToBreathing(bool);
void saveCurrentColors();
void loadSavedColors();
void setColor(int, int, int);
void setColorTransition(int, int, int);
void setColorTransition(int, int, int, long);
