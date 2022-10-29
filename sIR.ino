#include <IRremote.h> //including infrared remote header file

int RECV_PIN = 2; // the pin where you connect the output pin of IR sensor
IRrecv irrecv(RECV_PIN);
decode_results results;

void initializeIR()
{
  irrecv.enableIRIn();
}

void IRHandler()
{
  if (irrecv.decode(&results)) // Returns 0 if no data ready, 1 if data ready.
  {
    int value = results.value;
    textAll(String(value));
    irrecv.resume(); // Restart the ISR state machine and Receive the next value
  }
}