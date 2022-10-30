#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

const uint16_t kRecvPin = 2;
unsigned long key_value = 0;

IRrecv irrecv(kRecvPin);

decode_results results;

void initializeIR()
{
  irrecv.enableIRIn();
}

void IRHandler()
{
  if (irrecv.decode(&results))
  {
    // print() & println() can't handle printing long longs. (uint64_t)
    serialPrintUint64(results.value, HEX);
    textAll("New signal");
    textAll(String(results.value));
    irrecv.resume(); // Restart the ISR state machine and Receive the next value
  }
}