#include <IRremote.h>

// An IRsend instance is used to send data, default send pin is digital 3
IRsend sender;

void setup()
{ /* Initialize sketch */ }

void loop()
{

  /* Do other stuff */

  // The following values were obtained by reading the receiver in a different sketch
  // They correspond to the OFF button of another NEC compatible remote control
  uint32_t data = 0xFF609F;
  uint8_t len = 32;

  sender.sendNEC(data, len);

  // To send messages using a different protocol
  // see: https://github.com/z3t0/Arduino-IRremote/wiki/IRremote-library-API

}
