#include "Sequencer.h"

unsigned int t0 = millis();
Sequencer seq;


void setup() {
  seq.init(8, 2048, 240);
  seq.run();
}


void loop() {
    if(millis() - t0 > 500){
      t0 = millis();
      Serial.println("alive");

   }

}