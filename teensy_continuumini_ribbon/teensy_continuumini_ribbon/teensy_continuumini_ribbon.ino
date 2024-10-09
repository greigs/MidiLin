#include "USBHost_t36.h"

USBHost myusb;

void setup() {
  myusb.begin();
}

void loop() {
  delay(10000);
  Serial.println("testing");
}

