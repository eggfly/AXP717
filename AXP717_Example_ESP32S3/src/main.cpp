#include <Arduino.h>

void i2c_scan_setup();
void i2c_scan_loop();

void setup()
{
  i2c_scan_setup();
}

void loop()
{
  i2c_scan_loop();
}
