#include <Arduino.h>
#include "Wire.h"


volatile bool irqChanged2 = false;
volatile int irqState2 = LOW;

auto irqPin2 = 7;

void irqInterrupt2() {
  irqState2 = digitalRead(irqPin2);
  irqChanged2 = true;
}

// put function declarations here:
int myFunction(int, int);

void i2c_scan_setup() {
  Serial.begin(115200);
  Wire.begin(4, 5); // SDA, SCL
  pinMode(irqPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(7), irqInterrupt2, CHANGE);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void i2c_scan_loop() {
  delay(1000);
  Serial.println("Hello, World!");

  byte error, address;
  int nDevices = 0;

  if (irqChanged2) {
    if (irqState2 == HIGH) {
      Serial.println("引脚 IRQ 从 LOW 变为 HIGH");
    } else {
      Serial.println("引脚 IRQ 从 HIGH 变为 LOW");
    }
    irqState2 = false;
  }

  Serial.println("正在扫描 I2C 设备 ...");
  for (address = 0x01; address < 0x7f; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("在地址 0x%02X 发现 I2C 设备\n", address);
      nDevices++;
    } else if (error != 2 && error != 5) {
      Serial.printf("在地址 0x%02X 发生错误 %d\n", address, error);
    }
  }
  if (nDevices == 0) {
    Serial.println("未发现 I2C 设备");
  }
  delay(1000);
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}