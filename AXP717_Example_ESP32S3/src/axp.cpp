#include <Arduino.h>
#include "Wire.h"

volatile bool irqChanged = false;
volatile int irqState = LOW;

auto irqPin = 7;
auto MY_SDA = 6;
auto MY_SCL = 5;
auto AXP2585_ADDR = 0x34;

void irqInterrupt()
{
  irqState = digitalRead(irqPin);
  irqChanged = true;
}

int readRegister(uint8_t reg, uint8_t *buf, uint8_t length)
{
  Wire.beginTransmission(AXP2585_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0)
  {
    return -1;
  }
  Wire.requestFrom(AXP2585_ADDR, length);
  return Wire.readBytes(buf, length) == length ? 0 : -1;
}

int writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(AXP2585_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0) ? 0 : -1;
}

void getIRQEnableRegisters()
{
  Serial.println("IRQ 使能寄存器:");
  // reg40H/41H/42H/43H/44H/45H
  for (int i = 0x40; i <= 0x45; i++)
  {
    uint8_t value = 0;
    auto result = readRegister(i, &value, 1);
    if (result != 0)
    {
      Serial.printf("读取 I2C 寄存器 0x%02X 失败, code=%d\n", i, result);
    }
    else
    {
      Serial.printf("I2C 寄存器 0x%02X 的值: 0x%02X\n", i, value);
    }
  }
}

void getIRQStatus()
{
  Serial.println("IRQ 状态:");
  // reg48H/49H/4AH/4BH/4CH/4DH
  for (int i = 0x48; i <= 0x4D; i++)
  {
    uint8_t value = 0;
    auto result = readRegister(i, &value, 1);
    if (result != 0)
    {
      Serial.printf("读取 I2C 寄存器 0x%02X 失败, code=%d\n", i, result);
    }
    else
    {
      Serial.printf("I2C 寄存器 0x%02X 的值: 0x%02X\n", i, value);
    }
  }
}

void clearIRQIntoI2CRegister()
{
  Serial.println("清除 I2C 寄存器中的 IRQ:");
  // reg48H/49H/4AH/4BH/4CH/4DH
  for (int i = 0x48; i <= 0x4D; i++)
  {
    auto result = writeRegister(i, 0xFF);
    if (result != 0)
    {
      Serial.printf("写入 I2C 寄存器 0x%02X 失败, code=%d\n", i, result);
    }
  }
  Serial.println("已清除 I2C 寄存器中的 IRQ");
}

void detectChip()
{
  while (true)
  {
    Wire.beginTransmission(AXP2585_ADDR);
    if (Wire.endTransmission() == 0)
    {
      Serial.println("AXP2585 芯片已连接");
      break;
    }
    else
    {
      Serial.println("AXP2585 芯片未连接,delay 1s then retry");
      delay(1000);
    }
  }
}

void getChipID()
{
  uint8_t chipID = 0;
  auto result = readRegister(0x03, &chipID, 1);
  if (result != 0)
  {
    Serial.printf("读取芯片 ID 失败, code=%d\n", result);
  }
  else
  {
    Serial.printf("芯片 ID: 0x%02X\n", chipID);
  }
}

void axp_setup()
{
  Serial.begin(115200);
  Wire.begin(MY_SDA, MY_SCL); // SDA, SCL
  pinMode(irqPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(irqPin), irqInterrupt, CHANGE);
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  delay(500);
  detectChip();
  getChipID();
  getIRQEnableRegisters();
  clearIRQIntoI2CRegister();
}

void WireScan()
{
  byte error, address;
  int nDevices = 0;

  Serial.println("正在扫描 I2C 设备 ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("在地址 0x%02X 发现 I2C 设备\n", address);
      nDevices++;
    }
    else if (error != 2 && error != 5)
    {
      Serial.printf("在地址 0x%02X 发生错误 %d\n", address, error);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("未发现 I2C 设备");
  }
}

void axp_loop()
{
  // Serial.println("Hello, World!");
  delay(10);

  if (irqChanged)
  {
    if (irqState == HIGH)
    {
      Serial.println("引脚 IRQ 从 LOW 变为 HIGH");
    }
    else
    {
      Serial.println("引脚 IRQ 从 HIGH 变为 LOW");
      getIRQStatus();
      clearIRQIntoI2CRegister();
    }
    irqChanged = false;
  }
}
