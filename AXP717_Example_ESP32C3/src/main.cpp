#include <Arduino.h>
#include "Wire.h"
#include <driver/rtc_io.h>

#define IO4_PIN 4  // 定义 IO4 的引脚号  


// AXP717 ADDRESS = 0x34

#define AXP717_ICC_CHG_SET (0x62)
#define AXP717_DC_ONOFF_DVM_CTRL (0x80)

#define XPOWERS_AXP717_DCDC2_CTRL (0x84)
#define AXP717_BAT_PERCENT_DATA (0xA4)
#define AXP717_STATUS1 (0x00)
#define AXP717_ADC_CHANNEL_CTRL (0xC0)
#define AXP717_ADC_DATA_VBAT_H (0xC4)
#define AXP717_ADC_DATA_VBAT_L (0xC5)
#define AXP717_ADC_DATA_ICHG_HIGH (0xCA)
#define AXP717_ADC_DATA_ICHG_LOW (0xCB)

volatile bool irqChanged = false;
volatile int irqState = LOW;

auto irqPin = 12;

void irqInterrupt()
{
  irqState = digitalRead(irqPin);
  irqChanged = true;
}

// put function declarations here:
int myFunction(int, int);

void setup()
{
  Serial.begin(115200);
  pinMode(IO4_PIN, INPUT);
  if (digitalRead(IO4_PIN) == LOW) {
    delay(3000);
    Serial.println("----------------- 哈哈，想不用飞线了，一定要用我这段代码 -----------------");
    Serial.println("IO4 is LOW, will restart and force to download mode...");
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT_V); 
    esp_restart();
  }
  // while(1);
  delay(1000);
  // Wire.setClock(100000);
  Wire.begin(0, 1); // SDA, SCL -> C3 的 IO11 是 VDD_SPI, 默认不能用
  pinMode(irqPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(irqPin), irqInterrupt, CHANGE);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

auto __wire = &Wire;
auto __addr = 0x34;

int readRegister(uint8_t reg, uint8_t *buf, uint8_t length)
{
  if (__wire)
  {
    __wire->beginTransmission(__addr);
    __wire->write(reg);
    if (__wire->endTransmission() != 0)
    {
      return -1;
    }
    __wire->requestFrom(__addr, length);
    return __wire->readBytes(buf, length) == length ? 0 : -1;
  }
}

int readRegister(uint8_t reg)
{
  uint8_t val = 0;
  return readRegister(reg, &val, 1) == -1 ? -1 : val;
}

int writeRegister(uint8_t reg, uint8_t *buf, uint8_t length)
{
  if (__wire)
  {
    __wire->beginTransmission(__addr);
    __wire->write(reg);
    __wire->write(buf, length);
    return (__wire->endTransmission() == 0) ? 0 : -1;
  }
  return -1;
}

void print_binary(uint8_t num)
{
  Serial.printf("0b");
  for (int i = 7; i >= 0; i--)
  {
    // uint8_t has8 bits
    Serial.printf("%d", (num >> i) & 1);
  }
  Serial.printf("\n");
}

int writeRegister(uint8_t reg, uint8_t val)
{
  return writeRegister(reg, &val, 1);
}

bool inline setRegisterBit(uint8_t registers, uint8_t bit)
{
  int val = readRegister(registers);
  if (val == -1)
  {
    return false;
  }
  return writeRegister(registers, (val | (_BV(bit)))) == 0;
}

bool inline getRegisterBit(uint8_t registers, uint8_t bit)
{
  int val = readRegister(registers);
  if (val == -1)
  {
    return false;
  }
  return val & _BV(bit);
}

/*
 * Power control DCDC2 functions
 */
bool isEnableDC2(void)
{
  return getRegisterBit(AXP717_DC_ONOFF_DVM_CTRL, 1);
}

// getBatPresentState
bool isBatteryConnect(void)
{
  return getRegisterBit(AXP717_STATUS1, 3);
}

int getBatteryPercent(void)
{
  if (!isBatteryConnect())
  {
    return -1;
  }
  return readRegister(AXP717_BAT_PERCENT_DATA);
}

void printDC2VolReg()
{
  auto val = readRegister(XPOWERS_AXP717_DCDC2_CTRL);
  Serial.printf("DC2 Voltage Register: ");
  print_binary(val);
}

uint16_t inline readRegisterH5L8(uint8_t highReg, uint8_t lowReg)
{
  int h5 = readRegister(highReg);
  int l8 = readRegister(lowReg);
  if (h5 == -1 || l8 == -1)
    return 0;
  return ((h5 & 0x1F) << 8) | l8;
}

uint16_t inline readRegisterH6L8(uint8_t highReg, uint8_t lowReg)
{
  int h6 = readRegister(highReg);
  int l8 = readRegister(lowReg);
  if (h6 == -1 || l8 == -1)
    return 0;
  return ((h6 & 0x3F) << 8) | l8;
}

void printAdcControlReg()
{
  // default: 0b00000011
  auto val = readRegister(AXP717_ADC_CHANNEL_CTRL);
  Serial.printf("ADC Control Register: ");
  print_binary(val);
}

bool enableADCChargeCurrentMeasure(void)
{
  return setRegisterBit(AXP717_ADC_CHANNEL_CTRL, 5);
}

void printIchgAdcVal()
{
  uint16_t ichg = readRegisterH5L8(AXP717_ADC_DATA_ICHG_HIGH, AXP717_ADC_DATA_ICHG_LOW);
  Serial.printf("Charge Current: ");
  Serial.println(ichg);
}

uint16_t getBattVoltage(void)
{
  if (!isBatteryConnect())
  {
    return 0;
  }
  return readRegisterH5L8(AXP717_ADC_DATA_VBAT_H, AXP717_ADC_DATA_VBAT_L);
}

// uint16_t getDC2Voltage(void)
// {
//   int val = readRegister(XPOWERS_AXP2101_DCDC2_CTRL);
//   if (val == -1)
//     return 0;
//   val &= 0x7F;
//   if (val < XPOWERS_AXP2101_DCDC2_VOL_STEPS2_BASE)
//   {
//     return (val * XPOWERS_AXP2101_DCDC2_VOL_STEPS1) + XPOWERS_AXP2101_DCDC2_VOL1_MIN;
//   }
//   else
//   {
//     return (val * XPOWERS_AXP2101_DCDC2_VOL_STEPS2) - 200;
//   }
//   return 0;
// }

void setDC2VoltageTo3V4()
{
  uint8_t val = 0b11101011;
  writeRegister(XPOWERS_AXP717_DCDC2_CTRL, val);
}

void getAndSetChargeCurrent()
{
  auto curr = readRegister(AXP717_ICC_CHG_SET);
  print_binary(curr);
  uint8_t max_current = 0b101111; // max 3008mA
  float step = 64;                // 64mA / step
  float current = 4000;           // mA
  uint8_t current_reg = (uint8_t)(current / step);
  current_reg = constrain(current_reg, 0, max_current);

  writeRegister(AXP717_ICC_CHG_SET, current_reg);
  curr = readRegister(AXP717_ICC_CHG_SET);
  Serial.printf("Now the charge current is: ");
  print_binary(curr);
}

void loop()
{
  Serial.println("Hello, World!");

  byte error, address;
  int nDevices = 0;

  if (irqChanged)
  {
    if (irqState == HIGH)
    {
      Serial.println("引脚 IRQ 从 LOW 变为 HIGH");
    }
    else
    {
      Serial.println("引脚 IRQ 从 HIGH 变为 LOW");
    }
    irqState = false;
  }

  Serial.println("正在扫描 I2C 设备 ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("在地址 0x%02X 发现 I2C 设备\n", address);
      nDevices++;
      if (address == 0x34)
      {
        printAdcControlReg();
        enableADCChargeCurrentMeasure();
        getAndSetChargeCurrent();
        Serial.printf("DC2 ENABLE = ");
        Serial.println(isEnableDC2());
        printDC2VolReg();
        setDC2VoltageTo3V4();
        auto percent = getBatteryPercent();
        Serial.printf("Battery Percent: %d\n", percent);
        printIchgAdcVal();
        uint16_t vbat = getBattVoltage();
        Serial.printf("Battery Voltage: %d\n", vbat);
      }
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
  delay(500);
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}