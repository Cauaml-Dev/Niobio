
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include <stdlib.h>
#include <math.h>

#include "Adafruit_TCS34725_DUE.h"

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

float powf2(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_TCS34725_W0::write8 (uint8_t reg, uint32_t value)
{
  Wire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(TCS34725_COMMAND_BIT | reg);
  Wire.write(value & 0xFF);
  #else
  Wire.send(TCS34725_COMMAND_BIT | reg);
  Wire.send(value & 0xFF);
  #endif
  Wire.endTransmission();
}

void Adafruit_TCS34725_W1::write82 (uint8_t reg2, uint32_t value2)
{
  Wire1.beginTransmission(TCS34725_ADDRESS2);
  #if ARDUINO >= 100
  Wire1.write(TCS34725_COMMAND_BIT2 | reg2);
  Wire1.write(value2 & 0xFF);
  #else
  Wire1.send(TCS34725_COMMAND_BIT2 | reg2);
  Wire1.send(value2 & 0xFF);
  #endif
  Wire1.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_TCS34725_W0::read8(uint8_t reg)
{
  Wire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(TCS34725_COMMAND_BIT | reg);
  #else
  Wire.send(TCS34725_COMMAND_BIT | reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(TCS34725_ADDRESS, 1);
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

uint8_t Adafruit_TCS34725_W1::read82(uint8_t reg2)
{
  Wire1.beginTransmission(TCS34725_ADDRESS2);
  #if ARDUINO >= 100
  Wire1.write(TCS34725_COMMAND_BIT2 | reg2);
  #else
  Wire1.send(TCS34725_COMMAND_BIT2 | reg2);
  #endif
  Wire1.endTransmission();

  Wire1.requestFrom(TCS34725_ADDRESS2, 1);
  #if ARDUINO >= 100
  return Wire1.read();
  #else
  return Wire1.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725_W0::read16(uint8_t reg)
{
  uint16_t x; uint16_t t;

  Wire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(TCS34725_COMMAND_BIT | reg);
  #else
  Wire.send(TCS34725_COMMAND_BIT | reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(TCS34725_ADDRESS, 2);
  #if ARDUINO >= 100
  t = Wire.read();
  x = Wire.read();
  #else
  t = Wire.receive();
  x = Wire.receive();
  #endif
  x <<= 8;
  x |= t;
  return x;
}

uint16_t Adafruit_TCS34725_W1::read162(uint8_t reg2)
{
  uint16_t x2; uint16_t t2;

  Wire1.beginTransmission(TCS34725_ADDRESS2);
  #if ARDUINO >= 100
  Wire1.write(TCS34725_COMMAND_BIT2 | reg2);
  #else
  Wire1.send(TCS34725_COMMAND_BIT2 | reg2);
  #endif
  Wire1.endTransmission();

  Wire1.requestFrom(TCS34725_ADDRESS2, 2);
  #if ARDUINO >= 100
  t2 = Wire1.read();
  x2 = Wire1.read();
  #else
  t2 = Wire1.receive();
  x2 = Wire1.receive();
  #endif
  x2 <<= 8;
  x2 |= t2;
  return x2;
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void Adafruit_TCS34725_W0::enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);  
}

void Adafruit_TCS34725_W1::enable2(void)
{
  write82(TCS34725_ENABLE2, TCS34725_ENABLE_PON2);
  delay(3);
  write82(TCS34725_ENABLE2, TCS34725_ENABLE_PON2 | TCS34725_ENABLE_AEN2);  
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void Adafruit_TCS34725_W0::disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void Adafruit_TCS34725_W1::disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg2 = 0;
  reg2 = read82(TCS34725_ENABLE2);
  write82(TCS34725_ENABLE2, reg2 & ~(TCS34725_ENABLE_PON2 | TCS34725_ENABLE_AEN2));
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
Adafruit_TCS34725_W0::Adafruit_TCS34725_W0(tcs34725IntegrationTime_t it, tcs34725Gain_t gain) 
{
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
}

Adafruit_TCS34725_W1::Adafruit_TCS34725_W1(tcs34725IntegrationTime_t2 it2, tcs34725Gain_t2 gain2) 
{
  _tcs34725Initialised2 = false;
  _tcs34725IntegrationTime2 = it2;
  _tcs34725Gain2 = gain2;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
boolean Adafruit_TCS34725_W0::begin(void) 
{
  Wire.begin();
  
  /* Make sure we're actually connected */
  uint8_t x = read8(TCS34725_ID);
  //Serial.println(x, HEX);
  if (x != 0x44)
  {
    return false;
  }
  _tcs34725Initialised = true;

  /* Set default integration time and gain */
  setIntegrationTime(_tcs34725IntegrationTime);
  setGain(_tcs34725Gain);

  /* Note: by default, the device is in power down mode on bootup */
  enable();

  return true;
}

boolean Adafruit_TCS34725_W1::begin(void) 
{
  Wire1.begin();
  
  /* Make sure we're actually connected */
  uint8_t x2 = read82(TCS34725_ID2);
  //Serial.println(x2, HEX);
  if (x2 != 0x44)
  {
    return false;
  }
  _tcs34725Initialised2 = true;

  /* Set default integration time and gain2 */
  setIntegrationTime2(_tcs34725IntegrationTime2);
  setGain2(_tcs34725Gain2);

  /* Note: by default, the device is in power down mode on bootup */
  enable2();

  return true;
}
  
/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void Adafruit_TCS34725_W0::setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

void Adafruit_TCS34725_W1::setIntegrationTime2(tcs34725IntegrationTime_t2 it2)
{
  if (!_tcs34725Initialised2) begin();

  /* Update the timing reg2ister */
  write82(TCS34725_ATIME2, it2);

  /* Update value2 placeholders */
  _tcs34725IntegrationTime2 = it2;
}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void Adafruit_TCS34725_W0::setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

void Adafruit_TCS34725_W1::setGain2(tcs34725Gain_t2 gain2)
{
  if (!_tcs34725Initialised2) begin();

  /* Update the timing reg2ister */
  write82(TCS34725_CONTROL2, gain2);

  /* Update value2 placeholders */
  _tcs34725Gain2 = gain2;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void Adafruit_TCS34725_W0::getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) begin();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);
  
  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      delay(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      delay(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      delay(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      delay(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      delay(700);
      break;
  }
}

void Adafruit_TCS34725_W1::getRawData2 (uint16_t *r2, uint16_t *g2, uint16_t *b2, uint16_t *c2)
{
  if (!_tcs34725Initialised2) begin();

  *c2 = read162(TCS34725_CDATAL2);
  *r2 = read162(TCS34725_RDATAL2);
  *g2 = read162(TCS34725_GDATAL2);
  *b2 = read162(TCS34725_BDATAL2);
  
  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime2)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS2:
      delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS2:
      delay(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS2:
      delay(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS2:
      delay(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS2:
      delay(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS2:
      delay(700);
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725_W0::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}


uint16_t Adafruit_TCS34725_W1::calculateColorTemperature2(uint16_t r2, uint16_t g2, uint16_t b2)
{
  float X2, Y2, Z2;      /* RGB to XYZ correlation      */
  float xc2, yc2;       /* Chromaticity co-ordinates   */
  float n2;            /* McCamy's formula            */
  float cct2;

  /* 1. Map RGB value2s to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent value2s for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X2 = (-0.14282F * r2) + (1.54924F * g2) + (-0.95641F * b2);
  Y2 = (-0.32466F * r2) + (1.57837F * g2) + (-0.73191F * b2);
  Z2 = (-0.68202F * r2) + (0.77073F * g2) + ( 0.56332F * b2);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc2 = (X2) / (X2 + Y2 + Z2);
  yc2 = (Y2) / (X2 + Y2 + Z2);

  /* 3. Use McCamy's formula to determine the CCT    */
  n2 = (xc2 - 0.3320F) / (0.1858F - yc2);

  /* Calculate the final CCT */
  cct2 = (449.0F * powf2(n2, 3)) + (3525.0F * powf2(n2, 2)) + (6823.3F * n2) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct2;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725_W0::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}



uint16_t Adafruit_TCS34725_W1::calculateLux2(uint16_t r2, uint16_t g2, uint16_t b2)
{
  float illuminance2;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance2 = (-0.32466F * r2) + (1.57837F * g2) + (-0.73191F * b2);

  return (uint16_t)illuminance2;
}









void Adafruit_TCS34725_W0::setInterrupt(boolean i) {
  uint8_t r = read8(TCS34725_ENABLE);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  write8(TCS34725_ENABLE, r);
}

void Adafruit_TCS34725_W1::setInterrupt2(boolean i2) {
  uint8_t r2 = read82(TCS34725_ENABLE2);
  if (i2) {
    r2 |= TCS34725_ENABLE_AIEN2;
  } else {
    r2 &= ~TCS34725_ENABLE_AIEN2;
  }
  write82(TCS34725_ENABLE2, r2);
}








void Adafruit_TCS34725_W0::clearInterrupt(void) {
  Wire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(0x66);
  #else
  Wire.send(0x66);
  #endif
  Wire.endTransmission();
}

void Adafruit_TCS34725_W1::clearInterrupt2(void) {
  Wire1.beginTransmission(TCS34725_ADDRESS2);
  #if ARDUINO >= 100
  Wire1.write(0x66);
  #else
  Wire1.send(0x66);
  #endif
  Wire1.endTransmission();
}







void Adafruit_TCS34725_W0::setIntLimits(uint16_t low, uint16_t high) {
   write8(0x04, low & 0xFF);
   write8(0x05, low >> 8);
   write8(0x06, high & 0xFF);
   write8(0x07, high >> 8);
}

void Adafruit_TCS34725_W1::setIntLimits2(uint16_t low2, uint16_t high2) {
   write82(0x04, low2 & 0xFF);
   write82(0x05, low2 >> 8);
   write82(0x06, high2 & 0xFF);
   write82(0x07, high2 >> 8);
}
