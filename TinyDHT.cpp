/*!
 * @file TinyDHT.cpp
 *
 * @mainpage Adafruit TinyDHT Sensor Library
 *
 * @section intro_sec Introduction
 *
 * Integer version of the Adafruit DHT library for the
 * Trinket and Gemma mini microcontrollers
 *
 * @section license License
 *
 * MIT license
 *
 * @section author Author
 *
 * Written by Adafruit Industries
 */

#include "TinyDHT.h"

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  _pin = pin;
  _type = type;
  _count = count;
  firstreading = true;
}

void DHT::begin(void) {
  // set up the pins!
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);
  _lastreadtime = 0;
}

// tenths of a degree, always C.
int16_t DHT::readTemperatureTenths(bool S) {
  int16_t f;

  if (read()) {
    switch (_type) {
    case DHT11:
      f = (int16_t)data[2];
      if (S)
        f = convertCtoF(f);
      return f;
    case DHT22:
    case DHT21:
      f = (int16_t)(data[2] & 0x7F) * 256 + (int16_t)data[3];
      return data[2] & 0x80 ? -f : f;
    }
  }
  /* Serial.print("Read fail"); */
  return BAD_TEMP; // Bad read, return value (from TinyDHT.h)
}

int16_t DHT::readTemperature(bool S) {
  uint16_t t = readTemperatureTenths();
  return t == BAD_TEMP ? BAD_TEMP : (S ? convertCtoF(t) : t)/10;
}

int16_t DHT::convertCtoF(int16_t c) { return (c * 9) / 5 + 32; }

uint16_t DHT::readHumidityTenths(void) { // tenths of a percent (0-1000)
  uint8_t f;
  uint16_t f2; // bigger to allow for math operations
  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[0];
      return f;
    case DHT22:
    case DHT21:
      return (uint16_t)data[0] * 256 + data[1];
    }
  }
  /* Serial.print("Read fail"); */
  return BAD_HUM; // return bad value (defined in TinyDHT.h)
}

uint8_t DHT::readHumidity(void) { //  0-100 %
  uint16_t h = readHumidityTenths();
  return h == BAD_HUM ? BAD_HUM : h/10;
}

// Avoid locking up due to unexpected states/electrical issues
// 150us is longer than any reasonable state in the protocol
boolean DHT::waitUntil(uint8_t forState) {
  for(uint8_t i = 150; i--;) {
    delayMicroseconds(1); // could avoid delay by estimating loop cycles per iteration
    if (digitalRead(_pin) == forState) {
      return true;
    }
  }
  return false;
}

// Called when sensor is in the "start transmit" LOW period
uint8_t DHT::readByte(void) {
  uint8_t out = 0;
  uint8_t shift = 7;
  // bus should be in "start transmit" (LOW)
  do {
    // ignore timeouts, the checksum check can deal with it
    if (!waitUntil(HIGH)) return 0; // wait till end of start transmit low
    // pin is now high for 26-28us (0) or 70us (1)
    delayMicroseconds(48);
    out |= digitalRead(_pin) << shift;
    if (!waitUntil(LOW)) return 0; // if necessary, wait for start transmit low
  } while(shift--);
  return out;
}

boolean DHT::read(void) {
  uint8_t i;
  unsigned long currenttime;

  currenttime = millis();
  if (currenttime < _lastreadtime) {
    // ie there was a rollover
    _lastreadtime = 0;
  }
  if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
    return true; // return last correct measurement
    // delay(2000 - (currenttime - _lastreadtime));
  }
  firstreading = false;
  /*
    Serial.print("Currtime: "); Serial.print(currenttime);
    Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
  */
  _lastreadtime = currenttime;

  delay(20); // it's not clear why this is needed???

  // now pull it low for ~20 milliseconds
  digitalWrite(_pin, LOW);
  delay(20);
  cli();
  // sensor will try to pull bus high
  pinMode(_pin, INPUT_PULLUP);
  if (!waitUntil(HIGH)) goto timeout;

  // wait for sensor to pull low
  if (!waitUntil(LOW)) goto timeout;
  // pin low for 80us
  if (!waitUntil(HIGH)) goto timeout;
  // pin high for 80us
  if (!waitUntil(LOW)) goto timeout;
  // pin is now low for 50us ("start transmit")
  for(i = 0; i < 5; ++i) {
    data[i] = readByte();
  }
  sei();
  delay(1); // sensor is supposed to pull bus down for 50us after transmission
  // then host pulls it up as the idle state until next reading
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);

  // check that the checksum matches
  return data[4] == (data[0] + data[1] + data[2] + data[3]) & 0xFF;

timeout: // maybe sensor got confused?
  sei();
  delay(20);
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);
  return false;
}
