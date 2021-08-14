/*!
 * @file TinyDHT.h
 */
#ifndef DHT_H
#define DHT_H
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Tiny DHT library
Uses integer math to save space on Trinket/Gemma

MIT license
written by Adafruit Industries
*/

#define MAXTIMINGS                                                             \
  85 //!< how many timing transitions we need to keep track of. 2 * number bits
     //!< + extra

#define DHT11 11  //!< Used to specify that you want to use the DHT11
#define DHT22 22  //!< Used to specify that you want to use the DHT22
#define DHT21 21  //!< Used to specify that you want to use the DHT21
#define AM2301 21 //!< Used to specify that you want to use the AM2301

// NAN code in DHT library takes space, define bad values here
#define BAD_HUM -1    //!< Bad humidity reading
#define BAD_TEMP -999 //!< Bad temperature reading

#define CELSIUS_SCALE false
#define FAHRENHEIT_SCALE true

/*!
 * @brief Class that stores the state and functions for the DHT
 */
class DHT {
private:
  uint8_t data[6];
  uint8_t _pin, _type, _count;
  boolean read(void);
  unsigned long _lastreadtime;
  boolean firstreading;

public:
  /*!
   * @brief DHT constructor
   * @param pin Pin connected to the DHT
   * @param type What sensor you're connecting, DHT11, DHT22, DHT21, AM2301
   * @param count Used to set how much data to take at a time. Very few reasons
   * to change this
   */
  DHT(uint8_t pin, uint8_t type, uint8_t count = 6);
  /*!
   * @brief Begins connection with device
   */
  void begin(void);
  /*!
   * @brief Reads the temperature from device
   * @param S Scale. True = Fahrenheit, False = Celsius
   * @return Returns the temperature
   */
  int16_t readTemperatureTenths(bool S = false);
  int16_t readTemperature(bool S = false);
  /*!
   * @brief Converts Celsius to Fahrenheit
   * @param c Degrees celsius
   * @return Returns the inputted degrees converted to Fahrenheit
   */
  int16_t convertCtoF(int16_t);
  /*!
   * @brief Reads the humidity from the device
   * @return Returns the humidity read from the device
   */
  uint16_t readHumidityTenths(void);
  uint8_t readHumidity(void);
};
#endif
