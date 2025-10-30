#ifndef TEMPSENSE_H
#define TEMPSENSE_H

#include <Arduino.h>

#define TEMP_SENSOR_PIN         A0
#define ADC_RESOLUTION          10      // 10-bit ADC resolution

void setupTempSensor();
int readTemperatureC();        // Reads temperature in Celsius. range [2 .. 150]
int readTemperatureRAW();      // Reads raw ADC value from temp sensor (10-bit ADC)

#endif // TEMPSENSE_H
