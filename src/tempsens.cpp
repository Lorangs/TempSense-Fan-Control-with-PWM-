#include "tempsense.h"

void setupTempSensor()
{
    // Only for platforms that support setting ADC resolution
    #if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_ESP32)
        analogReadResolution(ADC_RESOLUTION); 
    #endif

    pinMode(TEMP_SENSOR_PIN, INPUT);

}

int readTemperatureRAW()
{
    // Read the raw ADC value from the temperature sensor pin
    int16_t rawValue = (int16_t)(analogRead(TEMP_SENSOR_PIN));
    if (rawValue > 0)
    {
        return rawValue;
    }
    else
    {
        return 0; // Error reading sensor
    }
}

int readTemperatureC()
{
    int rawADC = readTemperatureRAW();
    if (rawADC == 0) return -1; // Error condition
    return rawADC * 500 / 1023; 
    
}