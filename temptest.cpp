/**
 * Arduino Uno driven PWM modulation of fan control speed. Temprature is measured using analog thermometer LM35DZ (10 mV/C).
 * Switching is controlled by AQH1213AX Solid-State-Relay. It is zero-crossed at 50V or below to minimize EM noise. 
 *   
 */


#include <inttypes.h>
//#include <avr/interrupt.h>      


#define AC_PERIODE              10      // 10 ms per half periode in 50Hz AC supply
#define DC_SUPPLY_V             5       // 5V DC Power Supply Voltage
#define TEMP_UPDATE_INTERVAL_MS 1000    // [ms]

#define WINDOW_CYCLES   100              // 100 cycles in switching window (eg, SSR at 30%: 15 cycles ON, 35 cycles OFF)
volatile unsigned int8_t cycleSeenThisWindow;        // counts the elapsed cycles within a full cycle window
volatile unsigned int8_t onCyclesThisWindow;         // Number of cycles PWM should be ON in current window
volatile bool pwm                                   // wheter the current status of PWM signal is ON (1) or OFF (0) given by the cycle count

volatile signed int16_t temprature_C;               // Measured temprature in Celsius
volatile const signed int16_t tempLow_C;                  // Lower bound for linear fan control
volatile const signed int16_t tempHigh_C;                 // Upper bound for linear fan control

/**
 * Pin setup
 */
#define PIN_TEMP_SENSOR A0
#define PIN_SSR 1


/**
 * Initilize Arduino uno with the correct pin modes.
 */
void setup()
{
    // set the ADC resolution of Analog Pins
    //analogReadResolution(ADC_RESOLUTION);

    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);

    cycleSeenThisWindow = 0;
    onCyclesThisWindow = 0;

    // bounds for temprature control. can be modified in API later
    tempLow_C = 30;   // degrees C
    tempHigh_C = 60;  // degrees C


    // === Timer Setup ===
    noInterrupts();           // disable all interrupts
    
    // === Timer1: 1 second interrupt ===
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // OCRxx formula: counter_value = (CPU_freq / (prescaler * fire_frequency)
    OCR1A = 62500;              // 62500 = (16 MHz / (256 * 1Hz)) Output compare register
    TCCR1B |= (1 << WGM12);     // CTC mode
    TCCR1B |= (1 << CS12);      // 256 prescaler
    TIMSK1 |= (1 << OCIE1A);    // enable Timer1 compare interrupt

    
    // === Timer2: 20 ms interrupt ===
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;

    // Compute OCR2A for 10ms
    OCR2A = 156;                // 100 Hz = (16MHz / (1024 * 100) 
    TCCR2A |= (1 << WGM21);     // CTC mode
    TCCR2B |= (1 << CS22) | (1 << CS20);  // 1024 prescaler
    TIMSK2 |= (1 << OCIE2A);    // enable Timer2 compare interrupt

    interrupts();             // enable all interrupts

}



/** 
 * Timer interrupt service routine. At timer interrupt (1Hz), update temprature_C value.
 * ISR is a macro defined in avr/interrupts.h library
 */
ISR(TIMER1_COMPA_vect)
{
    temprature_C = analogRead(PIN_TEMP_SENSOR) / 10;                            // scaled by 1/10 according to datasheet of LM35DZ
    cycleSeenThisWindow = 0;                                                    // reset cycle counter
    if      (temprature_C < tempLow_C)      onCyclesThisWindow = 0;                       // OFF for the whole duration of window.
    else if (temprature_C >= tempHigh_C)    onCyclesThisWindow = WINDOW_CYCLES;    // ON for the whole duration of window.
    else {
        onCyclesThisWindow = WINDOW_CYCLES * (temprature_C - tempLow_C) / (tempHigh_C - tempLow_C);      // Linear in between tempLow_C and tempHigh_C
    }
}

ISR(TIMER2_COMPA_vect)
{
    if (cyclesSeenThisWindow < WINDOW_CYCLES - onCyclesThisWindow) pwm = true;
    else pwm = false;

    cyclesSeenThisWindow++;
}

/**
 * 
 */
loop()
{

}