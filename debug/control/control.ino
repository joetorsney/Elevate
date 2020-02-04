/**
 * Prints the controller inputs the Arduino recieves.
 */ 

#include <Wire.h>

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define LEDPIN 13

// Maps control axes to input channels
int mode_mapping[4];

volatile unsigned long current_time;
volatile unsigned long timer[4];

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel in µs
volatile unsigned int pulse_duration[4] = {1500, 1000, 1500, 1500};

void dumpChannels()
{
    for (int i = CHANNEL1; i <= CHANNEL4; i++) {
        Serial.print("Channel ");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(pulse_duration[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}

/**
 * configureChannelMapping maps axes to input channels.
 */
void configueChannelMapping() {
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

void setup() {

    Serial.begin(9600);

    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, HIGH);

    configueChannelMapping();

    // Set pin registers to interrupt on controller input
    // PCINT0..3 on the ATMega correspond to pins 8..11 on the Arduino Uno
    PCICR  |= (1 << PCIE0); // Set PCMSK0 scan (watch the register for change)
    PCMSK0 |= (1 << PCINT0); // Set pin 0 to interrupt on change
    PCMSK0 |= (1 << PCINT1);
    PCMSK0 |= (1 << PCINT2);
    PCMSK0 |= (1 << PCINT3);
}

void loop() {
    dumpChannels();
}

ISR(PCINT0_vect) {
    current_time = micros();

    // Check if pin 8 is high
    if (PINB & B00000001) {
        if (previous_state[CHANNEL1] == LOW) {
            // if change LOW -> HIGH, start timer
            previous_state[CHANNEL1] = HIGH;
            timer[CHANNEL1] = current_time;
        }
        
    } else if (previous_state[CHANNEL1] == HIGH) {
        // if change HIGH -> LOW, stop timer and store it
        previous_state[CHANNEL1] = LOW;
        pulse_duration[CHANNEL1] = (int) current_time - timer[CHANNEL1];
    }

    // Check if pin 9 is high
    if (PINB & B00000010) {
        if (previous_state[CHANNEL2] == LOW) {
            // if change LOW -> HIGH, start timer
            previous_state[CHANNEL2] = HIGH;
            timer[CHANNEL2] = current_time;
        }
        
    } else if (previous_state[CHANNEL2] == HIGH) {
        // if change HIGH -> LOW, stop timer and store it
        previous_state[CHANNEL2] = LOW;
        pulse_duration[CHANNEL2] = (int) current_time - timer[CHANNEL2];
    }

    // Check if pin 10 is high
    if (PINB & B00000100) {
        if (previous_state[CHANNEL3] == LOW) {
            // if change LOW -> HIGH, start timer
            previous_state[CHANNEL3] = HIGH;
            timer[CHANNEL3] = current_time;
        }
        
    } else if (previous_state[CHANNEL3] == HIGH) {
        // if change HIGH -> LOW, stop timer and store it
        previous_state[CHANNEL3] = LOW;
        pulse_duration[CHANNEL3] = (int) current_time - timer[CHANNEL3];
    }

    // Check if pin 11 is high
    if (PINB & B00001000) {
        if (previous_state[CHANNEL4] == LOW) {
            // if change LOW -> HIGH, start timer
            previous_state[CHANNEL4] = HIGH;
            timer[CHANNEL4] = current_time;
        }
        
    } else if (previous_state[CHANNEL4] == HIGH) {
        // if change HIGH -> LOW, stop timer and store it
        previous_state[CHANNEL4] = LOW;
        pulse_duration[CHANNEL4] = (int) current_time - timer[CHANNEL4];
    }
}