#include <Wire.h>

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X 0     
#define Y 1
#define Z 2    

#define FREQ 250 // Frequency in MHz

#define LEDPIN 13

// Maps control axes to input channels
int mode_mapping[4];

// ------------------ Capturing Input Signal ------------------- //

volatile unsigned long current_time;
volatile unsigned long timer[4];

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel in µs
volatile unsigned int pulse_duration[4] = {1500, 1000, 1500, 1500};

// ------------------- ESC Signal Generation -------------------- //

unsigned int esc_pulses[4] = {1000, 1000, 1000, 1000};
unsigned int esc_timer;
unsigned int now, duration;
unsigned int period = 1000000/FREQ;

enum State { STOPPED, STARTED };
State drone_state = STOPPED;
// ------------------------------------------------------------- //

void stateTransition() {
    if (drone_state == STOPPED) {
        if (pulse_duration[YAW] < 1200 && pulse_duration[THROTTLE] < 1200) {
            drone_state = STARTED;
        }
    } else if (drone_state == STARTED) {
        if (pulse_duration[YAW] > 1800 && pulse_duration[THROTTLE] < 1200) {
            drone_state = STOPPED;
            digitalWrite(LEDPIN, HIGH);
        }       
    }
}

void calcESCPulses() {
    if (drone_state == STOPPED) {
        esc_pulses[0] = 1000;
        esc_pulses[1] = 1000;
        esc_pulses[2] = 1000;
        esc_pulses[3] = 1000;

        return;
    }
    
    esc_pulses[0] = 1200;
    esc_pulses[1] = 1200;
    esc_pulses[2] = 1200;
    esc_pulses[3] = 1200;

}

/**
 * sendESCPulses sends pwm signals to the escs every 'period' microseconds.
 * ESCs on ports 4-7.
 */ 
void sendESCPulses() {

    // Start sending pulses every 'period' microseconds
    while ((micros() - esc_timer) < period);

    esc_timer = micros();
    PORTD |= B11110000;

    while (PORTD >= 16) {
        now = micros();
        duration = now - esc_timer;

        // If esc 0  has had its pulse, stop it.
        if (esc_pulses[0] <= duration) {
            PORTD &= B01111111;
        }
        if (esc_pulses[1] <= duration) {
            PORTD &= B10111111;
        }
        if (esc_pulses[2] <= duration) {
            PORTD &= B11011111;
        }
        if (esc_pulses[3] <= duration) {
            PORTD &= B11101111;
        }
    }
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

/**
 * setupMPU turns the MPU6050 and configures its registers to provide gyro
 * and accelerometer data.
 */
void setupMPU() {

}

void setup() {

    // LED for status indication
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, HIGH);

    // Set pins 4-7 as outputs 
    DDRD |= B11110000;

    // setupMPU();
    configueChannelMapping();

    // Set pin registers to interrupt on controller input
    // PCINT0..3 on the ATMega correspond to pins 8..11 on the Arduino Uno
    PCICR  |= (1 << PCIE0); // Set PCMSK0 scan (watch the register for change)
    PCMSK0 |= (1 << PCINT0); // Set pin 0 to interrupt on change
    PCMSK0 |= (1 << PCINT1);
    PCMSK0 |= (1 << PCINT2);
    PCMSK0 |= (1 << PCINT3);

    // Start the motors
    sendESCPulses();
}

void loop() {

    stateTransition();

    // Read MPU 
    // Calculate orientation
    // Do PID

    calcESCPulses();
    sendESCPulses();

    if (drone_state == STARTED) {
        digitalWrite(LEDPIN, millis()>>10 &(1));
    }
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