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

#define GYRO_SSF 65.5

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

// ------------------------ IMU Signal -------------------------- //

const int IMU_addr = 0x68;

int16_t accel[3] = {0, 0, 0};
int16_t gyro[3] = {0, 0, 0};
int16_t temp;

long gyro_offset[3] = {0, 0, 0};

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
        if (pulse_duration[mode_mapping[YAW]] < 1200 && pulse_duration[mode_mapping[THROTTLE]] < 1200) {
            drone_state = STARTED;
        }
    } else if (drone_state == STARTED) {
        if (pulse_duration[mode_mapping[YAW]] > 1800 && pulse_duration[mode_mapping[THROTTLE]] < 1200) {
            drone_state = STOPPED;
            digitalWrite(LEDPIN, HIGH);
        }       
    }
}

void readIMUSignal() {
    Wire.beginTransmission(IMU_addr);
    Wire.write(0x3B); // Request data starting from this register on the IMU
    Wire.endTransmission(false);

    Wire.requestFrom(IMU_addr, 14, true); // Now ask for 14 bytes.
    accel[X] = Wire.read() << 8 | Wire.read();
    accel[Y] = Wire.read() << 8 | Wire.read();
    accel[Z] = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyro[X] = Wire.read() << 8 | Wire.read();
    gyro[Y] = Wire.read() << 8 | Wire.read();
    gyro[Z] = Wire.read() << 8 | Wire.read();
}

void calcESCPulses() {
    if (drone_state == STOPPED) {
        // 1000ms pulse to keep motors happy.
        esc_pulses[0] = 1000;
        esc_pulses[1] = 1000;
        esc_pulses[2] = 1000;
        esc_pulses[3] = 1000;

        return;
    }
    
    esc_pulses[0] = pulse_duration[mode_mapping[THROTTLE]];
    esc_pulses[1] = pulse_duration[mode_mapping[THROTTLE]];
    esc_pulses[2] = pulse_duration[mode_mapping[THROTTLE]];
    esc_pulses[3] = pulse_duration[mode_mapping[THROTTLE]];
}

/**
 * sendESCPulses sends pwm signals to the escs every 'period' microseconds.
 * ESCs on ports 4-7.
 * 
 *     6   5
 *      \ /
 *       |      ^ Front
 *      / \     
 *     4   7
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
 * setupIMU turns the MPU6050 and configures its registers to provide gyro
 * and accelerometer data.
 * 
 * See MPU6050 Datasheet
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * 
 * MPU6050 Register Map
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * 
 */
void setupIMU() {
    Wire.begin();

    // Set PWR_MGMT register
    Wire.beginTransmission(IMU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    // Set Gyro sensitivity
    Wire.beginTransmission(IMU_addr);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(B00001000); // Set sensitivity scale factor = 65.5
    Wire.endTransmission(true);

    // Set Accel sensitivity
    Wire.beginTransmission(IMU_addr);
    Wire.write(0x1C); // ACCEL_CONFIG register
    Wire.write(B00001000); // Set sensitivity +-4g
    Wire.endTransmission(true);

    // Set low pass filter
    Wire.beginTransmission(IMU_addr);
    Wire.write(0x1A); // CONFIG register
    Wire.write(B0000010); // DLPF @ ~43hz
    Wire.endTransmission(true);

    // 'Zero' the gyro by mean of 2000 samples
    int samples = 2000;
    for (int i = 0; i < samples; i++) {
        readIMUSignal();

        gyro_offset[X] += gyro[X];
        gyro_offset[Y] += gyro[Y];
        gyro_offset[Z] += gyro[Z];

        // Drone state is STOPPED, send 1000ms pulses.
        sendESCPulses();
    }

    // Take mean
    for (int i = 0; i < 2; i++) {
        gyro_offset[i] /= samples;
    }
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