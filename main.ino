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

#define LEDPIN 13

// Maps control axes to input channels
int mode_mapping[4];

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
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, HIGH);

    // setupMPU();
    configueChannelMapping();

    delay(3000);
}

void loop() {
    // Read sensors
    // Calculate orientation
    // Do PID
    // Write to Motors
    digitalWrite(LEDPIN, millis()>>10 &1);
}