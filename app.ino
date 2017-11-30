#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <EnableInterrupt.h>

#define RC_NUM_CHANNELS  3

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rcMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *rcMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *rcMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *rcMotor4 = AFMS.getMotor(4);

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

int speed;
char direction;
int turnRate;

void rc_read_values() {
    noInterrupts();
    memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
    interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
    if (digitalRead(input_pin) == HIGH) {
        rc_start[channel] = micros();
    } else {
        uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
        rc_shared[channel] = rc_compare;
    }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }

void neutralRC()
{
    rcMotor1->setSpeed(0);
    rcMotor2->setSpeed(0);
    rcMotor3->setSpeed(0);
    rcMotor4->setSpeed(0);

    rcMotor1->run(RELEASE);
    rcMotor2->run(RELEASE);
    rcMotor3->run(RELEASE);
    rcMotor4->run(RELEASE);
}

void forwardRC(int speed, char direction, int turnRate)
{
    if (direction == 'N'){
        rcMotor1->setSpeed(speed);
        rcMotor2->setSpeed(speed);
        rcMotor3->setSpeed(speed);
        rcMotor4->setSpeed(speed);
    }
    if (direction == 'L'){
        int speedLeft = speed + turnRate;
        rcMotor1->setSpeed(speed);
        rcMotor2->setSpeed(speedLeft);
        rcMotor3->setSpeed(speedLeft);
        rcMotor4->setSpeed(speed);
    }
    if (direction == 'R'){
        int speedRight = speed + turnRate;
        rcMotor1->setSpeed(speedRight);
        rcMotor2->setSpeed(speed);
        rcMotor3->setSpeed(speed);
        rcMotor4->setSpeed(speedRight);
    }
    rcMotor1->run(FORWARD);
    rcMotor2->run(FORWARD);
    rcMotor3->run(FORWARD);
    rcMotor4->run(FORWARD);
}

void reverseRC(int speed, char direction, int turnRate)
{
    if (direction == 'N'){
        rcMotor1->setSpeed(speed);
        rcMotor2->setSpeed(speed);
        rcMotor3->setSpeed(speed);
        rcMotor4->setSpeed(speed);
    }
    if (direction == 'L'){
        int speedLeft = speed + turnRate;
        rcMotor1->setSpeed(speed);
        rcMotor2->setSpeed(speedLeft);
        rcMotor3->setSpeed(speedLeft);
        rcMotor4->setSpeed(speed);
    }
    if (direction == 'R'){
        int speedRight = speed + turnRate;
        rcMotor1->setSpeed(speedRight);
        rcMotor2->setSpeed(speed);
        rcMotor3->setSpeed(speed);
        rcMotor4->setSpeed(speedRight);
    }
    rcMotor1->setSpeed(speed);
    rcMotor2->setSpeed(speed);
    rcMotor3->setSpeed(speed);
    rcMotor4->setSpeed(speed);

    rcMotor1->run(BACKWARD);
    rcMotor2->run(BACKWARD);
    rcMotor3->run(BACKWARD);
    rcMotor4->run(BACKWARD);
}
void setup() {
    AFMS.begin();
    rcMotor1->setSpeed(0);
    rcMotor2->setSpeed(0);
    rcMotor3->setSpeed(0);
    rcMotor4->setSpeed(0);

    rcMotor1->run(RELEASE);
    rcMotor2->run(RELEASE);
    rcMotor3->run(RELEASE);
    rcMotor4->run(RELEASE);

    pinMode(RC_CH1_INPUT, INPUT);
    pinMode(RC_CH2_INPUT, INPUT);
    pinMode(RC_CH3_INPUT, INPUT);


    enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
    enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
    enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);

}

void loop() {
    rc_read_values();
    // Controller Off
    if (rc_values[RC_CH2] == 0) {
        neutralRC();
        return;
    }

    // Steering
    if ((rc_values[RC_CH1] < 1640 ) && (rc_values[RC_CH1] > 1400)) {
        direction = 'N';
        turnRate = 0;
    }
    if ((rc_values[RC_CH1] < 1400) && (rc_values[RC_CH1] > 900)) {
        direction = 'L';
        turnRate = map(rc_values[RC_CH1], 1400, 960, 0, 50);
        turnRate = constrain(turnRate, 0, 50);
    }
    if ((rc_values[RC_CH1] < 2200) && (rc_values[RC_CH1] > 1640)) {
        direction = 'R';
        turnRate = map(rc_values[RC_CH1], 1640, 2200, 0, 50);
        turnRate = constrain(turnRate, 0, 50);
    }

    // Driving
    if ((rc_values[RC_CH2] <= 1640) && (rc_values[RC_CH2] >= 1400)) {
        neutralRC();
        // Standing Turn
        if ((direction == 'L') || (direction = 'R')) {
            forwardRC(0, direction, turnRate);
        }
        return;
    }
    if ((rc_values[RC_CH2] < 1400) && (rc_values[RC_CH2] > 900)) {
        speed = map(rc_values[RC_CH2], 1400, 960, 0, 200);
        speed = constrain(speed, 0, 200);
        forwardRC(speed, direction, turnRate);
        return;
    }
    if ((rc_values[RC_CH2] > 1640) && (rc_values[RC_CH2] < 2200)) {
        speed = map(rc_values[RC_CH2], 1640, 2150, 0, 200);
        speed = constrain(speed, 0, 200);
        reverseRC(speed, direction, turnRate);
        return;
    }
}