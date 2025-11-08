#include "mbed.h"
#include "hcsr04.h"
#include "Servo.h"
#include "ThisThread.h"

/*
 * Line-Following Robot with PID Control & Ultrasonic Obstacle Avoidance
 * Target MCU : STM32 Nucleo-F767ZI
 *
 * Author : Mohitkumar Kori
 * Module : Embedded Control Systems (7ENT1042), University of Hertfordshire
 *
 * This firmware demonstrates:
 *  - 3x IR sensors for line tracking
 *  - PID based steering
 *  - Ultrasonic obstacle detection (HC-SR04) on a servo
 *  - Differential drive control
 */

// ----------------------- IR SENSOR CONFIG -----------------------

AnalogIn IRLeftSensor(A1);
AnalogIn IRCentreSensor(A2);
AnalogIn IRRightSensor(A3);

float irLeft = 0.0f;
float irCentre = 0.0f;
float irRight = 0.0f;

bool IRLeftFire = false;
bool IRCentreFire = false;
bool IRRightFire = false;

const float IR_THRESHOLD = 0.5f;  // tune to surface

// -------------------- ULTRASONIC + SERVO CONFIG -----------------

HCSR04 UltraSound(D14, D15);   // Trig, Echo
Servo ServoMotor(PA_3);

const int US_SAMPLES = 5;
const float SERVO_FRONT = 0.5f;
const float OBSTACLE_STOP_CM = 10.0f;

// ------------------------- MOTOR CONFIG -------------------------

PwmOut LMotor_speed(D10);
PwmOut RMotor_speed(D9);
DigitalOut LMotor_forwarddirection(D8);
DigitalOut RMotor_forwarddirection(D7);
DigitalOut LMotor_backwardsdirection(D6);
DigitalOut RMotor_backwardsdirection(D5);

const float BASE_SPEED = 0.35f;
const float MAX_SPEED  = 0.7f;

// --------------------------- PID GAINS ---------------------------

float Kp = 0.6f;
float Ki = 0.0f;
float Kd = 0.12f;

float prev_error = 0.0f;
float integral   = 0.0f;

// ------------------------ HELPER FUNCTIONS -----------------------

float clamp(float x, float min_v, float max_v) {
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void set_motors(float left_pwm, float right_pwm) {
    LMotor_forwarddirection = 1;
    LMotor_backwardsdirection = 0;
    RMotor_forwarddirection = 1;
    RMotor_backwardsdirection = 0;

    LMotor_speed = clamp(left_pwm, 0.0f, 1.0f);
    RMotor_speed = clamp(right_pwm, 0.0f, 1.0f);
}

void stop_motors() {
    // active brake
    LMotor_forwarddirection = 1;
    LMotor_backwardsdirection = 1;
    RMotor_forwarddirection = 1;
    RMotor_backwardsdirection = 1;
    LMotor_speed = 0.0f;
    RMotor_speed = 0.0f;
}

// ------------------- IR SENSOR & LINE ERROR LOGIC ----------------

void readIR() {
    irLeft   = IRLeftSensor.read();
    irCentre = IRCentreSensor.read();
    irRight  = IRRightSensor.read();

    IRLeftFire   = (irLeft   < IR_THRESHOLD);
    IRCentreFire = (irCentre < IR_THRESHOLD);
    IRRightFire  = (irRight  < IR_THRESHOLD);
}

/*
 * Map sensors to position:
 *  Left  -> -1
 *  Centre->  0
 *  Right -> +1
 * If multiple see line, take average.
 */
float compute_line_error() {
    int active = 0;
    float sum = 0.0f;

    if (IRLeftFire)   { sum += -1.0f; active++; }
    if (IRCentreFire) { sum +=  0.0f; active++; }
    if (IRRightFire)  { sum +=  1.0f; active++; }

    if (active == 0) {
        // caller should treat as "lost"
        return 0.0f;
    }

    float position = sum / (float)active;
    float desired = 0.0f;
    return desired - position;
}

// ------------------- ULTRASONIC FRONT DISTANCE -------------------

float get_front_distance_cm() {
    ServoMotor.write(SERVO_FRONT);
    ThisThread::sleep_for(200ms);

    float sum = 0.0f;
    int valid = 0;

    for (int i = 0; i < US_SAMPLES; i++) {
        float d = UltraSound.distance();
        if (d > 0.0f && d < 400.0f) {
            sum += d;
            valid++;
        }
        ThisThread::sleep_for(50ms);
    }

    if (valid == 0) {
        return -1.0f;
    }

    float avg = sum / (float)valid;
    printf("[US] Front distance: %.2f cm\r\n", avg);
    return avg;
}

// ----------------------------- MAIN ------------------------------

int main() {
    printf("\r\n=== Line-Following Robot with PID & Ultrasonic Obstacle Avoidance ===\r\n");
    printf("Author : Mohitkumar Kori\r\n");
    printf("Module : Embedded Control Systems (7ENT1042)\r\n");

    ServoMotor.write(SERVO_FRONT);
    stop_motors();
    ThisThread::sleep_for(1s);

    while (true) {
        readIR();

        bool any_on_line = IRLeftFire || IRCentreFire || IRRightFire;
        float front_dist = get_front_distance_cm();
        bool obstacle_close = (front_dist > 0.0f && front_dist < OBSTACLE_STOP_CM);

        // Lost line -> stop
        if (!any_on_line) {
            printf("[IR] Line lost -> STOP\r\n");
            stop_motors();
            ThisThread::sleep_for(50ms);
            continue;
        }

        // Obstacle too close -> stop
        if (obstacle_close) {
            printf("[US] Obstacle detected < %.1f cm -> STOP\r\n", OBSTACLE_STOP_CM);
            stop_motors();
            ThisThread::sleep_for(50ms);
            continue;
        }

        // PID control
        float error = compute_line_error();
        integral += error;
        float derivative = error - prev_error;
        float control = (Kp * error) + (Ki * integral) + (Kd * derivative);
        prev_error = error;

        float left_speed  = BASE_SPEED - control;
        float right_speed = BASE_SPEED + control;

        left_speed  = clamp(left_speed,  0.0f, MAX_SPEED);
        right_speed = clamp(right_speed, 0.0f, MAX_SPEED);

        set_motors(left_speed, right_speed);

        printf("[IR] L=%.2f C=%.2f R=%.2f | LF=%d CF=%d RF=%d | err=%.2f | L=%.2f R=%.2f\r\n",
               irLeft, irCentre, irRight,
               IRLeftFire, IRCentreFire, IRRightFire,
               error, left_speed, right_speed);

        ThisThread::sleep_for(20ms);
    }
}
