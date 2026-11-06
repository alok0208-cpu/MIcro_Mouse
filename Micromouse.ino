#include <Wire.h>
#include <MPU6050_light.h>

// Motors
#define PWML 19 // L=A
#define LIN1 4
#define LIN2 13
#define LENC_A 32
#define LENC_B 33

#define PWMR 27 // R=B
#define RIN1 18
#define RIN2 5
#define RENC_A 35
#define RENC_B 34
#define STBY 23

MPU6050 mpu(Wire);

volatile long encoderLCount = 0;
volatile long encoderRCount = 0;

// --- PID Tuning Parameters ---
// Tune these if the robot wobbles or under/overshoots
float Kp_straight = 2.0; 
float Ki_straight = 0.0; 
float Kd_straight = 0.5;

float Kp_turn = 3.5; 
float Ki_turn = 0.01; 
float Kd_turn = 1.0;

bool testComplete = false; // Ensures the square test only runs once

// Encoder functions
void IRAM_ATTR readLEncoder() {
    int a = digitalRead(LENC_A);
    int b = digitalRead(LENC_B);
    if (a == b) {
        encoderLCount++;
    } else {
        encoderLCount--;
    }
}

void IRAM_ATTR readREncoder() {
    int a = digitalRead(RENC_A);
    int b = digitalRead(RENC_B);
    if (a == b) {
        encoderRCount++;
    } else {
        encoderRCount--;
    }
}

// Motor functions
void motorForward(int speed, char motor) {
    if (motor == 'L') {
        digitalWrite(LIN1, HIGH);
        digitalWrite(LIN2, LOW);
        analogWrite(PWML, speed);
    } else {
        digitalWrite(RIN1, HIGH);
        digitalWrite(RIN2, LOW);
        analogWrite(PWMR, speed);
    }
}

void motorBackward(int speed, char motor) {
    if (motor == 'L') {
        digitalWrite(LIN1, LOW);
        digitalWrite(LIN2, HIGH);
        analogWrite(PWML, speed);
    } else {
        digitalWrite(RIN1, LOW);
        digitalWrite(RIN2, HIGH);
        analogWrite(PWMR, speed);
    }
}

void motorStop(char motor) {
    if (motor == 'L') {
        digitalWrite(LIN1, LOW);
        digitalWrite(LIN2, LOW);
        analogWrite(PWML, 0);
    } else {
        digitalWrite(RIN1, LOW);
        digitalWrite(RIN2, LOW);
        analogWrite(PWMR, 0);
    }
}

// Helper function to handle positive/negative speeds automatically
void setMotorSpeed(char motor, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        motorForward(speed, motor);
    } else {
        motorBackward(-speed, motor);
    }
}

void setBothMotors(int left_speed, int right_speed) {
    setMotorSpeed('L', left_speed);
    setMotorSpeed('R', right_speed);
}

// --- PID Control Functions ---

void driveStraight(long target_distance_ticks, float target_angle, int base_pwm) {
    long start_left_ticks = abs(encoderLCount);
    long start_right_ticks = abs(encoderRCount);
    
    float error = 0, prev_error = 0, integral = 0;
    long current_distance = 0;

    while (current_distance < target_distance_ticks) {
        mpu.update(); // Crucial for updating the angle continuously
        float current_angle = mpu.getAngleZ(); 
        
        error = target_angle - current_angle;
        integral += error;
        float derivative = error - prev_error;
        
        float correction = (Kp_straight * error) + (Ki_straight * integral) + (Kd_straight * derivative);
        
        // Positive error means we drifted right (need to turn left). 
        // Left motor slows down, Right motor speeds up.
        int left_pwm = base_pwm - correction;
        int right_pwm = base_pwm + correction;
        
        setBothMotors(left_pwm, right_pwm);
        
        long left_travel = abs(abs(encoderLCount) - start_left_ticks);
        long right_travel = abs(abs(encoderRCount) - start_right_ticks);
        current_distance = (left_travel + right_travel) / 2;
        
        prev_error = error;
        delay(10); // Prevent watchdog timeout and stabilize derivative
    }
    
    motorStop('L');
    motorStop('R');
}

void turnToAngle(float target_angle) {
    mpu.update();
    float error = target_angle - mpu.getAngleZ();
    float prev_error = error, integral = 0;
    
    // Loop until we are within 1.5 degrees of the target
    while (abs(error) > 1.5) {
        mpu.update();
        float current_angle = mpu.getAngleZ();
        error = target_angle - current_angle;
        
        integral += error;
        float derivative = error - prev_error;
        
        float turn_speed = (Kp_turn * error) + (Ki_turn * integral) + (Kd_turn * derivative);
        
        // Positive turn_speed (turning left): Left motor backward, Right motor forward
        setBothMotors(-turn_speed, turn_speed);
        
        prev_error = error;
        delay(10);
    }
    
    motorStop('L');
    motorStop('R');
    delay(300); // Settle chassis before next move
}

void executeSquareTest(long side_length_ticks, int speed) {
    mpu.update();
    float current_heading = mpu.getAngleZ();
    
    for (int i = 0; i < 4; i++) {
        // Drive one side of the square
        driveStraight(side_length_ticks, current_heading, speed);
        delay(500); 
        
        // Turn right by 90 degrees (adjust to +90.0 for left turns)
        current_heading -= 90.0; 
        turnToAngle(current_heading);
        delay(500); 
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(PWML, OUTPUT);
    pinMode(LIN1, OUTPUT);
    pinMode(LIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(PWMR, OUTPUT);
    pinMode(RIN1, OUTPUT);
    pinMode(RIN2, OUTPUT);

    digitalWrite(STBY, HIGH);
    Serial.println("TB6612FNG Initialized");

    pinMode(LENC_A, INPUT_PULLUP);
    pinMode(LENC_B, INPUT_PULLUP);
    pinMode(RENC_A, INPUT);
    pinMode(RENC_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(LENC_A), readLEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RENC_A), readREncoder, CHANGE);

    Serial.println("Encoder Initialized");

    Wire.begin(); 

    byte status = mpu.begin();
    Serial.print("MPU6050 status: ");
    Serial.println(status); 

    Serial.println("Keep mouse still for calibration...");
    delay(1000);
    mpu.calcOffsets(); 
    Serial.println("Calibration done. Starting square test in 3 seconds...");
    delay(3000);
}

void loop() {
    // Run the square test once
    if (!testComplete) {
        // Adjust 1000 ticks to match your physical encoder resolution for 1 side of the square
        executeSquareTest(1000, 150); 
        testComplete = true;
        Serial.println("Test Complete. Entering telemetry mode.");
    }

    static unsigned long previousTime = 0;

    // Fixed the millis() syntax error here
    if (millis() - previousTime >= 500) {
        previousTime = millis();

        noInterrupts();
        long Lcount = encoderLCount;
        long Rcount = encoderRCount;
        interrupts();

        Serial.print("Encoder Count: ");
        Serial.print(Lcount);
        Serial.print(" ");
        Serial.println(Rcount);

        mpu.update();
        Serial.print("Yaw Angle: ");
        Serial.println(mpu.getAngleZ());
    }
}