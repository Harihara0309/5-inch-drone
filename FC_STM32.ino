#include <Arduino.h>

// Motor PWM pins (for STM32 Bluepill)
#define MOTOR_1_PIN PA8
#define MOTOR_2_PIN PA9
#define MOTOR_3_PIN PA10
#define MOTOR_4_PIN PA11

// Serial communication for HC12
#define HC12_SERIAL Serial3  // Using USART3 for HC12 communication

// PID constants
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

// Variables for PID and stabilization
float roll, pitch, yaw, throttle;
float roll_output, pitch_output, yaw_output;
float previous_roll_error = 0, previous_pitch_error = 0, previous_yaw_error = 0;
float roll_integral = 0, pitch_integral = 0, yaw_integral = 0;

// Motor PWM limits
#define PWM_MIN 1000
#define PWM_MAX 2000

void setup() {
  // Initialize Serial communication
  HC12_SERIAL.begin(9600);
  
  // Motor pins as output
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);
  pinMode(MOTOR_3_PIN, OUTPUT);
  pinMode(MOTOR_4_PIN, OUTPUT);

  // Ensure motors start at a safe minimum speed
  analogWrite(MOTOR_1_PIN, PWM_MIN);
  analogWrite(MOTOR_2_PIN, PWM_MIN);
  analogWrite(MOTOR_3_PIN, PWM_MIN);
  analogWrite(MOTOR_4_PIN, PWM_MIN);
}

void loop() {
  // Check for data from HC12
  if (HC12_SERIAL.available() > 0) {
    String radioData = HC12_SERIAL.readStringUntil('\n');
    
    // Parsing radio data
    if (radioData.length() > 0) {
      int commaIndex1 = radioData.indexOf(',');
      int commaIndex2 = radioData.indexOf(',', commaIndex1 + 1);
      int commaIndex3 = radioData.indexOf(',', commaIndex2 + 1);
      
      // Extract Throttle, Roll, Pitch, Yaw from the radio data
      throttle = radioData.substring(0, commaIndex1).toFloat();
      roll = radioData.substring(commaIndex1 + 1, commaIndex2).toFloat();
      pitch = radioData.substring(commaIndex2 + 1, commaIndex3).toFloat();
      yaw = radioData.substring(commaIndex3 + 1).toFloat();
      
      // Call the PID control function to get the new PWM values
      applyPIDControl();
    }
  }
}

void applyPIDControl() {
  // Compute PID for Roll, Pitch, and Yaw
  float roll_error = roll;
  float pitch_error = pitch;
  float yaw_error = yaw;

  // Roll PID
  roll_integral += roll_error;
  float roll_derivative = roll_error - previous_roll_error;
  roll_output = Kp * roll_error + Ki * roll_integral + Kd * roll_derivative;
  previous_roll_error = roll_error;

  // Pitch PID
  pitch_integral += pitch_error;
  float pitch_derivative = pitch_error - previous_pitch_error;
  pitch_output = Kp * pitch_error + Ki * pitch_integral + Kd * pitch_derivative;
  previous_pitch_error = pitch_error;

  // Yaw PID
  yaw_integral += yaw_error;
  float yaw_derivative = yaw_error - previous_yaw_error;
  yaw_output = Kp * yaw_error + Ki * yaw_integral + Kd * yaw_derivative;
  previous_yaw_error = yaw_error;

  // Constrain the PID output to PWM limits
  roll_output = constrain(roll_output, -1000, 1000);
  pitch_output = constrain(pitch_output, -1000, 1000);
  yaw_output = constrain(yaw_output, -1000, 1000);

  // Adjust motor speeds based on PID output
  int motor1_speed = throttle - roll_output + pitch_output + yaw_output;
  int motor2_speed = throttle + roll_output + pitch_output - yaw_output;
  int motor3_speed = throttle + roll_output - pitch_output + yaw_output;
  int motor4_speed = throttle - roll_output - pitch_output - yaw_output;

  // Constrain motor speeds to the PWM range
  motor1_speed = constrain(motor1_speed, PWM_MIN, PWM_MAX);
  motor2_speed = constrain(motor2_speed, PWM_MIN, PWM_MAX);
  motor3_speed = constrain(motor3_speed, PWM_MIN, PWM_MAX);
  motor4_speed = constrain(motor4_speed, PWM_MIN, PWM_MAX);

  // Apply motor speeds via PWM
  analogWrite(MOTOR_1_PIN, motor1_speed);
  analogWrite(MOTOR_2_PIN, motor2_speed);
  analogWrite(MOTOR_3_PIN, motor3_speed);
  analogWrite(MOTOR_4_PIN, motor4_speed);
}
