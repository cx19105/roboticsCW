# include "lineSensor.h"
# include "motor.h"
# include "encoders.h"
# include "kinematics.h"
# include "pid.h"

# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define LS_LEFT_IN_PIN A0
# define LS_CENTRE_IN_PIN A2
# define LS_RIGHT_IN_PIN A3
# define LS_FLEFT_IN_PIN A11
# define LS_FRIGHT_IN_PIN A4
# define LS_LBUMP_IN_PIN 4
# define LS_RBUMP_IN_PIN 5
# define EMIT 11
# define NB_LS_PINS 7

# define FWD LOW
# define REV HIGH

# define LINE_SENSOR_UPDATE 100
# define MOTOR_UPDATE 2000
# define KINEMATICS_UPDATE 100

# define L_LS_THRESHOLD 1200
# define C_LS_THRESHOLD 1000
# define R_LS_THRESHOLD 1200

PID_c spd_pid_right;
PID_c spd_pid_left;
PID_c heading_pid;

float ave_e1_spd;
float ave_e0_spd;

LineSensor lineSensors(NB_LS_PINS, LS_FLEFT_IN_PIN, LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, LS_FRIGHT_IN_PIN, EMIT);
Kinematics_c kinematics;
Motors_c motors(L_PWM_PIN, L_DIR_PIN, R_PWM_PIN, R_DIR_PIN);

unsigned long leftCentreAmbient;
unsigned long rightCentreAmbient;
unsigned long leftCentreCurrent;
unsigned long rightCentreCurrent;

bool initialHeadingState = true;
bool findAngleState = false;
bool turnByAngleState = false;
bool shadowFollowState = false;
bool shortestPathClockwise = true;

float startingDistance = 200; //mm
float sensorWidth = 20; //mm
float xOne;
float x;
float theta;
float y;

void setup() {
  
  setupEncoder0();
  setupEncoder1();
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  float ambientLight[3];

  lineSensors.getAmbient(ambientLight);
  Serial.println(ambientLight[0]);

  unsigned long leftAmbient = ambientLight[0];
  unsigned long rightAmbient = ambientLight[1];
  unsigned long centreAmbient = ambientLight[2];

  leftCentreAmbient = leftAmbient + centreAmbient;
  rightCentreAmbient = rightAmbient + centreAmbient;
  Serial.println(leftCentreAmbient);
  delay(2000);

  motors.setMotorPower(15,15);

  spd_pid_left.initialise(100, 0.15, 100);
  spd_pid_right.initialise(100, 0.15, 100);
  heading_pid.initialise(0.3, 0.000001, 1);
}

float getLineError(unsigned long sensor_read[7]) {
  float w_left = sensor_read[1] + 0.5*sensor_read[2];
  float w_right = sensor_read[3] + 0.5*sensor_read[2];
  float e_line = (w_right - w_left)/(w_right + w_left);
  return e_line;
}

float turn_pwm;
float gain = 7.5;

void loop() {
  unsigned long current_ts;
  unsigned long elapsed_t;

  unsigned long ls_ts;
  unsigned long motors_ts;
  
 
  current_ts = millis();

  elapsed_t = current_ts - ls_ts;

  if(elapsed_t > KINEMATICS_UPDATE) {
    kinematics.update(-count_e1, -count_e0);
    
    float e1_speed;
    float e0_speed;
    
    e1_speed = -(float)count_e1;
    e1_speed /= (float)elapsed_t;

    e0_speed = -(float)count_e0;
    e0_speed /= (float)elapsed_t;

    ave_e1_spd = (ave_e1_spd * 0.7) + (e1_speed*0.3);
    ave_e0_spd = (ave_e0_spd * 0.7) + (e0_speed*0.3);
    count_e0 = 0;
    count_e1 = 0;

    unsigned long sensor_read[NB_LS_PINS];
    lineSensors.readLineSensor(sensor_read);
    
    Serial.print(sensor_read[0]);
    Serial.print(",");
    Serial.print(sensor_read[1]);
    Serial.print(",");
    Serial.print(sensor_read[2]);
    Serial.print(",");
    Serial.print(sensor_read[3]);
    Serial.print(",");
    Serial.print(sensor_read[4]);
    Serial.print(",");
    Serial.print(sensor_read[5]);
    Serial.print(",");
    Serial.print(sensor_read[6]);
    Serial.print(",");
    Serial.print(kinematics.x);
    Serial.print("\n");
    
    leftCentreCurrent = sensor_read[1] + sensor_read[2];
    rightCentreCurrent = sensor_read[3] + sensor_read[2];

    if (initialHeadingState) {
      float pwml;
      float pwmr;
      pwml = spd_pid_left.update(ave_e1_spd, 15, elapsed_t);
      pwmr = spd_pid_right.update(ave_e0_spd, 15, elapsed_t);
      motors.setMotorPower(15,15);
      if (leftCentreCurrent > leftCentreAmbient*2) {
        motors.setMotorPower(0,0);
        delay(500);
        xOne = kinematics.x;
        shortestPathClockwise = false;
        findAngleState = true;
        initialHeadingState = false;
        motors.setMotorPower(15,15);
      } else if (rightCentreCurrent > rightCentreAmbient*2) {
        motors.setMotorPower(0,0);
        delay(500);
        xOne = kinematics.x;
        shortestPathClockwise = true;
        findAngleState = true;
        initialHeadingState = false;
        motors.setMotorPower(15,15);
      }
    }
    
    if (findAngleState && shortestPathClockwise) {
      if (leftCentreCurrent > leftCentreAmbient*2) {
        motors.setMotorPower(0,0);
        delay(500);
        x = kinematics.x - xOne;
        theta = atan(sensorWidth/x); // + or - for clockwise??
        findAngleState = false;
        turnByAngleState = true;
      }
    }
    if (findAngleState && !shortestPathClockwise) {
      if (rightCentreCurrent > rightCentreAmbient*2) {
        motors.setMotorPower(0,0);
        delay(500);
        x = kinematics.x - xOne;
        theta = -atan(sensorWidth/x);
        findAngleState = false;
        turnByAngleState = true;
      }
    }
    
    if (turnByAngleState) {
      // Code needed from PID when ready
      
      // on completion...
      if (kinematics.x < startingDistance) {
        shadowFollowState = true;
        turnByAngleState = false;
        y = kinematics.y;
      }
      else {
        // Move briefly forward to finish
        motors.setMotorPower(15,15);
        delay(1000);
      }
    }

    if (shadowFollowState && shortestPathClockwise) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( (15 + turn_pwm), (15 - turn_pwm));
      if (kinematics.y > y) {
        shadowFollowState = false;
        turnByAngleState = true;
      }
    }
    if (shadowFollowState && !shortestPathClockwise) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( (15 + turn_pwm), (15 - turn_pwm));
      if (kinematics.y < y) {
        shadowFollowState = false;
        turnByAngleState = true;
      }
    }

    ls_ts = millis();
  }
  

}
