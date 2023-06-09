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
# define KINEMATICS_UPDATE 20

# define L_LS_THRESHOLD 1400
# define C_LS_THRESHOLD 1000
# define R_LS_THRESHOLD 1400

PID_c spd_pid_right;
PID_c spd_pid_left;
PID_c heading_pid;

float ave_e1_spd;
float ave_e0_spd;

long count_e1_last;

LineSensor lineSensors(NB_LS_PINS, LS_FLEFT_IN_PIN, LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, LS_FRIGHT_IN_PIN, EMIT);
Kinematics_c kinematics;
Motors_c motors(L_PWM_PIN, L_DIR_PIN, R_PWM_PIN, R_DIR_PIN);

unsigned long leftCentreAmbient;
unsigned long rightCentreAmbient;
unsigned long leftCentreCurrent;
unsigned long rightCentreCurrent;

bool initialHeadingState = false;
bool rotateTrue = true;
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

  spd_pid_left.initialise(100, 0.15, 100);
  spd_pid_right.initialise(100, 0.15, 100);
  heading_pid.initialise(10, 0.0001, 0);

  float ls_ts = millis();
  count_e1_last = count_e1;

  motors.setMotorPower(0, 0, 0);

  ave_e1_spd = 0;
  ave_e0_spd = 0;
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
    ls_ts = millis();
    count_e1 = -count_e1;
    count_e0 = -count_e0;
    kinematics.update(count_e1, count_e0);
    
    float e1_speed;
    float e0_speed;
    
    e1_speed = (float)count_e1;
    e1_speed /= (float)elapsed_t;

    e0_speed = (float)count_e0;
    e0_speed /= (float)elapsed_t;

    ave_e1_spd = (ave_e1_spd * 0.7) + (e1_speed*0.3);
    ave_e0_spd = (ave_e0_spd * 0.7) + (e0_speed*0.3);

    float demand = 0.4;
    float demandTheta = -1.57;
    float pwml;
    float pwmr;
    float pwmh;

    pwmh = heading_pid.update(kinematics.theta, demandTheta, elapsed_t);

    if (rotateTrue) {
      motors.setMotorRotate(pwmh);
      
      if (abs(kinematics.theta - demandTheta) < 0.13){
        Serial.println("COMPLETE");
        rotateTrue = false;
        initialHeadingState = true;
        motors.setMotorPower(0, 0, 0);
      }
    }

    unsigned long sensor_read[NB_LS_PINS];
    lineSensors.readLineSensor(sensor_read);
    
    leftCentreCurrent = sensor_read[1] + sensor_read[2];
    rightCentreCurrent = sensor_read[3] + sensor_read[2];

    if (initialHeadingState) {
      pwml = spd_pid_left.update(ave_e1_spd, demand, elapsed_t);
      pwmr = spd_pid_right.update(ave_e0_spd, demand, elapsed_t);
      if (pwml > 30) {
        pwml = 30;
      } if (pwmr > 30) {
        pwmr = 30;
      }
      motors.setMotorPower(pwml, pwmr, 0);
      if (leftCentreCurrent > leftCentreAmbient*3) {
        //motors.setMotorPower(0,0,0);
        //delay(500);
        xOne = kinematics.x;
        Serial.println(xOne);
        Serial.println(rightCentreCurrent);
        Serial.println(rightCentreAmbient);
        shortestPathClockwise = false;
        findAngleState = true;
        initialHeadingState = false;
        //motors.setMotorPower(pwml, pwmr, 0);
      } else if (rightCentreCurrent > rightCentreAmbient*3) {
        //motors.setMotorPower(0,0,0);
        //delay(500);
        xOne = kinematics.x;
        shortestPathClockwise = true;
        findAngleState = true;
        initialHeadingState = false;
        //motors.setMotorPower(pwml, pwmr, 0);
      }
    }
    
    if (findAngleState && shortestPathClockwise) {
      if (leftCentreCurrent > leftCentreAmbient*3) {
        x = kinematics.x - xOne;
        demandTheta = -atan(sensorWidth/x);
        motors.setMotorPower(0,0,0);
        delay(500);
        findAngleState = false;
        turnByAngleState = true;
      }
    }
    if (findAngleState && !shortestPathClockwise) {
      if (rightCentreCurrent > rightCentreAmbient*3) {
        x = kinematics.x - xOne;
        demandTheta = atan(sensorWidth/x);
        Serial.println(kinematics.x);
        Serial.println(demandTheta);
        motors.setMotorPower(0,0,0);
        delay(500);        
        findAngleState = false;
        turnByAngleState = true;
      }
    }
    pwmh = heading_pid.update(kinematics.theta, demandTheta, elapsed_t);
    if (turnByAngleState) {
      motors.setMotorRotate(pwmh);
      
      if (abs(kinematics.theta - demandTheta) < 0.03){
        
        turnByAngleState = false;
        motors.setMotorPower(0, 0, 0);
      
        // on completion...
        if (kinematics.x < startingDistance) {
          shadowFollowState = true;
          y = kinematics.y;
        }
        else {
          // Move briefly forward to finish
          motors.setMotorPower(pwml, pwmr, 0);
          delay(1000);
        }
      }
    }

    if (shadowFollowState && shortestPathClockwise) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( (pwml + turn_pwm), (pwmr - turn_pwm), 0);
      if (kinematics.y > y - 0.5) {
        shadowFollowState = false;
        turnByAngleState = true;
      }
    }
    if (shadowFollowState && !shortestPathClockwise) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( (pwml + turn_pwm), (pwmr - turn_pwm), 0);
      if (kinematics.y < y + 0.5) {
        shadowFollowState = false;
        turnByAngleState = true;
      }
    }

    count_e0 = 0;
    count_e1 = 0;
  }
  

}
