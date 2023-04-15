# include "lineSensor.h"
# include "motor.h"
# include "encoders.h"
# include "kinematics.h"

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

float ave_e1_spd;
float ave_e0_spd;

LineSensor lineSensors(NB_LS_PINS, LS_FLEFT_IN_PIN, LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, LS_FRIGHT_IN_PIN, EMIT);
Kinematics_c kinematics;
Motors_c motors(L_PWM_PIN, L_DIR_PIN, R_PWM_PIN, R_DIR_PIN);
PID_c spd_pid_left;
PID_c spd_pid_right
PID_c heading_pid;

unsigned long leftCentreAmbient;
unsigned long rightCentreAmbient;
unsigned long centreAmbient;
unsigned long leftCentreCurrent;
unsigned long rightCentreCurrent;

bool initialHeadingState = true;
bool turnToShadowEdgeState = false;
bool shadowFollowClockwiseState = false;
bool shadowFollowAnticlockwiseState = false;

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

  motors.setMotorPower(15, 15);

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

    leftCentreCurrent = sensor_read[1] + sensor_read[2];
    rightCentreCurrent = sensor_read[3] + sensor_read[2];

    if (initialHeadingState) {
      if (leftCentreCurrent > leftCentreAmbient*2 || rightCentreCurrent > rightCentreAmbient*2) {
        motors.setMotorPower(0,0);
        Serial.println(leftCentreAmbient);
        Serial.println(leftCentreCurrent);
        centreAmbient = sensor_read[2];
        delay(500);
        if (leftCentreCurrent/leftCentreAmbient > rightCentreCurrent/rightCentreAmbient) {
          motors.setMotorPower(20,0);
          turnToShadowEdgeState = true;
          initialHeadingState = false;
        } else {
          motors.setMotorPower(0,20);
          turnToShadowEdgeState = true;
          initialHeadingState = false;
        }
      }
    }
    if (turnToShadowEdgeState) {
      if (sensor_read[2]*1.5 < centreAmbient) {
        turnToShadowEdgeState = false;
        if (sensor_read[1] > sensor_read[3]) {
          shadowFollowAnticlockwiseState = true;
        } else {
          shadowFollowClockwiseState = true;
        }
      }
    }

    if (shadowFollowAnticlockwiseState) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( (15 + turn_pwm), (15 - turn_pwm));
    }

    if (shadowFollowClockwiseState) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( (15 + turn_pwm), (15 - turn_pwm));
    }

    ls_ts = millis();
  }
 
 
  Serial.print("\n");
  

}
