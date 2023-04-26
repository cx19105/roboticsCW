# include "motor.h"
# include "encoders.h"
# include "kinematics.h"
# include "pid.h"

# define LS_LEFT_IN_PIN A0
# define LS_CENTRE_IN_PIN A2
# define LS_RIGHT_IN_PIN A3
# define LS_FLEFT_IN_PIN A11
# define LS_FRIGHT_IN_PIN A4
# define EMIT 11
# define NB_LS_PINS 5

# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define FWD LOW
# define REV HIGH

# define LINE_SENSOR_UPDATE 100
# define MOTOR_UPDATE 2000
# define KINEMATICS_UPDATE 20

# define L_LS_THRESHOLD 1400
# define C_LS_THRESHOLD 1000
# define R_LS_THRESHOLD 1400
# define FLR_LS_THRESHOLD 1400

# define L_LS_MAX 2000
# define R_LS_MAX 2000

# define L_LS_MIN 800
# define R_LS_MIN 800

unsigned long current_ts = millis();


Motors_c motors(L_PWM_PIN, L_DIR_PIN, R_PWM_PIN, R_DIR_PIN);
Kinematics_c kinematics;
PID_c spd_pid_left;
PID_c spd_pid_right;
PID_c heading_pid;

unsigned long update_ts;

float ave_e1_spd;
float ave_e0_spd;

long count_e1_last;

bool rotateTrue = false;
bool forwardTrue = true;

void setup() {
  setupEncoder0();
  setupEncoder1();

  spd_pid_left.initialise(100, 0.15, 100);
  spd_pid_right.initialise(100, 0.15, 100);
  heading_pid.initialise(10, 0.0001, 0);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  update_ts = millis();
  count_e1_last = count_e1;

  motors.setMotorPower(0, 0, 0);

  ave_e1_spd = 0;
  ave_e0_spd = 0;
  
  //spd_pid_left.reset()

}

void loop() {

  unsigned long elapsed_t;
  
  elapsed_t = millis() - update_ts;
  //motors.setMotorPower(25, 25, 0);
  
  if (elapsed_t > KINEMATICS_UPDATE){
    update_ts = millis();
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
    float demandTheta = 3.14;
    float demandDist = 300;
    float pwml;
    float pwmr;
    float pwmh;

    float distance = sqrt(pow(kinematics.x, 2) + pow(kinematics.y, 2));
    

    
    
    pwmh = heading_pid.update(kinematics.theta, demandTheta, elapsed_t);

    if (forwardTrue) {
      demand = min(0.4, 0.4*((demandDist-distance)/100));
      Serial.print(demand);
      pwml = spd_pid_left.update(ave_e1_spd, demand, elapsed_t);
      pwmr = spd_pid_right.update(ave_e0_spd, demand, elapsed_t);
      Serial.print(pwml);
      Serial.print(",");
      Serial.print(pwmr);
      if (pwml > 30) {
        pwml = 30;
      } if (pwmr > 30) {
        pwmr = 30;
      }
      motors.setMotorPower(pwml, pwmr, 0);
      if (abs(demandDist - distance) < 30) {
        forwardTrue = false;
        motors.setMotorPower(0, 0, 0);
      }
    }
    
    
    if (rotateTrue) {
      motors.setMotorRotate(pwmh);
      
      if (abs(kinematics.theta - demandTheta) < 0.03){
        
        rotateTrue = false;
        motors.setMotorPower(0, 0, 0);
      }
    }
    //motors.setMotorPower(pwml, pwmr, 0);
    
    Serial.print(",");
    Serial.print(kinematics.theta);
    Serial.print(",");
    Serial.print(kinematics.x);
    Serial.print(",");
    Serial.print(kinematics.y);
    Serial.print("\n");
    
  
    

    count_e0 = 0;
    count_e1 = 0;

  }
  
  
  
}
