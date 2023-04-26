# include "motor.h"
# include "encoders.h"
# include "kinematics.h"
# include "pid.h"
# include "lineSensor.h"
# include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

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
# define READINGS_UPDATE 50

# define L_LS_THRESHOLD 1400
# define C_LS_THRESHOLD 1000
# define R_LS_THRESHOLD 1400
# define FLR_LS_THRESHOLD 1400

# define L_LS_MAX 2000
# define R_LS_MAX 2000

# define L_LS_MIN 800
# define R_LS_MIN 800

unsigned long current_ts = millis();

LineSensor lineSensors(NB_LS_PINS, LS_FLEFT_IN_PIN, LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, LS_FRIGHT_IN_PIN, EMIT);
Motors_c motors(L_PWM_PIN, L_DIR_PIN, R_PWM_PIN, R_DIR_PIN);
Kinematics_c kinematics;
PID_c spd_pid_left;
PID_c spd_pid_right;
PID_c heading_pid;

unsigned long update_ts;

float ave_e1_spd;
float ave_e0_spd;

long count_e1_last;
/*
bool rotateTrue = true;
bool forwardTrue = false;
*/
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
float sensorWidth = 16; //mm
float x;
float y;
float demand = 0.3;
float demandTheta = -1.57;
float errorTheta = 1;
float pwml;
float pwmr;
float pwmh;

float results[5][100];
unsigned long readings_ts;
unsigned long finishTime;
int j;

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

  float ambientLight[3];
  lineSensors.getAmbient(ambientLight);
  unsigned long leftAmbient = ambientLight[0];
  unsigned long rightAmbient = ambientLight[1];
  unsigned long centreAmbient = ambientLight[2];
  //leftCentreAmbient = leftAmbient + centreAmbient;
  //rightCentreAmbient = rightAmbient + centreAmbient;
  leftCentreAmbient = leftAmbient;
  rightCentreAmbient = rightAmbient;
  
  //spd_pid_left.reset()
  readings_ts = millis();
}

float getLineError(unsigned long sensor_read[7]) {
  float w_left = sensor_read[1] + 0.5*sensor_read[2];
  float w_right = sensor_read[3] + 0.5*sensor_read[2];
  float e_line = (w_right - w_left)/(w_right + w_left);
  return e_line;
}

float turn_pwm;
float gain = 30;

void reportResultsOverSerial() {

  // Print millis for debug so we can 
  // validate this is working in real
  // time, and not glitched somehow
  if( SERIAL_ACTIVE ) Serial.print(millis());
  if( SERIAL_ACTIVE ) Serial.print(", ");
  if( SERIAL_ACTIVE ) Serial.println( finishTime );
  delay(1);


  // Loop through array to print all 
  // results collected
  int i,j;  
  for( j = 0; j < 100; j++ ) {   // row

    // Comma seperated values, to 2 decimal places
    if( SERIAL_ACTIVE ) Serial.print( results[0][j], 0 );
    delay(1);
    if( SERIAL_ACTIVE ) Serial.print( "," );
    if( SERIAL_ACTIVE ) Serial.print( results[1][j], 0 );
    delay(1);
    if( SERIAL_ACTIVE ) Serial.print( "," );
    if( SERIAL_ACTIVE ) Serial.print( results[2][j], 0 );
    delay(1);
    if( SERIAL_ACTIVE ) Serial.print( "," );
    if( SERIAL_ACTIVE ) Serial.print( results[3][j], 2 );
    delay(1);
    if( SERIAL_ACTIVE ) Serial.print( "," );
    if( SERIAL_ACTIVE ) Serial.print( results[4][j], 2 );
    delay(1);
    if( SERIAL_ACTIVE ) Serial.print( "\n" ); // new row
  }

  Serial.println(demandTheta);
  if( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n\n" ); 

}

void loop() {

  unsigned long elapsed_t;
  unsigned long sensor_read[NB_LS_PINS];
  
  elapsed_t = millis() - update_ts;
  
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

    lineSensors.readLineSensor(sensor_read);
    
    //leftCentreCurrent = sensor_read[1] + sensor_read[2];
    //rightCentreCurrent = sensor_read[3] + sensor_read[2];
    leftCentreCurrent = sensor_read[1];
    rightCentreCurrent = sensor_read[3];

    pwml = spd_pid_left.update(ave_e1_spd, demand, elapsed_t);
    pwmr = spd_pid_right.update(ave_e0_spd, demand, elapsed_t);
    if (pwml > 30) {
      pwml = 30;
    } if (pwmr > 30) {
      pwmr = 30;
    }

    if (initialHeadingState) {
      motors.setMotorPower(pwml, pwmr, 0);
      if (leftCentreCurrent > leftCentreAmbient*3) {
        motors.setMotorPower(0,0,0);
        delay(500);
        x = kinematics.x;
        shortestPathClockwise = false;
        findAngleState = true;
        initialHeadingState = false;
        motors.setMotorPower(pwml, pwmr, 0);
      } else if (rightCentreCurrent > rightCentreAmbient*3) {
        motors.setMotorPower(0,0,0);
        delay(500);
        x = kinematics.x;
        shortestPathClockwise = true;
        findAngleState = true;
        initialHeadingState = false;
        motors.setMotorPower(pwml, pwmr, 0);
      }
    }

    if (findAngleState && shortestPathClockwise) {
      if (leftCentreCurrent > leftCentreAmbient*3) {
        x = kinematics.x - x;
        //delay(20);
        motors.setMotorPower(0,0,0);
        delay(500);
        demandTheta = -atan(sensorWidth/x)*errorTheta;
        findAngleState = false;
        turnByAngleState = true;
      }
    }
    if (findAngleState && !shortestPathClockwise) {
      if (rightCentreCurrent > rightCentreAmbient*3) {
        x = kinematics.x - x;
        //delay(20);
        motors.setMotorPower(0,0,0);
        delay(500); 
        demandTheta = atan(sensorWidth/x)*errorTheta;   
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
        delay(500);
      
        // on completion...
        if (kinematics.x < startingDistance - 20) {
          shadowFollowState = true;
          y = kinematics.y;
        }
        else {
          // Move briefly forward to finish
          motors.setMotorPower(pwml, pwmr, 0);
          delay(250);
          motors.setMotorPower(0,0,0);
          for( j;j < 100;j++) {
            results[0][j] = sensor_read[0];
            results[1][j] = sensor_read[1];
            results[2][j] = sensor_read[2];
            results[3][j] = kinematics.x;
            results[4][j] = kinematics.y;
          }
          delay(10000);
          reportResultsOverSerial();

          // Delay, we don't need to do this
          // excessively fast.
          delay(100000);
        }
      }
    }
  
    if (shadowFollowState && shortestPathClockwise) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( pwml + turn_pwm, pwmr - turn_pwm, 0);
      if (kinematics.y > y + 0.5) {
        delay(500);
        kinematics.theta = 0;
        shadowFollowState = false;
        turnByAngleState = true;
      }
    }
    if (shadowFollowState && !shortestPathClockwise) {
      float e_line = getLineError(sensor_read);
      turn_pwm = e_line*gain;
      motors.setMotorPower( pwml - turn_pwm, pwmr + turn_pwm, 0);
      if (kinematics.y < y - 0.5) {
        delay(500);
        kinematics.theta = 0;
        shadowFollowState = false;
        turnByAngleState = true;
      }
    }

    count_e0 = 0;
    count_e1 = 0;

  }

  if (millis() - readings_ts > READINGS_UPDATE && j < 100) {
      results[0][j] = sensor_read[0];
      results[1][j] = sensor_read[1];
      results[2][j] = sensor_read[2];
      results[3][j] = kinematics.x;
      results[4][j] = kinematics.y;
      j++;
      if (j == 100) {
        finishTime = millis();
      }
      readings_ts = millis();
    }
  
  
  
}
