# include "lineSensor.h"
# include "motor.h"
# include "encoders.h"
# include "kinematics.h"
# include "pid.h"
# include <EEPROM.h>
# include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

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

# define LINE_SENSOR_UPDATE 400
# define MOTOR_UPDATE 2000
# define KINEMATICS_UPDATE 500

# define L_LS_THRESHOLD 1200
# define C_LS_THRESHOLD 1000
# define R_LS_THRESHOLD 1200
/*
# define UPDATE_MS   200      // period
unsigned long eep_update_ts;  // timestamp
int eeprom_address;
*/
float cal = -0.9;
float xMultiplier = 1.35;
float startingDistance = 200-45; //mm

PID_c spd_pid_right;
PID_c spd_pid_left;
PID_c heading_pid;

float ave_e1_spd;
float ave_e0_spd;

LineSensor lineSensors(NB_LS_PINS, LS_FLEFT_IN_PIN, LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, LS_FRIGHT_IN_PIN, EMIT);
Kinematics_c kinematics;
Motors_c motors(L_PWM_PIN, L_DIR_PIN, R_PWM_PIN, R_DIR_PIN);

float results[4][50];
unsigned long update_ts;

// State machine
# define STATE_RUNNING_EXPERIMENT  0
# define STATE_FINISHED_EXPERIMENT 1
int state;

void setup() {
  
  setupEncoder0();
  setupEncoder1();
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  //for (int i = 0 ; i < EEPROM.length() ; i++) {
  //  EEPROM.write(i, 0);
  //}

  motors.setMotorPower(18 + cal,18 - cal);

  spd_pid_left.initialise(100, 0.15, 100);
  spd_pid_right.initialise(100, 0.15, 100);
  heading_pid.initialise(0.3, 0.000001, 1);

  //eeprom_address = 0;

  // Start robot in running experiment state.
  state = STATE_RUNNING_EXPERIMENT;

  // Record start timestamp
  update_ts = millis();
}

void reportResultsOverSerial() {

  // Print millis for debug so we can 
  // validate this is working in real
  // time, and not glitched somehow
  if( SERIAL_ACTIVE ) Serial.print( "Time(ms): " );
  if( SERIAL_ACTIVE ) Serial.println( millis() );
  delay(1);


  // Loop through array to print all 
  // results collected
  int i,j;  
  for( j = 0; j < 50; j++ ) {   // row

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
    if( SERIAL_ACTIVE ) Serial.print( results[3][j], 0 );
    delay(1);
    if( SERIAL_ACTIVE ) Serial.print( "," );
    delay(1);
    if( SERIAL_ACTIVE ) Serial.print( "\n" ); // new row
  }

  if( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n\n" ); 

}

void loop() {

  if( state == STATE_RUNNING_EXPERIMENT ) {
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
      
      if (kinematics.x*xMultiplier > startingDistance - 40) {
        motors.setMotorPower(-(18+cal),-(18-cal));
      } else if (kinematics.x < 0) {
        motors.setMotorPower(18+cal,18-cal);
      }
      /*
      if( elapsed_t > UPDATE_MS ) {
    
        // Save "99" to current eeprom address
        // Could be a different value.
        EEPROM.write( eeprom_address, millis());
        eeprom_address++;
        EEPROM.write( eeprom_address, elapsed_t);
        eeprom_address++;
        EEPROM.write( eeprom_address, sensor_read[2]);
        eeprom_address++;
        EEPROM.write( eeprom_address, kinematics.x);
        // Advance address for next time.
        eeprom_address++;
      }
      */
      int j;
      for( j=0;j < 50;j++) {
        results[0][j] = sensor_read[0];
        results[1][j] = sensor_read[1];
        results[2][j] = sensor_read[2];
        results[3][j] = kinematics.x;
      }
  
      //if( eeprom_address > EEPROM.length() - 824 ) {
      //  motors.setMotorPower(0,0);
      //  delay(10000000);
      //}
    
        ls_ts = millis();
        //eep_update_ts = millis();
      }

    if( millis() - update_ts > 40000 ) {
      update_ts = millis();

      // Transition to other state.
      state = STATE_FINISHED_EXPERIMENT;
    }
  } else if ( state == STATE_FINISHED_EXPERIMENT) {

    // No transition out of this state, so 
    // the robot will now be stuck in a 
    // motors off condition and report results
    // periodically.

    // Stop robot
    motors.setMotorPower(0,0);

    // Put LED on here

    delay(5000);

    / Turn LED off here

    // Print results - uses USB fix.
    reportResultsOverSerial();

    // Delay, we don't need to do this
    // excessively fast.
    delay(100000);
  
  }
}
