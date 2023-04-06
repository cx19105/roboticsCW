# include "lineSensor.h"
# include "encoders.h"
# include "kinematics.h"


# define LS_LEFT_IN_PIN A0
# define LS_CENTRE_IN_PIN A2
# define LS_RIGHT_IN_PIN A3

# define EMIT 11
# define NB_LS_PINS 3

# define LINE_SENSOR_UPDATE 100
# define KINEMATICS_UPDATE 100

float ave_e1_spd;
float ave_e0_spd;

LineSensor lineSensors(NB_LS_PINS, LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, EMIT);
Kinematics_c kinematics;

unsigned long leftCentreAmbient;
unsigned long rightCentreAmbient;

void setup() {
  setupEncoder0();
  setupEncoder1();
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  float ambientLight[3];

  lineSensors.getAmbient(ambientLight);
  Serial.print(ambientLight[0]);

  unsigned long leftAmbient = ambientLight[0];
  unsigned long rightAmbient = ambientLight[1];
  unsigned long centreAmbient = ambientLight[2];

  leftCentreAmbient = leftAmbient + centreAmbient;
  rightCentreAmbient = rightAmbient + centreAmbient;
  delay(2000);
}

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
    lineSensors.getDist(sensor_read, leftCentreAmbient, rightCentreAmbient);
    

    ls_ts = millis();
  }
 
 
  Serial.print("\n");
  


}
