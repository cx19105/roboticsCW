#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH


class Motors_c {
  private:
    int lPWM;
    int lDIR;
    int rPWM;
    int rDIR;

   public:


   Motors_c(int lPWM, int lDIR, int rPWM, int rDIR) {
    this->lPWM = lPWM;
    this->lDIR = lDIR;
    this->rPWM = rPWM;
    this->rDIR = rDIR;
    init();


   }
   void init() {
    

    pinMode(lPWM, OUTPUT);
    pinMode(lDIR, OUTPUT);
    pinMode(rPWM, OUTPUT);
    pinMode(rDIR, OUTPUT);

    digitalWrite(lDIR, LOW);
    digitalWrite(rDIR, LOW);

   }
   

   void setMotorPower(float left_pwm,float right_pwm) {
    float dir_error = 0;
    int gain = 20;
    digitalWrite(lDIR, LOW);
    digitalWrite(rDIR, LOW);
    left_pwm += -dir_error * gain;
    right_pwm += dir_error * gain;
    

    if ((left_pwm > -1) && (left_pwm < 101)){
      if ((right_pwm > -1) && (right_pwm < 101)){
        analogWrite(lPWM, left_pwm );
        analogWrite(rPWM, right_pwm );
       }
    } else {
      analogWrite(lPWM, left_pwm );
      analogWrite(rPWM, right_pwm );
    }
    
   }

    void turnOnSpot( float left_pwm, float right_pwm ) {
      if ( left_pwm < 0 ) {
        // Convert the pwm back to possible positive value
        left_pwm = -left_pwm;
        digitalWrite(L_DIR_PIN, REV);
        digitalWrite(R_DIR_PIN, FWD);
        analogWrite(L_PWM_PIN, left_pwm);
        analogWrite(R_PWM_PIN, right_pwm);
      } else {
        right_pwm = -right_pwm;
        digitalWrite(L_DIR_PIN, FWD);
        digitalWrite(R_DIR_PIN, REV);
        analogWrite(L_PWM_PIN, left_pwm);
        analogWrite(R_PWM_PIN, right_pwm);
      }
    }
    
};

#endif
