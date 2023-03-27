
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
   

   void setMotorPower(float left_pwm, float right_pwm) {
    if ((left_pwm > -1) && (left_pwm < 101)){
      if ((right_pwm > -1) && (right_pwm < 101)){
        analogWrite(lPWM, left_pwm );
        analogWrite(rPWM, right_pwm );
       }
    }
    
   }
};
