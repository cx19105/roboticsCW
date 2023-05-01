
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
   void motorChangeDirection(bool reverse){
    if (reverse = true) {
      digitalWrite(lDIR, HIGH);
      digitalWrite(rDIR, HIGH);
    } else {
      digitalWrite(lDIR, LOW);
      digitalWrite(rDIR, LOW);
    }
   }
   

   void motorRotate(bool left) {
    
    if (left == true){
      digitalWrite(lDIR, LOW);
      digitalWrite(rDIR, HIGH);
    } else {
      digitalWrite(lDIR, LOW);
      digitalWrite(rDIR, LOW);
    }
   }

   void setMotorRotate(float error){
    if (error < 0) {
      digitalWrite(lDIR, HIGH);
      digitalWrite(rDIR, LOW);
   } else {
    digitalWrite(lDIR, LOW);
    digitalWrite(rDIR, HIGH);
   }
   Serial.print("-");
   
   float leftPWM = abs(error)+20;
   float rightPWM = abs(error)+20;
   Serial.print(leftPWM);
   Serial.print("-");
   analogWrite(lPWM, leftPWM );
   analogWrite(rPWM, rightPWM );
   }

   void setMotorPower(float left_pwm,float right_pwm, float dir_error) {
    int gain = 20;
    left_pwm += -dir_error * gain;
    right_pwm += dir_error * gain;
    

    if (left_pwm < 0){
      digitalWrite(lDIR, HIGH);
    } else {
      digitalWrite(lDIR, LOW);
    } 
    if (right_pwm < 0){
      digitalWrite(rDIR, HIGH);
    } else {
      digitalWrite(rDIR, LOW);
    } 
    
    analogWrite(lPWM, left_pwm );
    analogWrite(rPWM, right_pwm );
    }
};
