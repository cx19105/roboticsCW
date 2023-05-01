
class LineSensor {
  private:
    int flPIN;
    int frPIN;
    int lPIN;
    int cPIN;
    int rPIN;
    int lbump;
    int rbump;
    int EMIT;
    int noPins;

  public:

  LineSensor(int NB_LS_PINS, int LS_FLEFT_IN_PIN, int LS_LEFT_IN_PIN, int LS_CENTRE_IN_PIN, int LS_RIGHT_IN_PIN, int LS_FRIGHT_IN_PIN, int EMIT) {
    this-> flPIN = LS_FLEFT_IN_PIN;
    this->frPIN = LS_FRIGHT_IN_PIN;
    this->lPIN = LS_LEFT_IN_PIN;
    this->cPIN = LS_CENTRE_IN_PIN;
    this->rPIN = LS_RIGHT_IN_PIN;
    this->lbump = 4;
    this->rbump = 5;
    this->EMIT = EMIT;
    this->noPins = NB_LS_PINS;
    init();
  }

  void init() {
    pinMode(EMIT, OUTPUT);
    pinMode(lPIN, INPUT);
    pinMode(cPIN, INPUT);
    pinMode(rPIN, INPUT);
    pinMode(flPIN, INPUT);
    pinMode(frPIN, INPUT);
    pinMode(lbump, INPUT);
    pinMode(rbump, INPUT);
  
    digitalWrite(EMIT, HIGH);
  }
  void chargeCapacitors(){
    digitalWrite(EMIT, HIGH);
    pinMode(lPIN, OUTPUT);
    pinMode(cPIN, OUTPUT);
    pinMode(rPIN, OUTPUT);
    pinMode(flPIN, OUTPUT);
    pinMode(frPIN, OUTPUT);
    pinMode(lbump, OUTPUT);
    pinMode(rbump, OUTPUT);
    digitalWrite(lPIN, HIGH);
    digitalWrite(cPIN, HIGH);
    digitalWrite(rPIN, HIGH);
    digitalWrite(flPIN, HIGH);
    digitalWrite(frPIN, HIGH);
    digitalWrite(lbump, HIGH);
    digitalWrite(rbump, HIGH);
  
    delayMicroseconds(10);
  
    pinMode(lPIN, INPUT);
    pinMode(cPIN, INPUT);
    pinMode(rPIN, INPUT);
    pinMode(flPIN, INPUT);
    pinMode(frPIN, INPUT);
    pinMode(lbump, INPUT);
    pinMode(rbump, INPUT);
}

void getAmbient(float ambientLight[3]){
  int numberOfReadings = 100;
  int readingNumber = 0;
  unsigned long sensor_read[3];
  unsigned long leftAmbient = 0;
  unsigned long centreAmbient = 0;
  unsigned long rightAmbient = 0;

  while (readingNumber < numberOfReadings) {
    readingNumber += 1;
    unsigned long sensor_read[3];
    int ls_pin[noPins] = {lPIN, cPIN, rPIN};
    int which_pin;
        
    chargeCapacitors();
       
      
    for( which_pin=0; which_pin < noPins; which_pin++) {
       sensor_read[which_pin] = 20000;
    }
      
    unsigned long start_time;
    unsigned long end_time;
      
    start_time = micros();
      
    int remaining = noPins;
      
    unsigned long timeout = 5000;
      
    while(remaining > 0){
          
       unsigned long current_time = micros();
       unsigned long elapsed_time = current_time - start_time;
          
       for (which_pin = 0; which_pin < noPins; which_pin++) {
         if( digitalRead(ls_pin[which_pin]) == LOW) {
              if (sensor_read[which_pin] > 10000){
          
                sensor_read[which_pin] = elapsed_time;
                remaining = remaining - 1;
                }
            }
          }
          if (elapsed_time >= timeout){
            
            sensor_read[which_pin] = timeout;
            remaining = 0;
          }
        }

    
    leftAmbient += sensor_read[0];
    centreAmbient += sensor_read[1];
    rightAmbient += sensor_read[2];
    
  }

  ambientLight[0] = leftAmbient/readingNumber;
  ambientLight[1] = centreAmbient/readingNumber;
  ambientLight[2] = rightAmbient/readingNumber;
  return ambientLight;
}

void readLineSensor(unsigned long sensor_read[7]) {
  int ls_pin[noPins] = {flPIN, lPIN, cPIN, rPIN, frPIN, lbump, rbump};
  int which_pin;
  
  chargeCapacitors();
 

  for( which_pin=0; which_pin < noPins; which_pin++) {
    sensor_read[which_pin] = 20000;
  }

  unsigned long start_time;
  unsigned long end_time;

  start_time = micros();

  int remaining = noPins;

  unsigned long timeout = 5000;

  while(remaining > 0){
    
    unsigned long current_time = micros();
    unsigned long elapsed_time = current_time - start_time;
    
    for (which_pin = 0; which_pin < noPins; which_pin++) {
      if( digitalRead(ls_pin[which_pin]) == LOW) {
        if (sensor_read[which_pin] > 10000){
    
          sensor_read[which_pin] = elapsed_time;
          remaining = remaining - 1;
          }
      }
    }
    if (elapsed_time >= timeout){
      
      sensor_read[which_pin] = timeout;
      remaining = 0;
    }
  }
  return sensor_read;
}
  
};
