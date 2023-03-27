class LineSensor {
  private:

    int lPIN;
    int cPIN;
    int rPIN;

    int EMIT;
    int noPins;

  public:

  LineSensor(int NB_LS_PINS, int LS_LEFT_IN_PIN, int LS_CENTRE_IN_PIN, int LS_RIGHT_IN_PIN, int EMIT) {
    
    
    this->lPIN = LS_LEFT_IN_PIN;
    this->cPIN = LS_CENTRE_IN_PIN;
    this->rPIN = LS_RIGHT_IN_PIN;

    this->EMIT = EMIT;
    this->noPins = NB_LS_PINS;
    init();
  }

  void init() {
    pinMode(EMIT, OUTPUT);
    pinMode(lPIN, INPUT);
    pinMode(cPIN, INPUT);
    pinMode(rPIN, INPUT);
  
    digitalWrite(EMIT, HIGH);
  }
  void chargeCapacitors(){
    digitalWrite(EMIT, HIGH);
    pinMode(lPIN, OUTPUT);
    pinMode(cPIN, OUTPUT);
    pinMode(rPIN, OUTPUT);

    digitalWrite(lPIN, HIGH);
    digitalWrite(cPIN, HIGH);
    digitalWrite(rPIN, HIGH);

  
    delayMicroseconds(10);
  
    pinMode(lPIN, INPUT);
    pinMode(cPIN, INPUT);
    pinMode(rPIN, INPUT);
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


void getDist(unsigned long sensor_read[3], unsigned long lcAmbient, unsigned long rcAmbient){
  float variance = 1.05;
  float minlcShadow = lcAmbient*variance;
  float minrcShadow = rcAmbient*variance;
  float dist;

  float lcLinear = 2.25947764;
  float rcLinear = 2.49289241;
  float aveDist = 0;
  unsigned long lcReading = sensor_read[0] + sensor_read[2];
  unsigned long rcReading = sensor_read[1] + sensor_read[2];
  Serial.print(lcReading);
  Serial.print("-");
  Serial.print(rcReading);
  Serial.print("-");

  if (lcReading > minlcShadow) {
    float ldist = (lcReading - lcAmbient) / lcLinear;
    float rdist = (rcReading - rcAmbient)/ rcLinear;
    aveDist = (ldist + rdist) / 2;
  }

  Serial.print(aveDist);
}
  
void readLineSensor(unsigned long sensor_read[3]) {
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
  return sensor_read;
}
  
};
