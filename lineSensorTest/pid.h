// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
    float p;
    float i;
    float d;
    float i_sum;
    float feedback;
    float last_error;
    
    
  
    // Constructor, must exist.
    PID_c() {
      this->p = 0;
      this->i = 0;
      this->d = 0;
      this->i_sum = 0;
      this->feedback = 0;
      this->last_error = 0;

    }
    void initialise(float newP, float newI, float newD) {
      p = newP;
      i = newI;
      d = newD;
      
    }
    void reset() {
      p = 0;
      i = 0;
      d = 0;
      i_sum =0;
      feedback = 0;
      last_error = 0;
    }
    
     float update(float ave_spd, float demand, unsigned long t){
      float error = demand-ave_spd;
      float diff_error;
      float float_dt;

      if (t == 0) return feedback;
      
      float p_term = p*error;

      i_sum = i_sum + (error*t);
      
      float i_term = i*i_sum;
      
      diff_error = (error-last_error) / t;
      last_error = error;
      float d_term = diff_error * d;

      feedback = p_term + i_term + d_term;

      return feedback;
      
     }

};



#endif
