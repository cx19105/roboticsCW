// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:
    float x;
    float y;
    float theta;
    float r;
    float l;
    float angleRes;
    float ltheta_dot;
    float rtheta_dot;
  
    // Constructor, must exist.
    Kinematics_c() {
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
      this->r = 16.0;
      this->l = 44.5;
      this->angleRes = (103/358.0); //mm
      this->ltheta_dot = 0;
      this->rtheta_dot = 0;

    } 

    // Use this function to update
    // your kinematics
    void update(long lcount, long rcount) {

      float lx = lcount*angleRes; //Linear distance in mm
      float rx = rcount*angleRes;

      float diameter = 3.14159*r*2;

      float lxdot = lx;  //Linear Velocity
      float rxdot = rx;
      
      ltheta_dot = (2*3.14159*lxdot)/diameter;//rads-1
      rtheta_dot = (2*3.14159*rxdot)/diameter;//rads-1
      /*float delta_theta = atan(((lcount-rcount)/(2*l*358.3)));*/
      float xr = 0.5*(r*ltheta_dot + r*rtheta_dot);
      float yr = 0;
      float theta_r_dot = (r*ltheta_dot )/(2.0*l) - (r*rtheta_dot)/(2.0*l);

      x = x + xr*cos(theta);
      y = y + xr*sin(theta);
      theta = theta + theta_r_dot;
      if (theta > (2*3.14159)){
        theta = theta - 2*3.14159;
      } else if (theta < -(2*3.14159)) {
        theta = theta + 2*3.14159;
      }
      
    }
    

};



#endif
