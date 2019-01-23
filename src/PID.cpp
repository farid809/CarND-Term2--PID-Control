#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */


// Init PID Gains
   this->Kp = Kp_;
   this->Ki = Ki_;
   this->Kd = Kd_;




}

void PID::InitErrorParams(void){

  this->p_error=0;
  this->i_error=0;
  this->d_error=0;
  this->prev_loop_error=0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */


  //Calculating Proportional error 
	this->p_error = cte ; 
	

  //Calculating Integral Error
	this->i_error += cte ; //should be around zero 


  //calculating Derivative error
  this->d_error = cte - this->prev_loop_error ; 
	
  
  //saving prev_loop_error
	this->prev_loop_error = this->p_error ; 

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  
  return -(this->Kp*this->p_error + this->Ki*this->i_error+this->Kd*this->d_error);  // TODO: Add your total error calc here!
}