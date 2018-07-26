#include "PID.h"
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::init(double Kp, double Kd, double Ki) {
	this->Kp = Kp;
	this->Kd = Kd;
	this->Ki = Ki;

	this->p_error = 0.0;
	this->d_error = 0.0;
	this->i_error = 0.0;
}

/*
* Calculate steering value here, remember the steering value is [-1, 1]
*/
double PID::getSteerValue(double cte)
{
	if(this->p_error == 0.0)
		this->p_error = cte;

	// DEBUG
	//std::cout << "Kp: " << this->Kp << ", Kd: " << this->Kd << ", Ki: " << this->Ki << std::endl;
	//std::cout << "p_error: " << this->p_error << ", d_error: " << this->d_error << ", i_error: " << this->i_error << std::endl;
	//std::cout << "CTE: " << cte << " Speed: " << speed << ", Angle: " << angle << ", Steering Value: " << steer_value << std::endl;
	//std::cout << "CTE: " << cte << std::endl;

	// Apply the PID control formula
	this->d_error = cte - this->p_error;
	this->p_error = cte;
	this->i_error += cte;

	return -this->Kp * this->p_error - this->Kd * this->d_error - this->Ki * this->i_error;
}
