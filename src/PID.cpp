#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->p_error = 0.0;
	this->d_error = 0.0;
	this->i_error = 0.0;
}


