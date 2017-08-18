#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(const double Kp, const double Ki, const double Kd) {
  this->Kp = Kp;
  p_error = 0;

  this->Kd = Kd;
  d_error = 0;

  this->Ki = Ki;
  i_error = 0;
}

void PID::UpdateError(const double cte) {
  const double delta_cte = cte - p_error;
  p_error = cte;
  d_error = delta_cte;
  i_error += cte;
}

double PID::TotalError() {
  return (-(Kp*p_error + Kd*d_error + Ki*i_error));
}
