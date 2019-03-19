#include "Arduino.hpp"
#include "Parameters.hpp"
#include "Shaping_Functions.hpp"
#include "States.hpp"
//Calc Sigmoid function and apply to the New point

double calculateNextValue(double desired_value, double starting_value,
                          double Ts, double exp_mult, int n_iter, int N)
{ // Makes the curve for your setpoint vs time look like the voltage time graph for a charging capacitor
  //leg->n_iter tells you at which of the Nth sample you are not counting the zero
  //Ts sampling time in this case 0.001 with exp_mult=2000 it takes 6 milliseconds to rise from 0 to 1
  // it has to stop if N==leg->n_iter
  //  N = round(1 / (Ts * exp_mult) * 10);
  //  if ((N % 2)) {
  //    N++;
  //  }
  double sig = 1 / (1 + exp(-exp_mult * ((-1 * N / 2 + n_iter + 1)) * Ts));
  double next_value = starting_value + (desired_value - starting_value) * sig;
  return next_value;
}

ShapingFunction::ShapingFunction(){
  iteration_count = 0;
  iteration_threshold = 0;
  next_iteration_threshold = 0;

  exp_mult = 0;
  recharge_timer = new Timer();
}

ShapingFunction::~ShapingFunction(){
  delete recharge_timer;
}

void ShapingFunction::setIterationCount(double n){
  next_iteration_threshold = n;
}

double ShapingFunction::getIterationCount(){
  return next_iteration_threshold;
}

void ShapingFunction::beginCurve(){
  iteration_count = 0;
  iteration_threshold = next_iteration_threshold;
  exp_mult = round((10 / Ts) / (iteration_threshold - 1));
}

bool ShapingFunction::isCurveDone(){
  return iteration_count >= iteration_threshold;
}

double ShapingFunction::shape(double desired_value, double current_value, double starting_value){

  double next_value = current_value;
  if (recharge_timer->lap() >= RECHARGE_TIME){
    recharge_timer->reset();

    if ((abs(desired_value - current_value) > 0.1) && isCurveDone()) {
      beginCurve();
    }

    if (iteration_threshold >= 1){
      next_value = calculateNextValue(desired_value, starting_value,
                                      Ts, exp_mult, iteration_count, iteration_threshold);
    } else {
      next_value = desired_value;
    }
    iteration_count++;
  }

  return next_value;
}
