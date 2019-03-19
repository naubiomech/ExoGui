#ifndef SHAPING_FUNCTIONS_HEADER
#define SHAPING_FUNCTIONS_HEADER
#include "Utils.hpp"

class ShapingFunction{
private:
  int iteration_count;
  int iteration_threshold;
  int next_iteration_threshold;

  double exp_mult;

  Timer* recharge_timer;
  void beginCurve();
  bool isCurveDone();
public:
  ShapingFunction();
  ~ShapingFunction();
  double shape(double desired_value, double current_value, double starting_value);
  double getPIDSetpoint(double newPIDSetpoint, double oldPIDSetpoint, double currentPIDSetpoint);
  void setIterationCount(double iterations);
  double getIterationCount();
};

#endif
