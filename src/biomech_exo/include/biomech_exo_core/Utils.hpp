#ifndef UTILITES_HEADER
#define UTILITES_HEADER

class RunningAverage{
private:
  double avg;
  double count;
public:
  RunningAverage();
  double update(double value);
  double getAverage();
  void reset();
};

class MovingAverage{
private:
  double* previous_values;
  double total;
  double average;
  int index;
  int size;
public:
  MovingAverage(int size);
  ~MovingAverage();
  double update(double value);
  double getAverage();
  void reset();
};

class Clamp{
private:
  double upper;
  double lower;
public:
  Clamp(double lower, double upper);
  double clamp(double value);
};

class Threshold{
private:
  double upper_threshold;
  double lower_threshold;
  bool state;
public:
  Threshold(bool starting_state, double upper_threshold_value, double lower_threshold_value);
  bool getState(double value);
  void setUpperThreshold(double value);
  void setLowerThreshold(double value);
};

class Timer{
private:
  double start_time;
  double pause_time;
public:
  Timer();
  double lap();
  double lapSec();
  void reset();
  void pause();
  void resume();
};

class Max{
private:
  double maxVal;
public:
  Max();
  double getMax();
  void update(double value);
  void reset();
};

class Min{
private:
  double minVal;
public:
  Min();
  double getMin();
  void update(double value);
  void reset();
};

class ChangeTrigger{
private:
  bool state;
public:
  ChangeTrigger(bool start_state);
  bool update(bool state);
};

class Range{
private:
  double avg;
  Max* maximum;
  Min* minimum;
  double last_max;
  double last_min;
  Threshold* threshold;
  ChangeTrigger* trigger;
  MovingAverage* avgMax;
  MovingAverage* avgMin;
public:
  double getMax();
  double getMin();
  void update(double value);
};

void updateMax(double* max_val, double val);

#endif
