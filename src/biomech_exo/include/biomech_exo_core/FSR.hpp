#ifndef FSR_HEADER
#define FSR_HEADER
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"
#include "Port.hpp"
#include "Linked_List.hpp"

class FSR{
private:
  void updateForce(double force);
  void fillLocalReport(FSRReport* report);
  InputPort* port;
  double calibration_peak;
  Max* max_force;
  Clamp* force_clamp;

  double force;

  virtual double adjustForce(double force) = 0;

public:
  FSR(InputPort* port);
  virtual ~FSR();
  void measureForce();
  double getForce();
  void calibrate();
  void resetMaxes();
};

class FSRType10:public FSR{
private:
  double adjustForce(double force);
public:
  FSRType10(InputPort* port);
};

class FSRType40:public FSR{
  double adjustForce(double force);
public:
  FSRType40(InputPort* port);
};


class FSRGroup{
private:
  int fsr_count;

  double force;

  double fsr_percent_thresh;
  Threshold* activation_threshold;
  bool is_activated;

  void fillLocalReport(FSRReport* report);
protected:
  LinkedList<FSR*> fsrs;

public:
  FSRGroup(LinkedList<FSR*>* fsrs);
  virtual ~FSRGroup();

  bool isActivated();
  void measureForce();
  double getForce();
  void calibrate();
  double getThreshold();
  void setPercentageThreshold(double percent);
  void resetMaxes();
  void updateMaxes();
  double getPercentage();
  double getMaxPercentage();
  void updateBaseline();
  FSRReport* generateReport();
  virtual double getRatio();
  virtual double getDifference();
  void fillReport(FSRReport* report);
};

class FsrPair:public FSRGroup{
public:
  FsrPair(LinkedList<FSR*>* fsrs);
  double getRatio();
  double getDifference();
};

class FsrFactory{
private:
  int type;
public:
  FsrFactory(int type);
  FSR* createFSR(InputPort* port);
};

class FsrGroupFactory{
public:
  FSRGroup* createFsrGroup(LinkedList<FSR*>* fsrs);
};
#endif
