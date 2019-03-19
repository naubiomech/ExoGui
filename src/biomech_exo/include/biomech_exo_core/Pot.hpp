#ifndef POT_HEADER
#define POT_HEADER
#include "Port.hpp"
#include "Report.hpp"

class Pot{
private:
  InputPort* port;
  double angle;
public:
  Pot(InputPort* port);
  ~Pot();
  void measure();
  double getAngle();
  PotReport* generateReport();
  void fillReport(PotReport* report);
};

#endif
