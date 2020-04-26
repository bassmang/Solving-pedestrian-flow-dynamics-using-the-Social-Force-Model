
#ifndef sfmPedestrianI_h
#define sfmPedestrianI_h

#include "sfmBasicTypes.h"
using namespace sfm;

class Pedestrian {
public:
  virtual pos2d GetTarget() = 0;
  pos2d orig;
  dir2d v;
  pos2d pos;
  float v_des;
  float t_relax;
};

#endif