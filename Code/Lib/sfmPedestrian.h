#ifndef sfmPedestrian_h
#define sfmPedestrian_h

#include "sfmBasicTypes.h"
using namespace sfm;

class Pedestrian {
public:
  pos2d orig;
  pos2d dest;
  dir2d v;
  pos2d pos;
  float v_des;
  float t_relax;
  Pedestrian(pos2d orig, pos2d dest, dir2d v, pos2d pos, float v_des, float t_relax);
  ~Pedestrian();
};

#endif
