#ifndef sfmDirectionalPedestrian_h
#define sfmDirectionalPedestrian_h

#include "sfmPedestrianI.h"

class DirectionalPedestrian: public Pedestrian {
public:
  DirectionalPedestrian(pos2d orig, dir2d desired_direc, dir2d v, pos2d pos, float v_des, float t_relax);
  ~DirectionalPedestrian();
  dir2d desired_direc;
  pos2d GetTarget();
};

#endif
