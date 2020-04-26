#ifndef sfmTargetedPedestrian_h
#define sfmTargetedPedestrian_h

#include "sfmPedestrianI.h"

class TargetedPedestrian: public Pedestrian {
public:
  TargetedPedestrian(pos2d orig, pos2d dest, dir2d v, pos2d pos, float v_des, float t_relax);
  ~TargetedPedestrian();
  pos2d GetTarget();
};

#endif
