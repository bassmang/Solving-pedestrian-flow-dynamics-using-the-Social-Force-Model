#include "sfmTargetedPedestrian.h"

TargetedPedestrian::TargetedPedestrian(pos2d orig, pos2d dest, dir2d v, pos2d pos, float v_des, float t_relax) {
	this -> orig = orig;
	this -> dest = dest;
	this -> v = v;
	this -> pos = pos;
	this -> v_des = v_des;
	this -> t_relax = t_relax;
}

TargetedPedestrian::~TargetedPedestrian() { }

pos2d TargetedPedestrian::GetTarget() {
	return dest;
}