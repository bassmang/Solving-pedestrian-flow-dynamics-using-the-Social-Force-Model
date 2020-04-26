#include "sfmDirectionalPedestrian.h"

DirectionalPedestrian::DirectionalPedestrian(pos2d orig, dir2d desired_direc, dir2d v, pos2d pos, float v_des, float t_relax) {
	this -> orig = orig;
	this -> desired_direc = desired_direc;
	this -> v = v;
	this -> pos = pos;
	this -> v_des = v_des;
	this -> t_relax = t_relax;
}

DirectionalPedestrian::~DirectionalPedestrian() { }

// Make target in desired direction to push toward that direction
pos2d DirectionalPedestrian::GetTarget() {
	return pos + desired_direc * (1 / desired_direc.length());
}