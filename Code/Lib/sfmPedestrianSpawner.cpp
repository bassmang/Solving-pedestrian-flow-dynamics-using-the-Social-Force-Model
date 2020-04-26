#include "sfmPedestrianSpawner.h"
#include <iostream>

namespace sfm {

vector<shared_ptr<Pedestrian> > PedestrianSpawner::createUniform(
	int n, PedestrianType type, pos2d dest) {
	vector<shared_ptr<Pedestrian> > peds;
	for (int i = 0; i < n; i++) {
		float x_pos = POS2D_XWRAP * ((float) rand() / (RAND_MAX));
		float y_pos = POS2D_YWRAP * ((float) rand() / (RAND_MAX));
		pos2d orig(x_pos, y_pos);
		dir2d v(0, 0);
		float v_des = 1.3;
		float t_relax = 0.5;
		if (type == Targeted) {
			TargetedPedestrian tp(orig,dest,v,orig,v_des,t_relax);
			shared_ptr<Pedestrian> tps = make_shared<TargetedPedestrian>(tp);
			peds.push_back(tps);
		} else {
			dir2d desired_direc(dest.x(),dest.y());
			DirectionalPedestrian dp(orig,desired_direc,v,orig,v_des,t_relax);
			shared_ptr<Pedestrian> dps = make_shared<DirectionalPedestrian>(dp);
			peds.push_back(dps);
		}
	}
	vector<shared_ptr<Pedestrian> > p;
	return peds;
}

vector<shared_ptr<Pedestrian> > PedestrianSpawner::createDistributed(
	int n, PedestrianType type, pos2d dest, pos2d min, pos2d max) {
	vector<shared_ptr<Pedestrian> > peds;
	for (int i = 0; i < n; i++) {
		float x_pos = min.x() + (max.x() - min.x()) * ((float) rand() / (RAND_MAX));
		float y_pos = min.y() + (max.y() - min.y()) * ((float) rand() / (RAND_MAX));
		pos2d orig(x_pos, y_pos);
		dir2d v(0, 0);
		float v_des = 1.3;
		float t_relax = 0.5;
		if (type == Targeted) {
			TargetedPedestrian tp(orig,dest,v,orig,v_des,t_relax);
			shared_ptr<Pedestrian> tps = make_shared<TargetedPedestrian>(tp);
			peds.push_back(tps);
		} else {
			dir2d desired_direc(dest.x(),dest.y());
			DirectionalPedestrian dp(orig,desired_direc,v,orig,v_des,t_relax);
			shared_ptr<Pedestrian> dps = make_shared<DirectionalPedestrian>(dp);
			peds.push_back(dps);
		}
	}
	vector<shared_ptr<Pedestrian> > p;
	return peds;
}

} // end namespace
