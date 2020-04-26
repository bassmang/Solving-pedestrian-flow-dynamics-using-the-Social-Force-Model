#include "sfmForces.h"
#include <iostream>

namespace sfm {

// Helper function to get unit direction of a pedestrian to destination
dir2d Force::unit_direction(const shared_ptr<Pedestrian> &p) {
	dir2d direction = p -> pos.direction(p -> GetTarget());
	return direction * (1.0 / direction.length());
}

// Compute force to destination of pedestrian
dir2d Force::F_dest(const shared_ptr<Pedestrian> &p) {
  // Return formula from write-up
  return (1.0 / p -> t_relax) * (unit_direction(p)*p -> v_des - p -> v); 
}

// Helper function to get semiminor axis of ellipse b
float Force::b(const shared_ptr<Pedestrian> &alpha, const shared_ptr<Pedestrian> &beta, float delta_t) {
	dir2d r_ab = alpha -> pos.direction(beta -> pos);
	float v_b = beta -> v.length();
	dir2d e_b = unit_direction(beta);
	float term1 = r_ab.length();
	float term2 = (r_ab - v_b*delta_t*e_b).length();
	float term1_2 = pow(term1 + term2, 2);
	float term3 = pow(v_b*delta_t, 2);
	return sqrt(term1_2 - term3) / 2.0;
}

// Helper to calculate force magnitude from Pedestrian on Pedestrian
float Force::f_ab(float b, float V_0, float sigma) {
	return (V_0 / sigma) * exp(-b / sigma);
}

// Helper function to get direction of force of beta on alphas
dir2d Force::direction_ab(const shared_ptr<Pedestrian> &alpha, const shared_ptr<Pedestrian> &beta) {
	dir2d direction = alpha -> pos.direction(beta -> pos);
	return -direction * (1.0 / direction.length());
}

// Helper function for field of vision. ** Make sure to put in negative repulsion force
float Force::field_of_vision_fac(const dir2d &movement, const dir2d &repulsion, float phi, float c) {
	if (movement * repulsion >= repulsion.length()*cos(phi)) {
		return 1;
	} else {
		return c;
	}
}

dir2d Force::F_pedestrians(const shared_ptr<Pedestrian> &alpha, const vector<shared_ptr<Pedestrian>> &betas,
	                  float delta_t, float V_0, float sigma, float phi, float c) {
	dir2d sum_forces(0,0);
	for (shared_ptr<Pedestrian> beta : betas) {
		float b_val = b(alpha, beta, delta_t);
		float f_ab_val = f_ab(b_val, V_0, sigma);
		dir2d repulsion = f_ab_val * direction_ab(alpha, beta);
		dir2d movement = unit_direction(alpha);
		float w = field_of_vision_fac(movement, -1 * repulsion, phi, c);
		sum_forces = sum_forces + (w * repulsion);
	}
	return sum_forces;
}

float Force::F_aB(float mag_ab, float U_0, float R) {
	return (U_0 / R) * exp(-mag_ab / R);
}

dir2d Force::F_border(const shared_ptr<Pedestrian> &alpha, float delta_t, float U_0, float R) {
	dir2d dir_ab;
	float mag_ab;
	if (POS2D_YWRAP / 2 > alpha -> pos.y()) {
		mag_ab = alpha -> pos.y();
		dir_ab = dir2d(0,1);
	} else {
		mag_ab = POS2D_YWRAP - (alpha -> pos.y());
		dir_ab = dir2d(0,-1);
	}
	float mag_force = F_aB(mag_ab, U_0, R);
	return mag_force * dir_ab;
}

dir2d Force::F_all(const shared_ptr<Pedestrian> &alpha, const vector<shared_ptr<Pedestrian>> &betas,
            float delta_t, float V_0, float sigma, float phi, float c, float U_0, float R) {
	dir2d F_destination = F_dest(alpha);
	dir2d F_peds = F_pedestrians(alpha, betas, delta_t, V_0, sigma, phi, c);
	dir2d F_bord = F_border(alpha, delta_t, U_0, R);
	return F_destination + F_peds + F_bord;
}
  
} // end namespace
