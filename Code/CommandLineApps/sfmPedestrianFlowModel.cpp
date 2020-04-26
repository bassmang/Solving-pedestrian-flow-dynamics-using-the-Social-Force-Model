// Your First C++ Program

#include <iostream>
#include "sfmForces.h"
using namespace std;

int main() {

  // Define necessary variables as provided in write-up
  float V_0 = 2.1;
  float sigma = .3;
  float delta_t = .1;
  float c = .5;
  float U_0 = 10;
  float R = .2;
  float phi = 100;
  float pi = 3.1415926535897;
  phi /= pi;
  
  // Set up first pedestrian
  pos2d orig1 = pos2d(1,2);
  pos2d dest1 = pos2d(3,4);
  dir2d v1 = dir2d(1,1);
  pos2d pos1 = pos2d(0,0);
  float v_des1 = 2;
  float t_relax1 = 1.0;
  Pedestrian p1(orig1, dest1, v1, pos1, v_des1, t_relax1);
  shared_ptr<Pedestrian> p1s = make_shared<Pedestrian>(p1);

  // Set up second pedestrian
  pos2d orig2 = pos2d(3,4);
  pos2d dest2 = pos2d(3,6);
  dir2d v2 = dir2d(6,8);
  pos2d pos2 = pos2d(0,1);
  float v_des2 = 3;
  float t_relax2 = 2.0;
  Pedestrian p2(orig2, dest2, v2, pos2, v_des2, t_relax2);
  shared_ptr<Pedestrian> p2s = make_shared<Pedestrian>(p2);

  // Set up third pedestrian
  pos2d orig3 = pos2d(5,6);
  pos2d dest3 = pos2d(7,7);
  dir2d v3 = dir2d(-1,-2);
  pos2d pos3 = pos2d(5,8);
  float v_des3 = 4;
  float t_relax3 = 3.0;
  Pedestrian p3(orig3, dest3, v3, pos3, v_des3, t_relax3);
  shared_ptr<Pedestrian> p3s = make_shared<Pedestrian>(p3);

  // Push pedestrians onto vector
  vector<shared_ptr<Pedestrian>> peds;
  peds.push_back(p1s);
  peds.push_back(p2s);
  peds.push_back(p3s);

  // Define time variables for outer loop
  float finish_time_s = 50.0;
  float curr_time = 0.0;
  // Loop over time period
  while (curr_time < finish_time_s) {
    // Loop over each Pedestrian
    for (int i = 0; i < peds.size(); i++) {
      vector<shared_ptr<Pedestrian>> betas;
      // Create vector of betas, including all other Pedestrians
      for (int j = 0; j < peds.size(); j++) {
        if (i != j) {
          betas.push_back(peds[j]);
        }
      }
      // Calculate force vector
      dir2d F = Force::F_all(peds[i], betas, delta_t, V_0, sigma, phi, c, U_0, R);
      // Calculate new velocity vector
      dir2d v_new = (peds[i] -> v) + F*delta_t;
      float v_max = (1.3 * peds[i] -> v_des);
      // Rescale v_new to maximum velocity if length is over
      if (v_new.length() > v_max) {
        v_new = v_new * (1 / v_new.length()) * v_max;
      }
      // Update velocity
      (peds[i] -> v) = v_new;
      // Update position
      (peds[i] -> pos) = (peds[i] -> pos) + v_new*delta_t;

    }
    curr_time += delta_t; // Increment timestep
  }

  return 0;
}