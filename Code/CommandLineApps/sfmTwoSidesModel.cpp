
#include "sfmForces.h"
#include "sfmVisualiser.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <array>
#include <vector>
#include <random>

using namespace std;

int main() {

  //  Variables to define viewer world
  double world_width_x = POS2D_XWRAP;
  double world_width_y = POS2D_YWRAP;
  const unsigned int n_pedestrians = 20;

  // Create viewer and initialise with required number of pedestrians
  sfm::Visualiser viewer(n_pedestrians, world_width_x, world_width_y);

  // Define necessary variables as provided in write-up
  float V_0 = 2.1;
  float sigma = .3;
  float delta_t = .5;
  float c = .5;
  float U_0 = 10;
  float R = .2;
  float phi = 100;
  float pi = 3.1415926535897;
  phi /= pi;

  // Set uniform variables for all 10 pedestrians
  float v_des = 1.3;
  float t_relax = 0.5;
  float r_start = POS2D_XWRAP - .01;

  // Set up 5 pedestrians on left side
  dir2d lv = dir2d(1,0);
  vector<int> l_ys = {1,3,5,7,9};
  TargetedPedestrian p0(pos2d(0,l_ys[0]), pos2d(r_start,l_ys[0]), lv, pos2d(0,l_ys[0]), v_des, t_relax);
  TargetedPedestrian p1(pos2d(0,l_ys[1]), pos2d(r_start,l_ys[1]), lv, pos2d(0,l_ys[1]), v_des, t_relax);
  TargetedPedestrian p2(pos2d(0,l_ys[2]), pos2d(r_start,l_ys[2]), lv, pos2d(0,l_ys[2]), v_des, t_relax);
  TargetedPedestrian p3(pos2d(0,l_ys[3]), pos2d(r_start,l_ys[3]), lv, pos2d(0,l_ys[3]), v_des, t_relax);
  TargetedPedestrian p4(pos2d(0,l_ys[4]), pos2d(r_start,l_ys[4]), lv, pos2d(0,l_ys[4]), v_des, t_relax);

  // Set up 5 pedestrians on right side
  dir2d rv = dir2d(-1,0);
  vector<int> r_ys = {1,3,5,7,9};
  TargetedPedestrian p5(pos2d(r_start,r_ys[0]), pos2d(0,r_ys[0]), rv, pos2d(r_start,r_ys[0]), v_des, t_relax);
  TargetedPedestrian p6(pos2d(r_start,r_ys[1]), pos2d(0,r_ys[1]), rv, pos2d(r_start,r_ys[1]), v_des, t_relax);
  TargetedPedestrian p7(pos2d(r_start,r_ys[2]), pos2d(0,r_ys[2]), rv, pos2d(r_start,r_ys[2]), v_des, t_relax);
  TargetedPedestrian p8(pos2d(r_start,r_ys[3]), pos2d(0,r_ys[3]), rv, pos2d(r_start,r_ys[3]), v_des, t_relax);
  TargetedPedestrian p9(pos2d(r_start,r_ys[4]), pos2d(0,r_ys[4]), rv, pos2d(r_start,r_ys[4]), v_des, t_relax);

  // Make shared pointers for each pedestrian
  shared_ptr<Pedestrian> p0s = make_shared<TargetedPedestrian>(p0);
  shared_ptr<Pedestrian> p1s = make_shared<TargetedPedestrian>(p1);
  shared_ptr<Pedestrian> p2s = make_shared<TargetedPedestrian>(p2);
  shared_ptr<Pedestrian> p3s = make_shared<TargetedPedestrian>(p3);
  shared_ptr<Pedestrian> p4s = make_shared<TargetedPedestrian>(p4);
  shared_ptr<Pedestrian> p5s = make_shared<TargetedPedestrian>(p5);
  shared_ptr<Pedestrian> p6s = make_shared<TargetedPedestrian>(p6);
  shared_ptr<Pedestrian> p7s = make_shared<TargetedPedestrian>(p7);
  shared_ptr<Pedestrian> p8s = make_shared<TargetedPedestrian>(p8);
  shared_ptr<Pedestrian> p9s = make_shared<TargetedPedestrian>(p9);

  // Make vector for all Pedestrians and push each
  vector<shared_ptr<Pedestrian>> peds;
  peds.push_back(p0s);
  peds.push_back(p1s);
  peds.push_back(p2s);
  peds.push_back(p3s);
  peds.push_back(p4s);
  peds.push_back(p5s);
  peds.push_back(p6s);
  peds.push_back(p7s);
  peds.push_back(p8s);
  peds.push_back(p9s);

  // Define time variables for outer loop
  float finish_time_s = 10.0;
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
      // Send updated pedestrian positions to viewer 
      viewer.SetPedestrian(i, (peds[i] -> pos).x(), (peds[i] -> pos).y(), 
		              (peds[i] -> v).x(), (peds[i] -> v).y());
      // Tell viewer to redraw scene
      viewer.UpdateScene();
      // Sleep for a bit so can see visualiser updating 
      std::this_thread::sleep_for (std::chrono::milliseconds(200));

    }
    curr_time += delta_t; // Increment timestep
  }

  return 0;
}
