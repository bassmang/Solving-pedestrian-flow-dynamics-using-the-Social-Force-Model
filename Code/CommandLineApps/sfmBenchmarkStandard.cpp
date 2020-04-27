#include "sfmTargetedPedestrian.h"
#include "sfmDirectionalPedestrian.h"
#include "sfmForces.h"
#include "sfmPedestrianSpawner.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <array>
#include <vector>
#include <random>
#include <iomanip>
#include <chrono>
#include <ctime>

using namespace std;

int main(int argc, char *argv[]) {

  //  Variables to define viewer world
  const unsigned int n_pedestrians = 200;

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

  // Set destination in bottom left corner
  pos2d dest(.01, .01);

  // Set first box to top right corner
  pos2d box_min(46,6);
  pos2d box_max(48,8);

  // Use factory method to generate Targeted Pedestrians
  vector<shared_ptr<Pedestrian>> peds = PedestrianSpawner::createDistributed(
    n_pedestrians / 2, Targeted, dest, box_min, box_max);


  // Set second box to left side
  pos2d box_min_dir(2,3);
  pos2d box_max_dir(2.5,7);

  // Use factory method to generate Directional Pedestrians
  pos2d direc_dest(1,0); // Set direction of Pedestrians to be (1,0)
  vector<shared_ptr<Pedestrian>> peds_dir = PedestrianSpawner::createDistributed(
    n_pedestrians / 2, Directional, direc_dest, box_min_dir, box_max_dir);

  // Add Directional Pedestrians to vector
  for (shared_ptr<Pedestrian> p : peds_dir) {
    peds.push_back(p);
  }

  // Define time variables for outer loop
  float finish_time_s = 20.0;
  float curr_time = 0.0;
  clock_t c_start = clock();
  auto t_start = chrono::high_resolution_clock::now();
  // Loop over time period
  while (curr_time < finish_time_s) {
    // Loop over each Pedestrian to get forces and set v/r
    #pragma omp parallel for
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
  std::clock_t c_end = std::clock();
  auto t_end = std::chrono::high_resolution_clock::now();
  std::cout << std::fixed << std::setprecision(2) << "CPU time used: "
            << 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC << " ms\n"
            << "Wall clock time passed: "
            << std::chrono::duration<double, std::milli>(t_end-t_start).count()
            << " ms\n";
  return 0;
}

