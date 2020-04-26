
#include "sfmTargetedPedestrian.h"
#include "sfmDirectionalPedestrian.h"
#include "sfmForces.h"
#include "sfmPedestrianSpawner.h"
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
  const unsigned int n_pedestrians = 20;

  // Create viewer and initialise with required number of pedestrians
  sfm::Visualiser viewer(n_pedestrians, POS2D_XWRAP, POS2D_YWRAP);

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

  // Set destination in top right corner
  pos2d dest(POS2D_XWRAP - .01, POS2D_YWRAP - .01)

  // Set box to bottom left corner
  pos2d box_min(.1,.1);
  pos2d box_max(2,2);

  // Use factory method to generate Pedestrians
  vector<shared_ptr<Pedestrian>> peds = PedestrianSpawner::createDistributed(
    n_pedestrians, Targeted, dest, box_min, box_max);

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
