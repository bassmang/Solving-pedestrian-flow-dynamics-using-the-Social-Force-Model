/*=============================================================================

  PHAS0100ASSIGNMENT2: PHAS0100 Assignment 2 Social Force Model

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include "sfmCatchMain.h"
#include "sfmPedestrian.h"
#include "sfmForces.h"
#include <iostream>
#include <vector>

// Helper funtion to compare floats
bool almost_eq(float a, float b, float epsilon) {
  return abs(a - b) < epsilon;
}
float epsilon = .001; // Small epsilon to ensure floats are equal

TEST_CASE( "Basic Pedestrian instantiation" ) {
  pos2d orig = pos2d(1.1,1.2);
  pos2d dest = pos2d(1.3,1.4);
  dir2d v = dir2d(1.1,52.2);
  pos2d pos = pos2d(1,1);
  float v_des = 1.1;
  float t_relax = 4.4;
  Pedestrian p(orig, dest, v, pos, v_des, t_relax);
  REQUIRE(almost_eq(p.orig.x(), 1.1, epsilon));
  REQUIRE(almost_eq(p.orig.y(), 1.2, epsilon));
  REQUIRE(almost_eq(p.v.y(), 52.2, epsilon));
  REQUIRE(almost_eq(p.t_relax, 4.4, epsilon));
}

TEST_CASE( "Check boundary conditions work when instantiated outside" ) {
  pos2d orig = pos2d(51.1,1.2);
  pos2d dest = pos2d(1.3,1.4);
  dir2d v = dir2d(1.1,52.2);
  pos2d pos = pos2d(1,1);
  float v_des = 1.1;
  float t_relax = 4.4;
  Pedestrian p(orig, dest, v, pos, v_des, t_relax);
  REQUIRE(almost_eq(p.orig.x(), 1.1, epsilon));
}

TEST_CASE( "Check boundary conditions work when moving outside" ) {
  pos2d orig = pos2d(1.1,1.2);
  pos2d dest = pos2d(1.3,1.4);
  dir2d v = dir2d(10,10);
  pos2d pos = pos2d(45,45);
  float v_des = 1.1;
  float t_relax = 4.4;
  Pedestrian p(orig, dest, v, pos, v_des, t_relax);
  REQUIRE(almost_eq(p.pos.x(), 45, epsilon));
  p.pos = p.pos + p.v;
  REQUIRE(almost_eq(p.pos.x(), 5, epsilon));
}

TEST_CASE( "Toy example to test all functionality of Force class" ) {
  // Define necessary variables as provided in write-up
  float V_0 = 2.1;
  float sigma = .3;
  float delta_t = 2;
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

  // Test unit_direction function
  REQUIRE(almost_eq(Force::unit_direction(p1s).x(), .6, epsilon));
  REQUIRE(almost_eq(Force::unit_direction(p1s).y(), .8, epsilon));
  REQUIRE(almost_eq(Force::unit_direction(p2s).x(), .5144, epsilon));
  REQUIRE(almost_eq(Force::unit_direction(p2s).y(), .8574, epsilon));
  REQUIRE(almost_eq(Force::unit_direction(p3s).x(), .8944, epsilon));
  REQUIRE(almost_eq(Force::unit_direction(p3s).y(), -.4472, epsilon));
  dir2d e1 = Force::unit_direction(p1s);

  // Test F_dest function
  REQUIRE(almost_eq(Force::F_dest(p1s).x(), .2, epsilon));
  REQUIRE(almost_eq(Force::F_dest(p1s).y(), .6, epsilon));
  REQUIRE(almost_eq(Force::F_dest(p2s).x(), -2.2282, epsilon));
  REQUIRE(almost_eq(Force::F_dest(p2s).y(), -2.7137, epsilon));
  REQUIRE(almost_eq(Force::F_dest(p3s).x(), 1.5259, epsilon));
  REQUIRE(almost_eq(Force::F_dest(p3s).y(), .0703, epsilon));
  dir2d ps1_dest_force = Force::F_dest(p1s);

  // Test b function
  REQUIRE(almost_eq(Force::b(p1s,p2s,delta_t), 1.2246, epsilon));
  REQUIRE(almost_eq(Force::b(p1s,p3s,delta_t), 9.4818, epsilon));
  float b_12 = Force::b(p1s,p2s,delta_t);
  float b_13 = Force::b(p1s,p3s,delta_t);

  // Test f_ab function
  REQUIRE(almost_eq(Force::f_ab(b_12,V_0,sigma), .1181, epsilon));
  REQUIRE(almost_eq(Force::f_ab(b_13,V_0,sigma), 0, epsilon));
  float f_ab_12 = Force::f_ab(b_12,V_0,sigma);
  float f_ab_13 = Force::f_ab(b_13,V_0,sigma);

  // Test direction_ab function
  REQUIRE(almost_eq(Force::direction_ab(p1s,p2s).x(), 0, epsilon));
  REQUIRE(almost_eq(Force::direction_ab(p1s,p2s).y(), -1, epsilon));
  REQUIRE(almost_eq(Force::direction_ab(p1s,p3s).x(), -.5299, epsilon));
  REQUIRE(almost_eq(Force::direction_ab(p1s,p3s).y(), -.8479, epsilon));
  dir2d dir_ab_12 = Force::direction_ab(p1s,p2s);
  dir2d dir_ab_13 = Force::direction_ab(p1s,p3s);
  dir2d repulsion_12 = f_ab_12*dir_ab_12;
  dir2d repulsion_13 = f_ab_13*dir_ab_13;

  // Test field_of_vision_fac function
  REQUIRE(almost_eq(Force::field_of_vision_fac(e1, -1*repulsion_12, phi, c), .5, epsilon));
  REQUIRE(almost_eq(Force::field_of_vision_fac(e1, -1*repulsion_13, phi, c), 1, epsilon));
  vector<shared_ptr<Pedestrian>> betas;
  betas.push_back(p2s);
  betas.push_back(p3s);

  // Test F_pedestrians function
  REQUIRE(almost_eq(Force::F_pedestrians(p1s, betas, delta_t, V_0, sigma, phi, c).x(), 0, epsilon));
  REQUIRE(almost_eq(Force::F_pedestrians(p1s, betas, delta_t, V_0, sigma, phi, c).y(), -.0590, epsilon));
  dir2d ps1_ped_force = Force::F_pedestrians(p1s, betas, delta_t, V_0, sigma, phi, c);

  // Test F_border function
  REQUIRE(almost_eq(Force::F_border(p1s, delta_t, U_0, R).x(), 0, epsilon));
  REQUIRE(almost_eq(Force::F_border(p1s, delta_t, U_0, R).y(), 50, epsilon));
  REQUIRE(almost_eq(Force::F_border(p2s, delta_t, U_0, R).x(), 0, epsilon));
  REQUIRE(almost_eq(Force::F_border(p2s, delta_t, U_0, R).y(), .3368, epsilon));
  REQUIRE(almost_eq(Force::F_border(p3s, delta_t, U_0, R).x(), 0, epsilon));
  REQUIRE(almost_eq(Force::F_border(p3s, delta_t, U_0, R).y(), -.0023, epsilon));

  // Test F_all function
  REQUIRE(almost_eq(Force::F_all(p1s, betas, delta_t, V_0, sigma, phi, c, U_0, R).x(), .2, epsilon));
  REQUIRE(almost_eq(Force::F_all(p1s, betas, delta_t, V_0, sigma, phi, c, U_0, R).y(), 50.541, epsilon));
}











