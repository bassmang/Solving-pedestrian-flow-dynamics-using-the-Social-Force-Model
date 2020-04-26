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
#include <iostream>
#include <vector>

// Helper funtion to compare floats
bool almost_eq(float a, float b, float epsilon) {
  return abs(a - b) < epsilon;
}
float epsilon = .0001; // Small epsilon to ensure floats are equal

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

TEST_CASE( "Check force to desination" ) {
  pos2d orig = pos2d(0,0);
  pos2d dest = pos2d(3,4);
  dir2d v = dir2d(1,1);
  pos2d pos = pos2d(0,0);
  float v_des = 2;
  float t_relax = 2.0;
  Pedestrian p(orig, dest, v, pos, v_des, t_relax);
  REQUIRE(almost_eq(p.F_dest().x(), .1, epsilon));
  REQUIRE(almost_eq(p.F_dest().y(), .3, epsilon));
}




