#ifndef sfmForces_h
#define sfmForces_h

#include "sfmTargetedPedestrian.h"
#include <vector>
#include <math.h>
#include <memory>
using namespace std;

//! Single namespace for all code in this package
namespace sfm
{

/**
* \class Force
* \brief class for forces on pedestrians
* \ingroup types
*/
class Force {

public:
// Unit direction of a Pedestrian to its destination
static dir2d unit_direction(const shared_ptr<Pedestrian> &p); 
// Force of a Pedestrian to its destination
static dir2d F_dest(const shared_ptr<Pedestrian> &p);
// Semiminor axis b calculation
static float b(const shared_ptr<Pedestrian> &alpha, const shared_ptr<Pedestrian> &beta, float delta_t);
// Function f_ab for Pedestrian-Pedestrian forces
static float f_ab(float b, float V_0, float sigma);
// Direction of a repulsive force of b on a
static dir2d direction_ab(const shared_ptr<Pedestrian> &alpha, const shared_ptr<Pedestrian> &beta);
// Feild of vision formula
static float field_of_vision_fac(const dir2d &movement, const dir2d &repulsion, float phi, float c);
// Computes force of a set of Pedestrians on one Pedestrian
static dir2d F_pedestrians(const shared_ptr<Pedestrian> &alpha, const vector<shared_ptr<Pedestrian>> &betas, 
                    float delta_t, float V_0, float sigma, float phi, float c);
// Function F_aB for Pedestrian-Border forces
static float F_aB(float mag_ab, float U_0, float R);
// Computes force of border on a pedestrian
static dir2d F_border(const shared_ptr<Pedestrian> &alpha, float delta_t, float U_0, float R);
static dir2d F_all(const shared_ptr<Pedestrian> &alpha, const vector<shared_ptr<Pedestrian>> &betas,
                   float delta_t, float V_0, float sigma, float phi, float c, float U_0, float R);
};

} // end namespace

#endif
