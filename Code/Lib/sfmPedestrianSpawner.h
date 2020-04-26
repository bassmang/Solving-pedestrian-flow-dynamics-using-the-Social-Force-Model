#ifndef sfmPedestrianSpawner_h
#define sfmPedestrianSpawner_h

#include "sfmTargetedPedestrian.h"
#include "sfmDirectionalPedestrian.h"
#include <vector>
using namespace std;

enum PedestrianType { Targeted, Directional };

//! Single namespace for all code in this package
namespace sfm
{

/**
* \class Force
* \brief class for forces on pedestrians
* \ingroup types
*/
class PedestrianSpawner {

public:
static vector<shared_ptr<Pedestrian> > createUniform(
	int n, PedestrianType type, pos2d dest);

static vector<shared_ptr<Pedestrian> > createDistributed(
	int n, PedestrianType type, pos2d dest, pos2d min, pos2d max);
};

} // end namespace

#endif
