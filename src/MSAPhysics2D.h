#pragma once

#include "MSAPhysics.h"

namespace msa {
namespace physics  {

typedef WorldT<Vec2f>                       World2D;
typedef ParticleT<Vec2f>                    Particle2D;
typedef SpringT<Vec2f>                      Spring2D;
typedef AttractionT<Vec2f>                  Attraction2D;
typedef ConstraintT<Vec2f>                  Constraint2D;

typedef shared_ptr< WorldT<Vec2f> >			World2D_ptr;
typedef shared_ptr< ParticleT<Vec2f> >      Particle2D_ptr;
typedef shared_ptr< SpringT<Vec2f> >		Spring2D_ptr;
typedef shared_ptr< AttractionT<Vec2f> >	Attraction2D_ptr;
typedef shared_ptr< ConstraintT<Vec2f> >    Constraint2D_ptr;
//typedef ParticleUpdater_ptr<Vec2f>	ParticleUpdater2D_ptr;

}
}
