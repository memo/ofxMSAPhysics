#pragma once

#include "MSAPhysics.h"

namespace msa {
namespace physics  {

typedef WorldT<Vec3f>                       World3D;
typedef ParticleT<Vec3f>                    Particle3D;
typedef SpringT<Vec3f>                      Spring3D;
typedef AttractionT<Vec3f>                  Attraction3D;
typedef ConstraintT<Vec3f>                  Constraint3D;

typedef shared_ptr< WorldT<Vec3f> >			World3D_ptr;
typedef shared_ptr< ParticleT<Vec3f> >      Particle3D_ptr;
typedef shared_ptr< SpringT<Vec3f> >		Spring3D_ptr;
typedef shared_ptr< AttractionT<Vec3f> >	Attraction3D_ptr;
typedef shared_ptr< ConstraintT<Vec3f> >    Constraint3D_ptr;
//typedef ParticleUpdater_ptr<Vec3f>	ParticleUpdater3D_ptr;

}
}
