#pragma once

#include "MSAPhysics.h"

namespace msa {
	namespace physics  {
		
		
//		template class WorldT<Vec3f>;
//		template class ParticleT<Vec3f>;
//		template class SpringT<Vec3f>;
//		template class AttractionT<Vec3f>;
//		template class ConstraintT<Vec3f>;
		
		typedef WorldT<Vec3f>			World3D;
		typedef ParticleT<Vec3f>		Particle3D;
		typedef SpringT<Vec3f>			Spring3D;
		typedef AttractionT<Vec3f>		Attraction3D;
		typedef ConstraintT<Vec3f>		Constraint3D;
		typedef ParticleUpdaterT<Vec3f>	ParticleUpdater3D;

		
	}
}
