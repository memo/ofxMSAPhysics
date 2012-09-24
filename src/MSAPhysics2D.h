#pragma once

#include "MSAPhysics.h"

namespace msa {
	namespace physics  {
		
		typedef WorldT<Vec2f>			World2D;
		typedef ParticleT<Vec2f>		Particle2D;
		typedef SpringT<Vec2f>			Spring2D;
		typedef AttractionT<Vec2f>		Attraction2D;
		typedef ConstraintT<Vec2f>		Constraint2D;
		typedef ParticleUpdaterT<Vec2f>	ParticleUpdater2D;
		
	}
}
