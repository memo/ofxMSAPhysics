#pragma once

#include "MSACore.h"

namespace msa {
	
	namespace physics {
		
		template <typename T>
		struct ParamsT {
			float		timeStep, timeStep2;
			float		drag;
			
			int			numIterations;
			bool		isCollisionEnabled;
			
			bool		doGravity;
			T			gravity;
			
			// do world boundaries
			bool		doWorldEdges;
			T			worldMin;					// use for sectors
			T			worldMax;
			T			worldSize;                  // cache these
//			T			worldSizeInv;
			T			sectorCount;				// number of sectors in each axis
		};
		
	}
}