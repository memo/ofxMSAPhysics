/*
 based on principles mentioned at
 http://www.gamasutra.com/resource_guide/20030121/jacobson_01.shtml
 
 
 fast inverse square root mentioned at
 http://en.wikipedia.org/wiki/Fast_inverse_square_root
 attributed to John Carmack but apparently much older
 */

#pragma once

#include "MSACore.h"
#include "MSAObjCPointer.h"

#include "MSAPhysicsParticle.h"
#include "MSAPhysicsConstraint.h"
#include "MSAPhysicsSpring.h"
#include "MSAPhysicsAttraction.h"

#include "MSAPhysicsParams.h"
#include "MSAPhysicsCallbacks.h"

#include "MSAPhysicsSector.h"
#include "MSAPhysicsWorld.h"

#ifdef MSAPHYSICS_USE_RECORDER
#include "MSAPhysicsDataRecorder.h"
#endif

