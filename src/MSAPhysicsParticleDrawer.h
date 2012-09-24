#pragma once

#include "MSAObjCPointer.h"
#include "MSAPhysicsParticle.h"


namespace msa {
	
	namespace physics {
		
		/********************* ParticleT drawer class *************************/
		template <typename T>
		class ParticleDrawerT : public ObjCPointer  {
		public:
			ParticleDrawerT() {
				setClassName("ParticleDrawerT");
			}
			
			virtual void draw(ParticleT<T>* p) {
//				glPushMatrix();
//				glTranslatef(p->getX(), p->getY(), p->getZ());
//				glutSolidSphere(10, 10, 10);
//				glPopMatrix();
			};
		};
		
		
		
		
		/********************* Base class for drawable class *************************/
		template <typename T>
		class ParticleDrawableT {
		public:
			ParticleDrawableT() {
				_drawer = NULL;
			}
			
			virtual ~ParticleDrawableT() {
				if(_drawer) _drawer->release();
			}
			
			
			void draw(ParticleT<T>* particle) {
				if(_drawer) draw(particle);
			}
			
			
		protected:
			ParticleDrawerT<T> *_drawer;
		};
		
	}
}
