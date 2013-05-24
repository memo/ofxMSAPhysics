#pragma once

#include "MSAPhysicsParticle.h"

namespace msa {
	
	namespace physics {
		
		template <typename T>
		class SectorT : public ObjCPointer {
		public:
			void	checkSectorCollisions();
			
			void	addParticle(ParticleT<T> *p) {
				_particles.push_back(p);
			}
			
			void	clear() {
				_particles.clear();
			}
			
			//	void	checkParticle(ParticleT *p);
			
		protected:
			bool	checkCollisionBetween(ParticleT<T>* a, ParticleT<T>* b);
			vector<ParticleT<T>*>	_particles;
		};
		
		
        //--------------------------------------------------------------
		template <typename T>
		void SectorT<T>::checkSectorCollisions() {
			int s = _particles.size();
			for(int i=0; i<s-1; i++) {
				for(int j=i+1; j<s; j++) {
					checkCollisionBetween(_particles[i], _particles[j]);
				}
			}
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		bool SectorT<T>::checkCollisionBetween(ParticleT<T> *a, ParticleT<T> *b) {
			if(a->hasCollision() == false || b->hasCollision() == false) return false;
            if(a->hasPassiveCollision() && b->hasPassiveCollision()) return false;
			if((a->collisionPlane & b->collisionPlane) == 0) return false;
            
            
//			printf("same planes %i %i\n", a->collisionPlane, b->collisionPlane);
			
			float restLength = b->getRadius() + a->getRadius();
			T delta = b->getPosition() - a->getPosition();
			float deltaLength2 = delta.lengthSquared();
			if(deltaLength2 >restLength * restLength) return false;
			
			float deltaLength = sqrt(deltaLength2);	// TODO: fast approximation of square root (1st order Taylor-expansion at a neighborhood of the rest length r (one Newton-Raphson iteration with initial guess r))
			float force = (deltaLength - restLength) / (deltaLength * (a->getInvMass() + b->getInvMass()));
			
			T deltaForce = delta * force;
			
			if (a->isFree()) a->moveBy(deltaForce * a->getInvMass(), false);
			if (b->isFree()) b->moveBy(deltaForce * -b->getInvMass(), false);
			
			a->collidedWithParticle(b, deltaForce);
			b->collidedWithParticle(a, -deltaForce);
			
			return true;
		}
		
		
	}
}