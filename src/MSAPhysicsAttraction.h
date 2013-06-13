#pragma once

#include "MSACore.h"
#include "MSAPhysicsConstraint.h"


namespace msa {
	
	namespace physics {
		
		template <typename T>
		class AttractionT : public ConstraintT<T> {
		public:
			friend class WorldT<T>;
			
			AttractionT(ParticleT<T> *a, ParticleT<T> *b, float strength) {
				this->_a	= a;
				this->_b	= b;
				this->_type = kConstraintTypeAttraction;
				this->setClassName("AttractionT");

				setStrength(strength);
			}
			
			void setStrength(float s);
			float getStrength();
			
		protected:
			float _strength;
			
			void solve() {
				T delta = this->_b->getPosition() - this->_a->getPosition();
				float deltaLength2 = delta.lengthSquared();
				
				float force = deltaLength2 > 0 ? _strength * (this->_b->getMass()) * (this->_a->getMass()) / deltaLength2 : 0;
				
				T deltaForce = delta * force;
				
				if (this->_a->isFree()) this->_a->moveBy(deltaForce * this->_a->getInvMass(), false);
				if (this->_b->isFree()) this->_b->moveBy(deltaForce * -this->_b->getInvMass(), false);
			}
			
			void debugDraw() {
				ConstraintT<T>::debugDraw();
			}
		};
		
		template <typename T>
		inline void AttractionT<T>::setStrength(float s) {
			_strength = s;
		}
		
		template <typename T>
		inline float AttractionT<T>::getStrength() {
			return _strength;
		}
		
	}
}