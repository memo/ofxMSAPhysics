#pragma once

#include "MSAObjCPointer.h"
#include "MSAPhysicsParticle.h"

namespace msa {
	
	namespace physics {
		
		/********************* ParticleT updater class *************************/
		template <typename T>
		class ParticleUpdaterT : public ObjCPointer {
		public:
			bool ignoreFixedParticles;
			
			ParticleUpdaterT() {
				setClassName("ParticleUpdaterT");
				ignoreFixedParticles = true;
			}
			
			virtual void update(ParticleT<T>* p) = 0;
		};
		
		
		
		/********************* ParticleT updateable class (for individual particles, or the whole physics class *************************/
		/********************* To allow forcefields on individual particles, or the whole particle system  *************************/
		template <typename T>
		class ParticleUpdatableT {
		public:
			ParticleUpdatableT() {}
			
			virtual ~ParticleUpdatableT() {
				for(typename vector<ParticleUpdaterT<T>*>::iterator it = _updaters.begin(); it != _updaters.end(); it++) {
					ParticleUpdaterT<T>* updater = *it;
					if(updater) {
						updater->release();
						updater = NULL;
					}
				}
				_updaters.clear();
			}
			
			ParticleUpdatableT<T>* addUpdater(ParticleUpdaterT<T>* updater) {
				_updaters.push_back(updater);
				updater->retain();
				return this;	// so you can carry on adding updater
			}
			
			void applyUpdaters(ParticleT<T>* particle) {
				for(typename vector<ParticleUpdaterT<T>*>::iterator it = _updaters.begin(); it != _updaters.end(); it++) {
					ParticleUpdaterT<T>* updater = *it;
					if(!(updater->ignoreFixedParticles && particle->isFixed())) updater->update(particle);
				}
			}
			
			
		protected:
			vector<ParticleUpdaterT<T>*> _updaters;
		};
		
	}
}