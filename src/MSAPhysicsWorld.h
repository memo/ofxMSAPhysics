#pragma once

#include "MSAPhysics.h"


namespace msa {
	
	namespace physics {
		
		template <typename T>
		class WorldT : public ParticleUpdatableT<T> {
			
		public:
			friend class ParticleT<T>;
			
			bool				verbose;
			
			WorldT();
			~WorldT();
			
			ParticleT<T>*		makeParticle(T pos, float m = 1.0f, float d = 1.0f);
			SpringT<T>*			makeSpring(ParticleT<T> *a, ParticleT<T> *b, float _strength, float _restLength);
			AttractionT<T>*		makeAttraction(ParticleT<T> *a, ParticleT<T> *b, float _strength);
			
			// this method retains the particle, so you should release() it after adding (obj-c style)
			ParticleT<T>*		addParticle(ParticleT<T> *p);
			
			// this method retains the constraint, so you should release it after adding (obj-c style)
			ConstraintT<T>*		addConstraint(ConstraintT<T> *c);
			
			ParticleT<T>*		getParticle(long i);
			ConstraintT<T>*		getConstraint(long i);			// generally you wouldn't use this but use the ones below
			SpringT<T>*			getSpring(long i);
			AttractionT<T>*		getAttraction(long i);
			
			long				numberOfParticles();
			long				numberOfConstraints();		// all constraints: springs, attractions and user created
			long				numberOfSprings();			// only springs
			long				numberOfAttractions();		// only attractions
			
			WorldT<T>*			setDrag(float drag = 0.99);					// set the drag. 1: no drag at all, 0.9: quite a lot of drag, 0: particles can't even move
			WorldT<T>*			setGravity(float gy = 0);					// set gravity (y component only)
			WorldT<T>*			setGravity(T g);						// set gravity (full vector)
			T&					getGravity();
			WorldT<T>*			setTimeStep(float timeStep);
			WorldT<T>*			setNumIterations(float numIterations = 20);	// default value
			
			// for optimized collision, set world dimensions first
			WorldT<T>*			setWorldMin(T worldMin);
			WorldT<T>*			setWorldMax(T worldMax);
			WorldT<T>*			setWorldSize(T worldMin, T worldMax);
			WorldT<T>*			clearWorldSize();
			
			// and then set sector size (or count)
			WorldT<T>*			enableCollision();
			WorldT<T>*			disableCollision();
			bool				isCollisionEnabled();
			WorldT<T>*			setSectorCount(int count);		// set the number of sectors (will be equal in each axis)
			WorldT<T>*			setSectorCount(T vCount);	// set the number of sectors in each axis
			
			// preallocate buffers if you know how big they need to be (they grow automatically if need be)
			WorldT<T>*			setParticleCount(long i);
			WorldT<T>*			setConstraintCount(long i);
			WorldT<T>*			setSpringCount(long i);
			WorldT<T>*			setAttractionCount(long i);
			
			
			void clear();
			void update(int frameNum = -1);
			void draw();
			void debugDraw();
			
#ifdef MSAPHYSICS_USE_RECORDER
			WorldT<T>*			setReplayMode(int i, float playbackScaler = 1.0f);		// when playing back recorded data, optionally scale positions up (so you can record in lores, playback at highres)
			WorldT<T>*			setReplayFilename(string f);
#endif
			
			ParamsT<T>&			getParams();
			
		protected:
			vector<ParticleT<T>*>	_particles;
			vector<ConstraintT<T>*>	_constraints[kConstraintTypeCount];
			vector<SectorT<T>*>		_sectors;
			
			ParamsT<T>				params;
			
			void					updateParticles();
			void					updateConstraints();
			void					updateConstraintsByType(vector<ConstraintT<T>*> constraints);
			//	void						updateWorldSize();
			
			
			void					checkAllCollisions();
			
			ConstraintT<T>*			getConstraint(ParticleT<T> *a, int constraintType);
			ConstraintT<T>*			getConstraint(ParticleT<T> *a, ParticleT<T> *b, int constraintType);
			
			
#ifdef MSAPHYSICS_USE_RECORDER
			DataRecorder<T>			_recorder;
			long					_frameCounter;
			long					_replayMode;
			float					_playbackScaler;
			void load(long frameNum);
#endif
		};
		
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>::WorldT() {
			verbose = false;
			setTimeStep(0.000010);
			setDrag();
			setNumIterations();
			disableCollision();
			setGravity();
			clearWorldSize();
			setSectorCount(0);
			
#ifdef MSAPHYSICS_USE_RECORDER
			_frameCounter = 0;
			setReplayMode(OFX_MSA_DATA_IDLE);
			setReplayFilename("recordedData/physics/physics");
#endif
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>::~WorldT() {
			clear();
		}
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* WorldT<T>::makeParticle(T pos, float m, float d) {
			ParticleT<T> *p = new ParticleT<T>(pos, m, d);
			addParticle(p);
			p->release();	// cos addParticle(p) retains it
			return p;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		SpringT<T>* WorldT<T>::makeSpring(ParticleT<T> *a, ParticleT<T> *b, float _strength, float _restLength) {
			if(a==b) return NULL;
			SpringT<T>* c = new SpringT<T>(a, b, _strength, _restLength);
			addConstraint(c);
			c->release();	// cos addConstraint(c) retains it
			return c;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		AttractionT<T>* WorldT<T>::makeAttraction(ParticleT<T> *a, ParticleT<T> *b, float _strength) {
			if(a==b) return NULL;
			AttractionT<T>* c = new AttractionT<T>(a, b, _strength);
			addConstraint(c);
			c->release();	// cos addConstraint(c) retains it
			return c;
		}
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* WorldT<T>::addParticle(ParticleT<T> *p) {
			p->verbose = verbose;
			_particles.push_back(p);
			p->setInstanceName(string("particle "));// + ofToString(_particles.size(), 0));
			p->_params = &params;
			p->_world = this;
			
#ifdef MSAPHYSICS_USE_RECORDER
			if(_replayMode == OFX_MSA_DATA_SAVE)
				_recorder.setSize(numberOfParticles());
#endif
			p->retain();
			return p;		// so you can configure the particle or use for creating constraints
		}
		
        //--------------------------------------------------------------
		template <typename T>
		ConstraintT<T>* WorldT<T>::addConstraint(ConstraintT<T> *c) {
			c->verbose = verbose;
			_constraints[c->type()].push_back(c);
			c->_params = &params;
			
			c->retain();
			(c->_a)->retain();
			(c->_b)->retain();
			
			switch(c->type()) {
				case kConstraintTypeCustom:
					c->setInstanceName(string("constraint "));// + ofToString(_constraints[kConstraintTypeCustom].size(), 0));
					break;
					
				case kConstraintTypeSpring:
					c->setInstanceName(string("spring "));// + ofToString(_constraints[kConstraintTypeSpring].size(), 0));
					break;
					
				case kConstraintTypeAttraction:
					c->setInstanceName(string("attraction "));// + ofToString(_constraints[kConstraintTypeAttraction].size(), 0));
					break;
			}
			
			return c;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>*		WorldT<T>::getParticle(long i) {
			return i < numberOfParticles() ? _particles[i] : NULL;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		ConstraintT<T>*	WorldT<T>::getConstraint(long i) {
			return i < numberOfConstraints() ? _constraints[kConstraintTypeCustom][i] : NULL;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		SpringT<T>*		WorldT<T>::getSpring(long i) {
			return i < numberOfSprings() ? (SpringT<T>*)_constraints[kConstraintTypeSpring][i] : NULL;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		AttractionT<T>*	WorldT<T>::getAttraction(long i) {
			return i < numberOfAttractions() ? (AttractionT<T>*)_constraints[kConstraintTypeAttraction][i] : NULL;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		long WorldT<T>::numberOfParticles() {
			return _particles.size();
		}
		
        //--------------------------------------------------------------
		template <typename T>
		long WorldT<T>::numberOfConstraints() {
			return _constraints[kConstraintTypeCustom].size();
		}
		
        //--------------------------------------------------------------
		template <typename T>
		long WorldT<T>::numberOfSprings() {
			return _constraints[kConstraintTypeSpring].size();
		}
		
        //--------------------------------------------------------------
		template <typename T>
		long WorldT<T>::numberOfAttractions() {
			return _constraints[kConstraintTypeAttraction].size();
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>*  WorldT<T>::setDrag(float drag) {
			params.drag = drag;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>*  WorldT<T>::setGravity(float gy) {
			T g = T::zero();
			g[1] = gy;
			setGravity(g);
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>*  WorldT<T>::setGravity(T g) {
			params.gravity= g;
			params.doGravity = params.gravity.lengthSquared() > 0;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		T& WorldT<T>::getGravity() {
			return params.gravity;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>*  WorldT<T>::setTimeStep(float timeStep) {
			params.timeStep = timeStep;
			params.timeStep2 = timeStep * timeStep;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>*  WorldT<T>::setNumIterations(float numIterations) {
			params.numIterations = numIterations;
			return this;
		}
		
		
		template <typename T>
		WorldT<T>* WorldT<T>::setWorldMin(T worldMin) {
			params.worldMin		= worldMin;
			params.worldSize	= params.worldMax - params.worldMin;
			params.doWorldEdges	= true;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::setWorldMax(T worldMax) {
			params.worldMax		= worldMax;
			params.worldSize	= params.worldMax - params.worldMin;
			params.doWorldEdges = true;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::setWorldSize(T worldMin, T worldMax) {
			setWorldMin(worldMin);
			setWorldMax(worldMax);
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::clearWorldSize() {
			params.doWorldEdges = false;
			disableCollision();
			return this;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::enableCollision() {
			params.isCollisionEnabled = true;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::disableCollision() {
			params.isCollisionEnabled = false;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		bool WorldT<T>::isCollisionEnabled() {
			return params.isCollisionEnabled;
		}
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::setSectorCount(int count) {
			T r;
			for(int i=0; i<T::DIM; i++) r[i] = count;
			setSectorCount(r);
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::setSectorCount(T vCount) {
			for(int i=0; i<T::DIM; i++) if(vCount[i] <= 0) vCount[i] = 1;
			
			params.sectorCount = vCount;
			
			//	params.sectorCount.x = 1 << (int)vPow.x;
			//	params.sectorCount.y = 1 << (int)vPow.y;
			//	params.sectorCount.z = 1 << (int)vPow.z;
			
			//	T sectorSize = params.worldSize / sectorCount;
			
			for(typename vector<SectorT<T>*>::iterator it = _sectors.begin(); it != _sectors.end(); it++) {
				SectorT<T>* sector = *it;
				sector->release();
			}
			_sectors.clear();
			
			
			int numSectors = 1;
			for(int i=0; i<T::DIM; i++) numSectors *= params.sectorCount[i];
			for(int i=0; i<numSectors; i++) {
				_sectors.push_back(new SectorT<T>);
			}
			//	_sectors.reserve(params.sectorCount.x * params.sectorCount.y * params.sectorCount.z);
			
			return this;
		}
		
		
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>*  WorldT<T>::setParticleCount(long i) {
			_particles.reserve(i);
#ifdef MSAPHYSICS_USE_RECORDER
			//	if(_replayMode == OFX_MSA_DATA_SAVE)
			_recorder.setSize(i);
#endif
			return this;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::setConstraintCount(long i){
			_constraints[kConstraintTypeCustom].reserve(i);
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::setSpringCount(long i){
			_constraints[kConstraintTypeSpring].reserve(i);
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		WorldT<T>* WorldT<T>::setAttractionCount(long i){
			_constraints[kConstraintTypeAttraction].reserve(i);
			return this;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		void WorldT<T>::clear() {
			for(typename vector<ParticleT<T>*>::iterator it = _particles.begin(); it != _particles.end(); it++) {
				ParticleT<T>* particle = *it;
				particle->release();
			}
			_particles.clear();
			
			
			for(int i=0; i<kConstraintTypeCount; i++) {
				for(typename vector<ConstraintT<T>*>::iterator it = _constraints[i].begin(); it != _constraints[i].end(); it++) {
					ConstraintT<T>* constraint = *it;
					constraint->release();
				}
				_constraints[i].clear();
			}
			
			//			for(typename vector<SectorT<T>*>::iterator it = _sectors.begin(); it != _sectors.end(); it++) {
			//				SectorT<T>* sector = *it;
			//				sector->release();
			//			}
			//			_sectors.clear();
			
			
		}
		
		
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		void WorldT<T>::update(int frameNum) {
#ifdef MSAPHYSICS_USE_RECORDER
			if(frameNum < 0) frameNum = _frameCounter;
			if(_replayMode == OFX_MSA_DATA_LOAD) {
				load(frameNum);
			} else {
				updateParticles();
				updateConstraints();
				if(isCollisionEnabled()) checkAllCollisions();
				if(_replayMode == OFX_MSA_DATA_SAVE) _recorder.save(frameNum);
			}
			_frameCounter++;
#else
			updateParticles();
			updateConstraints();
			if(isCollisionEnabled()) checkAllCollisions();
#endif
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		void WorldT<T>::draw() {
			for(int i=0; i<kConstraintTypeCount; i++) {
				for(typename vector<ConstraintT<T>*>::iterator it = _constraints[i].begin(); it != _constraints[i].end(); it++) {
					ConstraintT<T>* constraint = *it;
					constraint->draw();
				}
			}
			
			for(typename vector<ParticleT<T>*>::iterator it = _particles.begin(); it != _particles.end(); it++) {
				ParticleT<T>* particle = *it;
				particle->draw();
			}
		}
		
		//--------------------------------------------------------------
        template <typename T>
		void WorldT<T>::debugDraw() {
			for(int i=0; i<kConstraintTypeCount; i++) {
				for(typename vector<ConstraintT<T>*>::iterator it = _constraints[i].begin(); it != _constraints[i].end(); it++) {
					ConstraintT<T>* constraint = *it;
					constraint->debugDraw();
				}
			}
			
			for(typename vector<ParticleT<T>*>::iterator it = _particles.begin(); it != _particles.end(); it++) {
				ParticleT<T>* particle = *it;
				particle->debugDraw();
			}
		}
		
        //--------------------------------------------------------------
#ifdef MSAPHYSICS_USE_RECORDER
		template <typename T>
		void WorldT<T>::load(long frameNum) {
			_recorder.load(frameNum);
			for(vector<ParticleT<T>*>::iterator it = _particles.begin(); it != _particles.end(); it++) {
				ParticleT<T>* particle = *it;
				particle->set(_recorder.get());// * _playbackScaler);
			}
		}
#endif
		
        //--------------------------------------------------------------
		template <typename T>
		void WorldT<T>::updateParticles() {
			int num = 0;
			typename vector<ParticleT<T>*>::iterator it = _particles.begin();
			while(it != _particles.end()) {
				ParticleT<T>* particle = *it;
				if(particle->_isDead) {							// if particle is dead
					it = _particles.erase(it);
					particle->release();
				} else {
					num++;
					particle->doVerlet();
					particle->update();
					this->applyUpdaters(particle);
					if(params.doWorldEdges) {
						//				if(particle->isFree()) 
						particle->checkWorldEdges();
					}
					
					// find which sector particle is in
					//					int i = mapRange(particle->getX(), params.worldMin.x, params.worldMax.x, 0.0f, params.sectorCount.x, true);
					//					int j = mapRange(particle->getY(), params.worldMin.y, params.worldMax.y, 0.0f, params.sectorCount.y, true);
					//					int k = mapRange(particle->getZ(), params.worldMin.z, params.worldMax.z, 0.0f, params.sectorCount.z, true);
					
					if(isCollisionEnabled()) {
						int sectorIndex=0;
						for(int i=0; i<T::DIM; i++) {
							int t = params.sectorCount[i] ? mapRange(particle->getPosition()[i], params.worldMin[i], params.worldMax[i], 0.0f, params.sectorCount[1]-1, true) : 0;
							
							// TODO:
							//						for(int j=0; j<i; j++) {
							//							t *= params.sectorCount[i];
							//						}
							//						sectorIndex += t;
							
						}
						
						_sectors[sectorIndex]->addParticle(particle);
					}
					
					//					_sectors[i * params.sectorCount.y * params.sectorCount.x + j * params.sectorCount.x + k]->addParticle(particle);
					
					//			printf("sector for particle at %f, %f, %f is %i %i %i\n", particle->getX(), particle->getY(), particle->getZ(), i, j, k);
					//			for(int s=0; s<_sectors.size(); s++) _sectors[s].checkParticle(particle);
					
#ifdef MSAPHYSICS_USE_RECORDER
					if(_replayMode == OFX_MSA_DATA_SAVE) _recorder.add(*particle);
#endif
					it++;
				}
			}
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		void WorldT<T>::updateConstraintsByType(vector<ConstraintT<T>*> constraints) {
		}
		
        //--------------------------------------------------------------
		template <typename T>
		void WorldT<T>::updateConstraints() {
			// iterate all constraints and update
			for (int i = 0; i < params.numIterations; i++) {
				for(int i=0; i<kConstraintTypeCount; i++) {
					typename vector<ConstraintT<T>*>::iterator it = _constraints[i].begin();
					while(it != _constraints[i].end()) {
						ConstraintT<T>* constraint = *it;
						if(constraint->_isDead || constraint->_a->_isDead || constraint->_b->_isDead) {
							constraint->kill();
							it = _constraints[i].erase(it);
							constraint->release();
						} else {
							if(constraint->shouldSolve()) constraint->solve();
							it++;
						}
					}
					
				}
			}
		}
		
		
        //--------------------------------------------------------------
#ifdef MSAPHYSICS_USE_RECORDER
		template <typename T>
		WorldT<T>*  WorldT<T>::setReplayMode(long i, float playbackScaler) {
			_replayMode = i;
			_playbackScaler = playbackScaler;
			//	if(_replayMode == OFX_MSA_DATA_SAVE)		// NEW
			_recorder.setSize(i);
			return this;
		}
		
		
		WorldT<T>*  WorldT<T>::setReplayFilename(string f) {
			_recorder.setFilename(f);
			return this;
		}
#endif
		
		
        //--------------------------------------------------------------
		template <typename T>
		void WorldT<T>::checkAllCollisions() {
			int s = _sectors.size();
			for(int i=0; i<s; i++) {
				_sectors[i]->checkSectorCollisions();
				_sectors[i]->clear();
			}
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		ConstraintT<T>* WorldT<T>::getConstraint(ParticleT<T> *a, ParticleT<T> *b, int constraintType) {
			for(typename vector<ConstraintT<T>*>::iterator it = _constraints[constraintType].begin(); it != _constraints[constraintType].end(); it++) {
				ConstraintT<T>* s = *it;
				if(((s->_a == a && s->_b == b) || (s->_a == b && s->_b == a)) && !s->_isDead) {
					return s;
				}
			}
			return NULL;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		ConstraintT<T>* WorldT<T>::getConstraint(ParticleT<T> *a, int constraintType) {
			for(typename vector<ConstraintT<T>*>::iterator it = _constraints[constraintType].begin(); it != _constraints[constraintType].end(); it++) {
				ConstraintT<T>* s = *it;
				if (((s->_a == a ) || (s->_b == a)) && !s->_isDead) {
					return s;
				}
			}
			return NULL;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		ParamsT<T>&	WorldT<T>::getParams() {
			return params;
		}
	}
	
}