#pragma once

#include "MSAPhysics.h"


namespace msa {
namespace physics {

template <typename T>
class WorldT : public enable_shared_from_this< WorldT<T> > {
public:
    typedef shared_ptr< WorldT<T> >           World_ptr;
    typedef shared_ptr< SectorT<T> >          Sector_ptr;
    typedef shared_ptr< ParamsT<T> >          Params_ptr;
    typedef shared_ptr< ParticleT<T> >        Particle_ptr;
    typedef shared_ptr< SpringT<T> >          Spring_ptr;
    typedef shared_ptr< AttractionT<T> >      Attraction_ptr;
    typedef shared_ptr< ConstraintT<T> >      Constraint_ptr;

    //    friend class ParticleT<T>;

    ~WorldT() { clear(); }

    static World_ptr create()                        { return World_ptr(new WorldT<T>); }

    Particle_ptr     makeParticle(const T& pos = T(), float mass = 1.0f, float drag = 1.0f);
    Spring_ptr		makeSpring(Particle_ptr a, Particle_ptr b, float strength, float restLength);
    Attraction_ptr	makeAttraction(Particle_ptr a, Particle_ptr b, float strength);

    Particle_ptr     addParticle(Particle_ptr p)        { _particles.push_back(p); return p; }
    Constraint_ptr   addConstraint(Constraint_ptr c)    { _constraints[c->type()].push_back(c); return c; }

    Particle_ptr     getParticle(long i)             { return i < numberOfParticles() ? _particles[i] : nullptr; }
    Spring_ptr		getSpring(long i)               { return i < numberOfSprings() ? dynamic_pointer_cast< SpringT<T> >(_constraints[kConstraintTypeSpring][i]) : nullptr; }
    Attraction_ptr	getAttraction(long i)           { return i < numberOfAttractions() ? dynamic_pointer_cast< AttractionT<T> >(_constraints[kConstraintTypeAttraction][i]) : nullptr; }

    // findConstraint between particles. these do a search, so not instant, has some overheads
    Constraint_ptr	findConstraint(weak_ptr<ParticleT<T>> a, int constraintType);
    Constraint_ptr	findConstraint(weak_ptr<ParticleT<T>> a, weak_ptr<ParticleT<T>> b, int constraintType);

    long				numberOfParticles()             { return _particles.size(); }
    long				numberOfCustomConstraints()     { return _constraints[kConstraintTypeCustom].size(); }
    long				numberOfSprings()               { return _constraints[kConstraintTypeSpring].size(); }
    long				numberOfAttractions()           { return _constraints[kConstraintTypeAttraction].size(); }

    // Drag. 1: no drag at all, 0.9: quite a lot of drag, 0: particles can't even move
    World_ptr		setDrag(float drag = 0.99f)     { _params->drag = drag; return getThis(); }
    float               getDrag() const                 { return _params->drag; }

    // set gravity (y component only)
    World_ptr		setGravity(float gy = 0);

    // set gravity (full vector)
    World_ptr		setGravity(const T& g);
    const T&			getGravity() const              { return _params->gravity; }

    World_ptr		setTimeStep(float t)            { _params->timeStep = t; _params->timeStep2 = t*t; return getThis(); }
    World_ptr		setNumIterations(float n = 20)  { _params->numIterations = n; return getThis(); }

    // for optimized collision, set world dimensions first
    World_ptr		setWorldMin(const T& worldMin)  { _params->worldMin = worldMin; updateWorldSize(); return getThis(); }
    World_ptr		setWorldMax(const T& worldMax)  { _params->worldMax = worldMax; updateWorldSize(); return getThis(); }
    World_ptr		setWorldSize(const T& worldMin, const T& worldMax)  { setWorldMin(worldMin); setWorldMax(worldMax); return getThis(); }
    World_ptr		clearWorldSize()                { _params->doWorldEdges = false; disableCollision(); return getThis(); }

    // and then set sector size (or count)
    World_ptr		enableCollision()               { _params->isCollisionEnabled = true; return getThis(); }
    World_ptr		disableCollision()              { _params->isCollisionEnabled = false; return getThis(); }
    bool				isCollisionEnabled() const      { return _params->isCollisionEnabled; }
    World_ptr		setSectorCount(int count);		// set the number of sectors (will be equal in each axis)
    World_ptr		setSectorCount(T vCount);// set the number of sectors in each axis

    // preallocate buffers if you know how big they need to be (they grow automatically if need be)
    World_ptr		setParticleCount(long i);
    World_ptr		setCustomConstraintCount(long i);
    World_ptr		setSpringCount(long i);
    World_ptr		setAttractionCount(long i);


    void clear();
    void update(int frameNum = -1);
    void draw();
    void debugDraw();

#ifdef MSAPHYSICS_USE_RECORDER
    World_ptr			setReplayMode(int i, float playbackScaler = 1.0f);		// when playing back recorded data, optionally scale positions up (so you can record in lores, playback at highres)
    World_ptr			setReplayFilename(string f);
#endif

    Params_ptr			getParams() const           { return _params; }

    World_ptr           getThis()                   { return _isInited ? this->shared_from_this() : World_ptr(); }

protected:
    Params_ptr                           _params;
    vector< Particle_ptr >               _particles;
    map<int, vector< Constraint_ptr > >  _constraints;    // key: constraint type, value: vector of constraints
    vector< Sector_ptr >                 _sectors;

    bool _isInited;

    WorldT();

    void	updateParticles();
    void    updateConstraints();
    //    void	updateConstraintsByType(vector<Constraint_ptr> constraints);

    void    checkAllCollisions();

    void	updateWorldSize()                           { _params->worldSize = _params->worldMax - _params->worldMin; _params->doWorldEdges	= true; }


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
    _isInited = false;

    _params = make_shared< ParamsT<T> >();
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

    _isInited = true;
}


//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::Particle_ptr WorldT<T>::makeParticle(const T& pos, float mass, float drag) {
    return addParticle(ParticleT<T>::create(pos, mass, drag));
}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::Spring_ptr WorldT<T>::makeSpring(Particle_ptr a, Particle_ptr b, float strength, float restLength) {
    if(a==b) return nullptr;
    auto c = SpringT<T>::create(a, b, strength, restLength);
    addConstraint(c);
    return c;
}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::Attraction_ptr WorldT<T>::makeAttraction(Particle_ptr a, Particle_ptr b, float strength) {
    if(a==b) return nullptr;
    auto c = AttractionT<T>::create(a, b, strength);
    addConstraint(c);
    return c;
}



////--------------------------------------------------------------
//template <typename T>
//typename WorldT<T>::Particle_ptr WorldT<T>::addParticle(Particle_ptr p) {
//    _particles.push_back(p); return p;
//}

////--------------------------------------------------------------
//template <typename T>
//typename WorldT<T>::Constraint_ptr WorldT<T>::addConstraint(Constraint_ptr c) {
//    _constraints[c->type()].push_back(c); return c;
//}


//--------------------------------------------------------------
//template <typename T>
//Constraint_ptr WorldT<T>::getConstraint(long i) {
//    return i < numberOfConstraints() ? _constraints[kConstraintTypeCustom][i] : nullptr;
//}


//--------------------------------------------------------------
//template <typename T>
//Attraction_ptr WorldT<T>::getAttraction(long i) {
//    return i < numberOfAttractions() ? (Attraction_ptr)_constraints[kConstraintTypeAttraction][i] : nullptr;
//}


//--------------------------------------------------------------
//template <typename T>
//long WorldT<T>::numberOfParticles() {
//    return _particles.size();
//}

//--------------------------------------------------------------
//template <typename T>
//long WorldT<T>::numberOfConstraints() {
//    return _constraints[kConstraintTypeCustom].size();
//}

//--------------------------------------------------------------
//template <typename T>
//long WorldT<T>::numberOfSprings() {
//    return _constraints[kConstraintTypeSpring].size();
//}

//--------------------------------------------------------------
//template <typename T>
//long WorldT<T>::numberOfAttractions() {
//    return _constraints[kConstraintTypeAttraction].size();
//}_sectors


//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::setDrag(float drag) {
//    params->drag = drag; return getThis();
//}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setGravity(float gy) {
    T g(T::zero());
    g[1] = gy;
    setGravity(g);
    return getThis();
}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setGravity(const T& g) {
    _params->gravity= g;
    _params->doGravity = _params->gravity.lengthSquared() > 0;
    return getThis();
}

//--------------------------------------------------------------
//template <typename T>
//const T& WorldT<T>::getGravity() const {
//    return _params->gravity;
//}

//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::setTimeStep(float t) {
//    params->timeStep = t; params->timeStep2 = t*t; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::setNumIterations(float numIterations) {
//    params->numIterations = numIterations; return getThis();
//}


//template <typename T>
//World_ptr WorldT<T>::setWorldMin(const T& worldMin) {
//    _params->worldMin = worldMin; updateWorldSize(); return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::setWorldMax(const T& worldMax) {
//    _params->worldMax = worldMax; updateWorldSize(); return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::setWorldSize(const T& worldMin, const T& worldMax) {
//    setWorldMin(worldMin); setWorldMax(worldMax); return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::clearWorldSize() {
//    _params->doWorldEdges = false; disableCollision(); return getThis();
//}


//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::enableCollision() {
//    _params->isCollisionEnabled = true; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//World_ptr WorldT<T>::disableCollision() {
//    _params->isCollisionEnabled = false; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//bool WorldT<T>::isCollisionEnabled() const {
//    return _params->isCollisionEnabled;
//}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setSectorCount(int count) {
    T r;
    for(int i=0; i<T::DIM; i++) r[i] = count;
    setSectorCount(r);
    return getThis();
}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setSectorCount(T vCount) {
    for(int i=0; i<T::DIM; i++) if(vCount[i] <= 0) vCount[i] = 1;

    _params->sectorCount = vCount;

    //	params.sectorCount.x = 1 << (int)vPow.x;
    //	params.sectorCount.y = 1 << (int)vPow.y;
    //	params.sectorCount.z = 1 << (int)vPow.z;

    //	T sectorSize = params.worldSize / sectorCount;
    _sectors.clear();

    int numSectors = 1;
    for(int i=0; i<T::DIM; i++) numSectors *= _params->sectorCount[i];
    for(int i=0; i<numSectors; i++) _sectors.push_back(SectorT<T>::create());
    return getThis();
}





//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setParticleCount(long i) {
    _particles.reserve(i);
#ifdef MSAPHYSICS_USE_RECORDER
    //	if(_replayMode == OFX_MSA_DATA_SAVE)
    _recorder.setSize(i);
#endif
    return getThis();
}


//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setCustomConstraintCount(long i){
    _constraints[kConstraintTypeCustom].reserve(i);
    return getThis();
}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setSpringCount(long i){
    _constraints[kConstraintTypeSpring].reserve(i);
    return getThis();
}

//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::World_ptr WorldT<T>::setAttractionCount(long i){
    _constraints[kConstraintTypeAttraction].reserve(i);
    return getThis();
}


//--------------------------------------------------------------
template <typename T>
void WorldT<T>::clear() {
    _particles.clear();
    _constraints.clear();
    for(auto s: _sectors) s->clear();
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
    for(auto&& vc : _constraints) for(auto&& c : vc.second) c->draw();
    for(auto&& p : _particles) p->draw();
}

//--------------------------------------------------------------
template <typename T>
void WorldT<T>::debugDraw() {
    for(auto&& vc : _constraints) for(auto&& c : vc->second) c->debugDraw();
    for(auto&& p : _particles) p->debugDraw();
}

//--------------------------------------------------------------
#ifdef MSAPHYSICS_USE_RECORDER
template <typename T>
void WorldT<T>::load(long frameNum) {
    _recorder.load(frameNum);
    for(vector<Particle_ptr>::iterator it = _particles.begin(); it != _particles.end(); it++) {
        Particle_ptr particle = *it;
        p->set(_recorder.get());// * _playbackScaler);
    }
}
#endif

//--------------------------------------------------------------
template <typename T>
void WorldT<T>::updateParticles() {

    // remove dead particles first
    _particles.erase( remove_if(_particles.begin(), _particles.end(), [](const Particle_ptr &o) { return o->isDead(); }), _particles.end());

    // update remaining particles
    for(auto&& p : _particles) {
        // do verlet
        {
            if(p->isFree()) {
                if(_params->doGravity) p->addVelocity(_params->gravity);

                T curPos(p->getPosition());
                T vel(p->getVelocity());
                p->moveBy(vel * _params->drag * p->getDrag());// + timeStep2;
                //_pos += (_pos - _oldPos);// + timeStep2;	// TODO
                p->setOldPosition(curPos);
            }
        }

        p->update();
        //        this->applyUpdaters(particle);    // TODO: bring back updaters
        if(_params->doWorldEdges) {
            //				if(p->isFree())
            bool collided = false;
            T vel(p->getVelocity());
            T pos(p->getPosition());
            T oldPos;
            float radius = p->getRadius();
            float bounce = p->getBounce();
            for(int i=0; i<T::DIM; i++) {
                //				r[i] = _radius;

                float speed = vel[i];
                if(pos[i] < _params->worldMin[i] + radius) {
                    pos[i] = _params->worldMin[i] + radius;
                    oldPos[i] = pos[i] + speed * bounce;
                    collided = true;
                } else if(pos[i] > _params->worldMax[i] - radius) {
                    pos[i] = _params->worldMax[i] - radius;
                    oldPos[i] = pos[i] + speed * bounce;
                    collided = true;
                }
            }
            p->moveTo(pos);
            p->setOldPosition(oldPos);

            if(collided) p->collidedWithEdgeOfWorld(p->getVelocity() - vel);
        }

        // find which sector particle is in
        //					int i = mapRange(p->getX(), _params->worldMin.x, _params->worldMax.x, 0.0f, _params->sectorCount.x, true);
        //					int j = mapRange(p->getY(), _params->worldMin.y, _params->worldMax.y, 0.0f, _params->sectorCount.y, true);
        //					int k = mapRange(p->getZ(), _params->worldMin.z, _params->worldMax.z, 0.0f, _params->sectorCount.z, true);

        if(isCollisionEnabled()) {
            int sectorIndex = 0;
            for(int i=0; i<T::DIM; i++) {
                int t = _params->sectorCount[i] ? mapRange(p->getPosition()[i], _params->worldMin[i], _params->worldMax[i], 0.0f, _params->sectorCount[1]-1, true) : 0;

                // TODO:
                //						for(int j=0; j<i; j++) {
                //							t *= _params->sectorCount[i];
                //						}
                //						sectorIndex += t;

            }

            _sectors[sectorIndex]->addParticle(p);
        }

        //					_sectors[i * _params->sectorCount.y * _params->sectorCount.x + j * _params->sectorCount.x + k]->addParticle(particle);

        //			printf("sector for particle at %f, %f, %f is %i %i %i\n", p->getX(), p->getY(), p->getZ(), i, j, k);
        //			for(int s=0; s<_sectors.size(); s++) _sectors[s].checkParticle(particle);

#ifdef MSAPHYSICS_USE_RECORDER
        if(_replayMode == OFX_MSA_DATA_SAVE) _recorder.add(*particle);
#endif
    }
}


//--------------------------------------------------------------
//template <typename T>
//void WorldT<T>::updateConstraintsByType(vector<Constraint_ptr> constraints) {
//}


//--------------------------------------------------------------
template <typename T>
void WorldT<T>::updateConstraints() {

    // remove constraints if dead
    for(auto&& v : _constraints) {
        v.second.erase( remove_if(v.second.begin(), v.second.end(), [](const Constraint_ptr &c) { return c->isDead(); }), v.second.end());
    }

    // iterations
    for (int n=0; n<_params->numIterations; n++) {

        // iterate constraint types
        for(auto&& v : _constraints) {

            // iterate constraints
            for(auto&& c : v.second) {
                if(c->shouldSolve()) c->solve();
            }

        }
    }
}


//--------------------------------------------------------------
#ifdef MSAPHYSICS_USE_RECORDER
template <typename T>
World_ptr  WorldT<T>::setReplayMode(long i, float playbackScaler) {
    _replayMode = i;
    _playbackScaler = playbackScaler;
    //	if(_replayMode == OFX_MSA_DATA_SAVE)		// NEW
    _recorder.setSize(i);
    return this;
}


World_ptr  WorldT<T>::setReplayFilename(string f) {
    _recorder.setFilename(f);
    return this;
}
#endif


//--------------------------------------------------------------
template <typename T>
void WorldT<T>::checkAllCollisions() {
    for(auto&& s : _sectors) {
        s->checkSectorCollisions();
        s->clear();
    }
}


//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::Constraint_ptr WorldT<T>::findConstraint(weak_ptr<ParticleT<T>> a, weak_ptr<ParticleT<T>> b, int constraintType) {
    for(auto&& constraint : _constraints[constraintType]) {
        if(((constraint->_a == a && constraint->_b == b) || (constraint->_a == b && constraint->_b == a)) && !constraint->_isDead) {
            return constraint;
        }
    }
    return nullptr;
}


//--------------------------------------------------------------
template <typename T>
typename WorldT<T>::Constraint_ptr WorldT<T>::findConstraint(weak_ptr<ParticleT<T>> a, int constraintType) {
    for(auto&& constraint : _constraints[constraintType]) {
        if (((constraint->_a == a ) || (constraint->_b == a)) && !constraint->_isDead) {
            return constraint;
        }
    }
    return nullptr;
}


//--------------------------------------------------------------
//template <typename T>
//Params_ptr WorldT<T>::getParams() const {
//    return params;
//}

}
}

