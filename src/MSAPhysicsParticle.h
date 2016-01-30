#pragma once

#include "MSACore.h"
#include "MSAPhysicsParams.h"
#include "MSAPhysicsTypes.h"

namespace msa {
namespace physics {

template <typename T>
class ParticleT : public enable_shared_from_this< ParticleT<T> > {
public:
    typedef shared_ptr< WorldT<T> >           World_ptr;
    typedef shared_ptr< SectorT<T> >          Sector_ptr;
    typedef shared_ptr< ParamsT<T> >          Params_ptr;
    typedef shared_ptr< ParticleT<T> >        Particle_ptr;
    typedef shared_ptr< SpringT<T> >          Spring_ptr;
    typedef shared_ptr< AttractionT<T> >      Attraction_ptr;
    typedef shared_ptr< ConstraintT<T> >      Constraint_ptr;

    // virtual destructor needed in case we extend the class and delete via the base class
    virtual ~ParticleT() {}

    // create an instance of this class and return a smart pointer
    // this is the only way to instantiate this class
    static Particle_ptr create(const T& pos = T(), float mass = 1.0f, float drag = 1.0f) {
        return Particle_ptr(new ParticleT<T>(pos, mass, drag));
    }

    virtual void        init(const T& pos = T(), float mass = 1.0f, float drag = 1.0f);

    Particle_ptr        setMass(float t = 1);
    float               getMass() const                 { return _mass; }
    float               getInvMass() const              { return _invMass; }

    Particle_ptr        setDrag(float t = 1)            { _drag = t; return getThis(); }
    float               getDrag() const                 { return _drag; }

    Particle_ptr        setBounce(float t = 1)          { _bounce = t; return getThis(); }
    float               getBounce()                     { return _bounce; }

    Particle_ptr        setRadius(float t = 15)         { _radius = t; return getThis(); }
    float               getRadius()                     { return _radius; }

    // collision methods
    Particle_ptr        enableCollision()               { _collisionEnabled = true; return getThis(); }
    Particle_ptr        disableCollision()              { _collisionEnabled = false; return getThis(); }
    bool                hasCollision() const            { return _collisionEnabled; }

    // passive particles do not collied with each other, only with non-passive (collision must be enabled)
    Particle_ptr        enablePassiveCollision()        { _passiveCollision = true; return getThis(); }
    Particle_ptr        disablePassiveCollision()       { _passiveCollision = false; return getThis(); }
    bool                hasPassiveCollision() const     { return _passiveCollision; }

    bool                isFixed() const                 { return (_isFixed); }
    bool                isFree() const                  { return (!_isFixed); }
    Particle_ptr        makeFixed()                     { _isFixed = true; return getThis(); }
    Particle_ptr        makeFree()                      { _oldPos = _pos; _isFixed = false; return getThis(); }

    // quick way of enabling (collision and update) and disabling
    Particle_ptr        enable()                        { enableCollision(); makeFree(); return getThis(); }
    Particle_ptr        disable()                       { disableCollision(); makeFixed(); return getThis(); }

    // move the particle
    // if preserveVelocity == true, the particle will move to new position and keep it's old velocity
    // if preserveVelocity == false, the particle will move to new position but gain the velocity of the displacement
    Particle_ptr        moveTo(const T& targetPos, bool preserveVelocity = true);
    Particle_ptr        moveBy(const T& offset, bool preserveVelocity = true);
    Particle_ptr        setOldPosition(const T& o);

    T                   getPosition() const             { return _pos; }

    Particle_ptr        setVelocity(const T& vel)       { _oldPos = _pos - vel; return getThis(); }
    Particle_ptr        addVelocity(const T& vel)       { _oldPos -= vel; return getThis(); }
    T                   getVelocity() const             { return _pos - _oldPos; }

    // override these functions if you create your own particle type with custom behaviour and/or drawing
    virtual void        update() {}		// called every frame in world::update();
    virtual void        draw() {}		// called every frame in world::draw();

    // called when particle collides with another particle (called for both particles) or edge of world
    virtual void        collidedWithParticle(weak_ptr<ParticleT<T>> other, const T& collisionForce) {}
    virtual void        collidedWithEdgeOfWorld(const T& collisionForce) {}

    void                kill()                          { _isDead = true; }
    bool                isDead() const                  { return _isDead; }

    Particle_ptr        getThis()                       { return _isInited ? this->shared_from_this() : Particle_ptr(); }

    // custom void* which you can use to store any kind of custom data without extending the class
    // very old school, i might scrap it
    void                *data;

    // only particles sharing bits in the collision plane collide with each other
    unsigned int        collisionPlane;

protected:
    T				_pos;
    T				_oldPos;
    float			_mass, _invMass;
    float			_drag;
    float			_bounce;
    float			_radius;
    float			_age;
    bool			_isDead;
    bool			_isFixed;
    bool			_collisionEnabled;
    bool            _passiveCollision;
    bool            _isInited;

    ParticleT(const T& pos, float mass = 1.0f, float drag = 1.0f);
    ParticleT(ParticleT& p);

    virtual void debugDraw();
};

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::create(const T& pos, float mass, float drag) {
//    return shared_Particle_ptr< ParticleT<T> >( new ParticleT<T>(pos, mass, drag) );
//}

//--------------------------------------------------------------
template <typename T>
void ParticleT<T>::init(const T& pos, float mass, float drag) {
    //    _params = nullptr;
    //    _world = nullptr;

    _pos = _oldPos = pos;
    setMass(mass);
    setDrag(drag);
    setBounce();
    setRadius();
    enableCollision();
    disablePassiveCollision();
    makeFree();
    _isDead = false;
    _age = 0;
    data = NULL;

    collisionPlane = -1;
}


//--------------------------------------------------------------
template <typename T>
typename ParticleT<T>::Particle_ptr ParticleT<T>::setMass(float m) {
    _mass = std::max(m, 0.00001f);    // can't remember why I did this, lazy way to avoid divide-by-zero later?
    _invMass = _mass > 0 ? 1.0f/_mass : 0;
    return getThis();
}

//--------------------------------------------------------------
//template <typename T>
//float ParticleT<T>::getMass() const {
//    return _mass;
//}

//--------------------------------------------------------------
//template <typename T>
//float ParticleT<T>::getInvMass() const {
//    return _invMass;
//}


//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::setDrag(float t) {
//    _drag = t; return this;
//}

//--------------------------------------------------------------
//template <typename T>
//float ParticleT<T>::getDrag() const {
//    return _drag;
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::setBounce(float t) {
//    _bounce = t; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//float ParticleT<T>::getBounce() {
//    return _bounce;
//}


//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::setRadius(float t) {
//    _radius = t; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//float ParticleT<T>::getRadius() {
//    return _radius;
//}

//--------------------------------------------------------------
//template <typename T>
//bool ParticleT<T>::isFixed() const {
//    return (_isFixed == true);
//}

//--------------------------------------------------------------
//template <typename T>
//bool ParticleT<T>::isFree() const {
//    return (_isFixed == false);
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::makeFixed() {
//    _isFixed = true; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::makeFree() {
//    _oldPos = _pos; _isFixed = false; return getThis();
//}

//--------------------------------------------------------------
template <typename T>
typename ParticleT<T>::Particle_ptr ParticleT<T>::moveTo(const T& targetPos, bool preserveVelocity) {
    T diff(targetPos - _pos);
    moveBy(diff, preserveVelocity);
    return getThis();
}

//--------------------------------------------------------------
template <typename T>
typename ParticleT<T>::Particle_ptr ParticleT<T>::moveBy(const T& offset, bool preserveVelocity) {
    _pos += offset;
    if(preserveVelocity) _oldPos += offset;
    return getThis();
}

//--------------------------------------------------------------
template <typename T>
typename ParticleT<T>::Particle_ptr ParticleT<T>::setOldPosition(const T& p) {
    _oldPos = p;
    return getThis();
}

//--------------------------------------------------------------
//template <typename T>
//const T& ParticleT<T>::getPosition() const {
//    return _pos;
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::setVelocity(const T& vel) {
//    _oldPos = _pos - vel; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::addVelocity(const T& vel) {
//    _oldPos -= vel; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//const T& ParticleT<T>::getVelocity() const {
//    return _pos - _oldPos;
//}

//--------------------------------------------------------------
//template <typename T>
//void ParticleT<T>::kill() {
//    _isDead = true;
//}


//--------------------------------------------------------------
//template <typename T>
//bool ParticleT<T>::isDead() const {
//    return _isDead;
//}

//--------------------------------------------------------------
template <typename T>
ParticleT<T>::ParticleT(const T& pos, float mass, float drag) {
    _isInited = false;
    init(pos, mass, drag);
    _isInited = true;
}

//--------------------------------------------------------------
template <typename T>
ParticleT<T>::ParticleT(ParticleT<T> &p) {
    _isInited = false;
    init(p.getPosition(), p._mass, p._drag);
    _isFixed = p._isFixed;
    setBounce(p._bounce);
    setRadius(p._radius);
    _isInited = true;
}


//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::enableCollision(){
//    _collisionEnabled = true; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::disableCollision() {
//    _collisionEnabled = false; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//bool ParticleT<T>::hasCollision() const {
//    return _collisionEnabled;
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::enablePassiveCollision() {
//    _passiveCollision = true; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::disablePassiveCollision() {
//    _passiveCollision = false; return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//bool ParticleT<T>::hasPassiveCollision() const {
//    return _passiveCollision;
//}



//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::enable(){
//    enableCollision(); makeFree(); return getThis();
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ParticleT<T>::disable() {
//    disableCollision(); makeFixed(); return getThis();
//}



//--------------------------------------------------------------
//template <typename T>
//Params_Particle_ptr<T> ParticleT<T>::getParams() {
//    return _params;
//}


//--------------------------------------------------------------
template <typename T>
void ParticleT<T>::debugDraw() {
    //			glPushMatrix();
    //			glTranslatef(_pos.x, _pos.y, _pos.z);
    //#ifndef TARGE_OS_IPHONE
    //			glutSolidSphere(_radius, 12, 12);
    //#else
    //			ofCircle(0, 0, _radius);
    //#endif
    //			glPopMatrix();
}


}
}
