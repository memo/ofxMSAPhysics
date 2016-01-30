#pragma once

#include "MSACore.h"
#include "MSAPhysicsParticle.h"
#include "MSAPhysicsTypes.h"

namespace msa {
namespace physics {

typedef enum ConstraintType {
    kConstraintTypeCustom,
    kConstraintTypeSpring,
    kConstraintTypeAttraction,
    kConstraintTypeCount,
} ConstraintType;



template <typename T>
class ConstraintT : public enable_shared_from_this< ConstraintT<T> >{
public:
    typedef shared_ptr< WorldT<T> >           World_ptr;
    typedef shared_ptr< SectorT<T> >          Sector_ptr;
    typedef shared_ptr< ParamsT<T> >          Params_ptr;
    typedef shared_ptr< ParticleT<T> >        Particle_ptr;
    typedef shared_ptr< SpringT<T> >          Spring_ptr;
    typedef shared_ptr< AttractionT<T> >      Attraction_ptr;
    typedef shared_ptr< ConstraintT<T> >      Constraint_ptr;

//    friend class WorldT<T>;

    // virtual destructor needed in case we extend the class and delete via the base class
    virtual ~ConstraintT() {}

    int type()          const                           { return _type; }

    // getOneEnd is a same as getA and getTheOtherEnd is same as getB
    // just have both methods so you can choose whichever you please
//    Particle_ptr getOneEnd() const                   { return _a; }
//    Particle_ptr getTheOtherEnd() const              { return _b; }

    Particle_ptr getA() const                           { return _a; }
    Particle_ptr getB() const                           { return _b; }

    void turnOff()                                      { _isOn = false; }
    void turnOn()                                       { _isOn = true; }

    bool isOn() const                                   { return (_isOn == true); }
    bool isOff() const                                  { return (_isOn == false); }

    void kill()                                         { _isDead = true; }
    bool isDead() const                                 { return _isDead || (_a && _a->isDead()) || (_b && _b->isDead()) || !_a || !_b; }

    // set minimum distance before constraint takes affect
    void setMinDistance(float d)                        { _minDist = d; _minDist2 = d*d; }

    // get minimum distance
    float getMinDistance() const                        { return _minDist; }

    // set maximum distance before constraint takes affect
    void setMaxDistance(float d)                        { _maxDist = d; _maxDist2 = d*d; }

    // get maximum distance
    float getMaxDistance() const                        { return _maxDist; }

    // only worth solving the constraint if its on, and at least one end is free
    bool shouldSolve() const;

    virtual void update() {}
    virtual void draw() {}

    virtual void solve() = 0;

protected:
    Particle_ptr _a, _b;
    ConstraintType	_type;

    bool			_isOn;
    bool			_isDead;
    float			_minDist;
    float			_minDist2;
    float			_maxDist;
    float			_maxDist2;

    ConstraintT(Particle_ptr a, Particle_ptr b, ConstraintType type = kConstraintTypeCustom):
        _a(a), _b(b), _type(type), _isOn(true), _isDead(false)

    {
        setMinDistance(0);
        setMaxDistance(0);
    }


    virtual void debugDraw() {
        //ofLine(_a->x, _a->y, _b->x, _b->y);
        /*
                 T vec = (*_b - *_a);
                 float dist = msaLength(vec);
                 float angle = acos( vec.z / dist ) * RAD_TO_DEG;
                 if(vec.z <= 0 ) angle = -angle;
                 float rx = -vec.y * vec.z;
                 float ry =  vec.x * vec.z;

                 glPushMatrix();
                 glTranslatef(_a->x, _a->y, _a->z);
                 glRotatef(angle, rx, ry, 0.0);
                 glScalef(1, 1, dist);
                 glTranslatef(0, 0, 0.5);
                 #ifndef TARGE_OF_IPHONE
                 glutSolidCube(1);
                 #endif
                 glPopMatrix();
                 */
    }
};



//--------------------------------------------------------------
//template <typename T>
//int ConstraintT<T>::type() const {
//    return _type;
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ConstraintT<T>::getOneEnd() const {
//    return _a;
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ConstraintT<T>::getTheOtherEnd() const {
//    return _b;
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ConstraintT<T>::getA() const {
//    return _a;
//}

//--------------------------------------------------------------
//template <typename T>
//Particle_ptr ConstraintT<T>::getB() const {
//    return _b;
//}

//--------------------------------------------------------------
//template <typename T>
//void ConstraintT<T>::turnOff() {
//    _isOn = false;
//}

//--------------------------------------------------------------
//template <typename T>
//void ConstraintT<T>::turnOn() {
//    _isOn = true;
//}

//--------------------------------------------------------------
//template <typename T>
//bool ConstraintT<T>::isOn() const {
//    return (_isOn == true);
//}

//--------------------------------------------------------------
//template <typename T>
//bool ConstraintT<T>::isOff() const {
//    return (_isOn == false);
//}

//--------------------------------------------------------------
//template <typename T>
//void ConstraintT<T>::kill() {
//    _isDead = true;
//}

//--------------------------------------------------------------
//template <typename T>
//bool ConstraintT<T>::isDead() const {
//    return _isDead;
//}


//--------------------------------------------------------------
//template <typename T>
//void ConstraintT<T>::setMinDistance(float d) {
//    _minDist = d; _minDist2 = d*d;
//}

//--------------------------------------------------------------
//template <typename T>
//float ConstraintT<T>::getMinDistance() const {
//    return _minDist;
//}

//--------------------------------------------------------------
//template <typename T>
//void ConstraintT<T>::setMaxDistance(float d) {
//    _maxDist = d; _maxDist2 = d*d;
//}

//--------------------------------------------------------------
//template <typename T>
//float ConstraintT<T>::getMaxDistance() const {
//    return _maxDist;
//}

//--------------------------------------------------------------
// only worth solving the constraint if its on, and at least one end is free
template <typename T>
bool ConstraintT<T>::shouldSolve() const {

    // if the constraint is off or both sides are fixed then return false
    if(isOff() || (_a->isFixed() && _b->isFixed())) return false;

    // if no length restrictions then return true
    if(_minDist == 0 && _maxDist == 0) return true;

    T delta(_b->getPosition() - _a->getPosition());
    float deltaLength2 = delta.lengthSquared();

    bool minDistSatisfied;
    if(_minDist) minDistSatisfied = deltaLength2 > _minDist2;
    else minDistSatisfied = true;

    bool maxDistSatisfied;
    if(_maxDist) maxDistSatisfied = deltaLength2 < _maxDist2;
    else maxDistSatisfied = true;

    return minDistSatisfied && maxDistSatisfied;
}

}
}
