#pragma once

#include "MSACore.h"
#include "MSAPhysicsConstraint.h"
#include "MSAPhysicsTypes.h"

namespace msa {
namespace physics {

template <typename T>
class SpringT : public ConstraintT<T> {
public:
    typedef shared_ptr< WorldT<T> >           World_ptr;
    typedef shared_ptr< SectorT<T> >          Sector_ptr;
    typedef shared_ptr< ParamsT<T> >          Params_ptr;
    typedef shared_ptr< ParticleT<T> >        Particle_ptr;
    typedef shared_ptr< SpringT<T> >          Spring_ptr;
    typedef shared_ptr< AttractionT<T> >      Attraction_ptr;
    typedef shared_ptr< ConstraintT<T> >      Constraint_ptr;

    // create an instance of this class and return a smart pointer
    // this is the only way to instantiate this class
    static Spring_ptr create(Particle_ptr a, Particle_ptr b, float strength, float restLength) {
        return Spring_ptr(new SpringT<T>(a, b, strength, restLength));
    }

    Spring_ptr          setStrength(float s)            { _strength = s; return getThis(); }
    float               getStrength() const             { return _strength; }

    Spring_ptr          setForceCap (float c)           { _forceCap = c; return getThis(); }
    float               getForceCap () const            { return _forceCap; }

    Spring_ptr          setRestLength(float l)          { _restLength = l; return getThis(); }
    float               getRestLength() const           { return _restLength; }

    Spring_ptr          getThis()                       { return _isInited ? dynamic_pointer_cast< SpringT<T> >(this->shared_from_this()) : Spring_ptr(); }

    void solve() override {
        T delta = this->_b->getPosition() - this->_a->getPosition();
        float deltaLength2 = delta.lengthSquared();
        float deltaLength = sqrt(deltaLength2);	// TODO: fast approximation of square root (1st order Taylor-expansion at a neighborhood of the rest length r (one Newton-Raphson iteration with initial guess r))
        float force = deltaLength > 0 ? _strength * (deltaLength - _restLength) / (deltaLength * (this->_a->getInvMass() + this->_b->getInvMass())) : 0;

        T deltaForce(delta * force);
        if (_forceCap > 0) deltaForce.limit(_forceCap);
        if (this->_a->isFree()) this->_a->moveBy(deltaForce * this->_a->getInvMass(), false);
        if (this->_b->isFree()) this->_b->moveBy(deltaForce * -this->_b->getInvMass(), false);
    }

protected:
    float _restLength;
    float _strength;
    float _forceCap;
    bool _isInited;


    SpringT(Particle_ptr a, Particle_ptr b, float strength, float restLength):
        ConstraintT<T>(a, b, kConstraintTypeSpring)
    {
        _isInited = false;
        setStrength(strength);
        setRestLength(restLength);
        setForceCap(0);
        _isInited = true;
    }

    void debugDraw() {
        ConstraintT<T>::debugDraw();
    }

};

}
}
