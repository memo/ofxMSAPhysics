#pragma once

#include "MSACore.h"
#include "MSAPhysicsConstraint.h"
#include "MSAPhysicsTypes.h"


namespace msa {
namespace physics {

template <typename T>
class AttractionT : public ConstraintT<T> {
public:
    typedef shared_ptr< WorldT<T> >           World_ptr;
    typedef shared_ptr< SectorT<T> >          Sector_ptr;
    typedef shared_ptr< ParamsT<T> >          Params_ptr;
    typedef shared_ptr< ParticleT<T> >        Particle_ptr;
    typedef shared_ptr< SpringT<T> >          Spring_ptr;
    typedef shared_ptr< AttractionT<T> >      Attraction_ptr;
    typedef shared_ptr< ConstraintT<T> >      Constraint_ptr;

    //    friend class WorldT<T>;

    // create an instance of this class and return a smart pointer
    // this is the only way to instantiate this class
    static Attraction_ptr create(Particle_ptr a, Particle_ptr b, float strength) {
        return Attraction_ptr(new AttractionT<T>(a, b, strength));
    }


    Attraction_ptr setStrength(float s)              { _strength = s; return dynamic_pointer_cast< AttractionT<T> >(this->shared_from_this()); }
    float getStrength() const                           { return _strength; }


    void solve() override {
        T delta(this->_b->getPosition() - this->_a->getPosition());
        float deltaLength2 = delta.lengthSquared();
        float force = deltaLength2 > 0 ? _strength * (this->_b->getMass()) * (this->_a->getMass()) / deltaLength2 : 0;

        T deltaForce(delta * force);
        if (this->_a->isFree()) this->_a->moveBy(deltaForce * this->_a->getInvMass(), false);
        if (this->_b->isFree()) this->_b->moveBy(deltaForce * -this->_b->getInvMass(), false);
    }

protected:    float _strength;


    AttractionT(Particle_ptr a, Particle_ptr b, float strength):
        ConstraintT<T>(a, b, kConstraintTypeAttraction)
    {
        setStrength(strength);
    }


    void debugDraw() {
        ConstraintT<T>::debugDraw();
    }
};


}
}
