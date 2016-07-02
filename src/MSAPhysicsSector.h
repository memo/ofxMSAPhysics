#pragma once

#include "MSAPhysicsParticle.h"
#include "MSAPhysicsTypes.h"

namespace msa {
namespace physics {

template <typename T>
class SectorT {
public:
    typedef shared_ptr< WorldT<T> >           World_ptr;
    typedef shared_ptr< SectorT<T> >          Sector_ptr;
    typedef shared_ptr< ParamsT<T> >          Params_ptr;
    typedef shared_ptr< ParticleT<T> >        Particle_ptr;
    typedef shared_ptr< SpringT<T> >          Spring_ptr;
    typedef shared_ptr< AttractionT<T> >      Attraction_ptr;
    typedef shared_ptr< ConstraintT<T> >      Constraint_ptr;

    static Sector_ptr   create()                        { return Sector_ptr(new SectorT<T>); }

    void                checkSectorCollisions();
    void                addParticle(Particle_ptr p)     { _particles.push_back(p); }
    void                clear()                         { _particles.clear(); }

protected:
    vector< Particle_ptr >	_particles;

    SectorT() {}
    static bool checkCollisionBetween(ParticleT<T>& a, ParticleT<T>& b);
};


//--------------------------------------------------------------
template <typename T>
void SectorT<T>::checkSectorCollisions() {
    int s = _particles.size();
    for(int i=0; i<s-1; i++) {
        auto& part1 = *_particles[i];
        for(int j=i+1; j<s; j++) {
            checkCollisionBetween(part1, *_particles[j]);
        }
    }
}


//--------------------------------------------------------------
template <typename T>
bool SectorT<T>::checkCollisionBetween(ParticleT<T>& a, ParticleT<T>& b) {
    if(a.hasCollision() == false || b.hasCollision() == false) return false;
    if(a.hasPassiveCollision() && b.hasPassiveCollision()) return false;
    if((a.collisionPlane & b.collisionPlane) == 0) return false;

    //			printf("same planes %i %i\n", a.collisionPlane, b.collisionPlane);

    float restLength = b.getRadius() + a.getRadius();
    T delta = b.getPosition() - a.getPosition();
    float deltaLength2 = delta.lengthSquared();
    if(deltaLength2 >restLength * restLength) return false;

    // TODO: fast approximation of square root
    // (1st order Taylor-expansion at a neighborhood of the rest length r (one Newton-Raphson iteration with initial guess r))
    float deltaLength = sqrt(deltaLength2);
    float force = (deltaLength - restLength) / (deltaLength * (a.getInvMass() + b.getInvMass()));

    T deltaForce(delta * force);

    if (a.isFree()) a.moveBy(deltaForce * a.getInvMass(), false);
    if (b.isFree()) b.moveBy(deltaForce * -b.getInvMass(), false);

    a.collidedWithParticle(b, deltaForce);
    b.collidedWithParticle(a, -deltaForce);

    return true;
}

}
}
