//#pragma once

//#include "MSAPhysicsParticle.h"
//#include "MSAPhysicsTypes.h"

//namespace msa {
//namespace physics {

///********************* ParticleT drawer class *************************/
//template <typename T>
//class ParticleDrawerT {
//public:
//    virtual void draw(Particle_ptr<T> p) {
//        //				glPushMatrix();
//        //				glTranslatef(p->getX(), p->getY(), p->getZ());
//        //				glutSolidSphere(10, 10, 10);
//        //				glPopMatrix();
//    };
//};




///********************* Base class for drawable class *************************/
//template <typename T>
//class ParticleDrawableT {
//public:
//    ParticleDrawableT() {
//        _drawer = NULL;
//    }

//    virtual ~ParticleDrawableT() {
//        if(_drawer) _drawer->release();
//    }

//    void draw(ParticleT<T>* particle) {
//        if(_drawer) draw(particle);
//    }


//protected:
//    ParticleDrawerT<T> *_drawer;
//};

//}
//}
