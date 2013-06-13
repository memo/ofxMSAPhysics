#pragma once

#include "MSACore.h"
#include "MSAObjCPointer.h"
#include "MSAPhysicsParams.h"

namespace msa {
	
	namespace physics {
		
		template <typename T>
		class WorldT;
		
		template <typename T>
		class ParticleT : public ObjCPointer {
			
		public:
			friend class WorldT<T>;
			
			ParticleT();
			ParticleT(T pos, float m = 1.0f, float d = 1.0f);
			ParticleT(ParticleT &p);
			
			virtual void	init(T pos, float m = 1.0f, float d = 1.0f);
			
			ParticleT*		setMass(float t = 1);
			float			getMass();
			float			getInvMass();
			
			ParticleT*		setDrag(float t = 1);
			float			getDrag();
			
			ParticleT*		setBounce(float t = 1);
			float			getBounce();
			
			ParticleT*		setRadius(float t = 15);
			float			getRadius();
			
			// collision methods
			ParticleT*		enableCollision();
			ParticleT*		disableCollision();
			bool			hasCollision();
            
            // passive particles do not collied with each other, only with non-passive (collision must be enabled)
            ParticleT*		enablePassiveCollision();
            ParticleT*		disablePassiveCollision();
            bool            hasPassiveCollision();
			
			bool			isFixed();
			bool			isFree();
			ParticleT*		makeFixed();
			ParticleT*		makeFree();
			
			// quick way of enabling (collision and update) and disabling
			ParticleT*		enable();
			ParticleT*		disable();
			
			// move the particle
			// if preserveVelocity == true, the particle will move to new position and keep it's old velocity
			// if preserveVelocity == false, the particle will move to new position but gain the velocity of the displacement
			ParticleT* moveTo(T targetPos, bool preserveVelocity = true);
			ParticleT* moveBy(T offset, bool preserveVelocity = true);
			
//			float			getX();
//			float			getY();
//			float			getZ();
			const T&		getPosition();
			
			ParticleT*		setVelocity(T vel);
			ParticleT*		addVelocity(T vel);
			T				getVelocity();
			
			// override these functions if you create your own particle type with custom behaviour and/or drawing
			virtual void	update() {}		// called every frame in world::update();
			virtual void	draw() {}		// called every frame in world::draw();
			virtual void	collidedWithParticle(ParticleT *other, T collisionForce) {}	// called when this particle collides with another particle (called for both particles)
			virtual void	collidedWithEdgeOfWorld(T collisionForce) {}
			
			void			kill();
			bool			isDead();
			
			// custom void* which you can use to store any kind of custom data
			void			*data;
			
			ParamsT<T>		*getParams();

			// only particles sharing bits in the collision plane collide with each other
			unsigned int	collisionPlane;

		protected:
			ParamsT<T>		*_params;
			WorldT<T>		*_world;
			
			
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
			
			void			doVerlet();
			void			checkWorldEdges();
			
			virtual void debugDraw();
		};
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::setMass(float t) {
			if(t==0) t=0.00001f;
			_mass = t;
			_invMass = t > 0 ? 1.0f/t : 0;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline float ParticleT<T>::getMass() {
			return _mass;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline float ParticleT<T>::getInvMass() {
			return _invMass;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::setDrag(float t) {
			_drag = t;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline float ParticleT<T>::getDrag() {
			return _drag;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::setBounce(float t) {
			_bounce = t;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline float ParticleT<T>::getBounce() {
			return _bounce;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::setRadius(float t) {
			_radius = t;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline float ParticleT<T>::getRadius() {
			return _radius;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline bool ParticleT<T>::isFixed() {
			return (_isFixed == true);
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline bool ParticleT<T>::isFree() {
			return (_isFixed == false);
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::makeFixed() {
			_isFixed = true;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::makeFree() {
			_oldPos = _pos;
			_isFixed = false;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::moveTo(T targetPos, bool preserveVelocity) {
			T diff = targetPos - _pos;
			moveBy(diff, preserveVelocity);
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::moveBy(T offset, bool preserveVelocity) {
			_pos += offset;
			if(preserveVelocity) _oldPos += offset;
			return this;
		}
		
        //--------------------------------------------------------------
//		template <typename T>
//		inline float ParticleT<T>::getX() {
//			return _pos.x;
//		}
//		
        //--------------------------------------------------------------
//		template <typename T>
//		inline float ParticleT<T>::getY() {
//			return _pos.y;
//		}
//		
        //--------------------------------------------------------------
//		template <typename T>
//		inline float ParticleT<T>::getZ() {
//			return _pos.z;
//		}
//		
        //--------------------------------------------------------------
		template <typename T>
		inline const T& ParticleT<T>::getPosition() {
			return _pos;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::setVelocity(T vel) {
			_oldPos = _pos - vel;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline ParticleT<T>* ParticleT<T>::addVelocity(T vel) {
			_oldPos -= vel;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline T ParticleT<T>::getVelocity() {
			return _pos - _oldPos;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		inline void ParticleT<T>::kill() {
			_isDead = true;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		inline bool ParticleT<T>::isDead() {
			return _isDead;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>::ParticleT() {
			init(T());
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>::ParticleT(T pos, float m, float d) {
			init(pos, m, d);
		}
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>::ParticleT(ParticleT<T> &p) {
			init(p.getPosition(), p._mass, p._drag);
			_isFixed = p._isFixed;
			setBounce(p._bounce);
			setRadius(p._radius);
		}
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		void ParticleT<T>::init(T pos, float m, float d) {
			_params = NULL;
			_world = NULL;
			
			_pos = _oldPos = pos;
			setMass(m);
			setDrag(d);
			setBounce();
			setRadius();
			enableCollision();
            disablePassiveCollision();
			makeFree();
			_isDead = false;
			_age = 0;
			verbose = true;
			data = NULL;
			
			collisionPlane = -1;
			
			setClassName("ParticleT");
		}
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* ParticleT<T>::enableCollision(){
			_collisionEnabled = true;
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* ParticleT<T>::disableCollision() {
			_collisionEnabled = false;
			return this;
		}

        //--------------------------------------------------------------
		template <typename T>
		bool ParticleT<T>::hasCollision() {
			return _collisionEnabled;
		}
        
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* ParticleT<T>::enablePassiveCollision() {
			_passiveCollision = true;
			return this;
		}

        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* ParticleT<T>::disablePassiveCollision() {
			_passiveCollision = false;
			return this;
		}
        
        //--------------------------------------------------------------
		template <typename T>
		bool ParticleT<T>::hasPassiveCollision() {
			return _passiveCollision;
		}


		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* ParticleT<T>::enable(){
			enableCollision();
			makeFree();
			return this;
		}
		
        //--------------------------------------------------------------
		template <typename T>
		ParticleT<T>* ParticleT<T>::disable() {
			disableCollision();
			makeFixed();
			return this;
		}
		
        
        //--------------------------------------------------------------
		template <typename T>
		void ParticleT<T>::doVerlet() {
			if (!_isFixed) {
				if(_params->doGravity) {
					T gravityForce = _params->gravity;
					addVelocity(gravityForce);
				}
				
				T curPos = _pos;
				T vel = _pos - _oldPos;
				_pos += vel * _params->drag * _drag;// + _params->timeStep2;
				//_pos += (_pos - _oldPos);// + _params->timeStep2;	// TODO
				_oldPos = curPos;
			}
		}
		
		
		
        //--------------------------------------------------------------
		template <typename T>
		void ParticleT<T>::checkWorldEdges() {
			//	printf("%.3f, %.3f, %.3f\n", _params->worldMin.x, _params->worldMax.y, _params->worldMax.z);

			// not keen on this solution, but best so far
			//			T r;
			
			bool collided = false;
			T oldVel = getVelocity();
			for(int i=0; i<T::DIM; i++) {
				//				r[i] = _radius;
				
				float vel = _pos[i] - _oldPos[i];
				if(_pos[i] < _params->worldMin[i] + _radius) {
					_pos[i] = _params->worldMin[i] + _radius;
					_oldPos[i] = _pos[i] + vel * _bounce;
					collided = true;
				} else if(_pos[i] > _params->worldMax[i] - _radius) {
					_pos[i] = _params->worldMax[i] - _radius;
					_oldPos[i] = _pos[i] + vel * _bounce;
					collided = true;
				}
			}
			
			if(collided) collidedWithEdgeOfWorld(getVelocity() - oldVel);

			
			//			if(_pos.x < _params->worldMin.x + _radius) {
			//				float vel = _pos.x - _oldPos.x;
			//				_pos.x = _params->worldMin.x + _radius;
			//				_oldPos.x = _pos.x + vel * _bounce;
			//			} else if(_pos.x > _params->worldMax.x - _radius) {
			//				float vel = _pos.x - _oldPos.x;
			//				_pos.x = _params->worldMax.x - _radius;
			//				_oldPos.x = _pos.x + vel * _bounce;
			//			}
			//			
			//			if( _pos.y < _params->worldMin.y + _radius) {
			//				float vel = _pos.y - _oldPos.y;
			//				_pos.y = _params->worldMin.y + _radius;
			//				_oldPos.y = _pos.y + vel * _bounce;
			//			} else if(_pos.y > _params->worldMax.y - _radius) {
			//				float vel = _pos.y - _oldPos.y;
			//				_pos.y = _params->worldMax.y - _radius;
			//				_oldPos.y = _pos.y + vel * _bounce;
			//			}
			//			
			//			if(_pos.z < _params->worldMin.z + _radius) {
			//				float vel = _pos.z - _oldPos.z;
			//				_pos.z = _params->worldMin.z + _radius;
			//				_oldPos.z = _pos.z + vel * _bounce;
			//			} else if(_pos.z > _params->worldMax.z - _radius) {
			//				float vel = _pos.z - _oldPos.z;
			//				_pos.z = _params->worldMax.z - _radius;
			//				_oldPos.z = _pos.z + vel * _bounce;
			//			}
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		ParamsT<T> *ParticleT<T>::getParams() {
			return _params;
		}
		
		
        //--------------------------------------------------------------
		template <typename T>
		void ParticleT<T>::debugDraw() {
			//			glPushMatrix();
			//			glTranslatef(_pos.x, _pos.y, _pos.z);
			//#ifndef TARGET_OS_IPHONE
			//			glutSolidSphere(_radius, 12, 12);
			//#else
			//			ofCircle(0, 0, _radius);
			//#endif
			//			glPopMatrix();
		}
		
		
	}
}
