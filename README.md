ofxMSAPhysics
=====================================

Introduction
------------
C++ openFrameworks addon for particle/constraint based physics library with springs, attractors and collision. It uses a very similar api to the traer.physics library for processing to make getting into it as easy as possible. All classes are template based with typedefs for physics in 2D or 3D - Potentially even more dimensions! Demo at [www.memo.tv/msaphysics](http://www.memo.tv/msaphysics)

Licence
-------
The code in this repository is available under the [MIT License](https://secure.wikimedia.org/wikipedia/en/wiki/Mit_license).  
Copyright (c) 2008-2012 Memo Akten, [www.memo.tv](http://www.memo.tv)  
The Mega Super Awesome Visuals Company


Installation
------------
Copy to your openFrameworks/addons folder.

Dependencies
------------
- MSACore
- MSAObjCPointer


Compatibility
------------
openFrameworks 0072  
I am generally testing only with [openFrameworks](www.openframeworks.cc), however it should work with [Cinder](www.libcinder.org) too. If it doesn't, please file an issue.

Known issues
------------
none

Version history
------------
TODO: 
- auto calculate bin size (keep track of largest particle, on physics::addParticle and particle::setSize)


### v3.2    23/09/2012
- compatible with OF0072
- renamed (uppercase) MSA namespace to (lowercase) msa. (kept MSA as an alias for backwards compatibility)

### v3.1
- moved to all template classes for 2D/3D (or more). so msa::Physics::WorldT<T>. e.g. World2D, World3D.
- enable() and disable() for particles, to not update and leave out of collision

### v3.0
- move to centralized MSALibs (requires MSACore)
- everything is msa::Physics:: namespace
- container is msa::Physics::World (thats what you instantiate now NOT MSAPhysics)
- flat folder structure
- all headers have 'MSAPhysics' prefix
- constraint types are now enums, and defined MSAPhysicsConstraint.h
- brand new binning system, much faster (currenly has bugs on sector borders)

### v2.5
- ofxMSAParticle no longer extends Vec3f but instead has a protected _pos property
- use ofxMSAParticle::getX(), getY(), getZ() and getPosition() to get position
- ofxMSAParticle::moveTo() and moveBy() takes second parameter whether to preserve velocity or not
- all position and velocity setters only take Vec3f instead of individual x,y,z. this is for future plans of having 2D and 3D physics within the same API
- added minimum and maximum distance to all constraints (attraction, spring, collision etc.)

### v2.1	07/04/2009
- changed license to revised BSD (a lot more more permissive than GPL)

### v2.0.2a 09/03/09
- updates to fix warnings and an error (long) on PC
- fixes to collision / bin checking

### v2.0a	29/02/09
- a lot of new files in this one, so you'll need to remove and re-add it to your project
- no longer requires ofxVectorMath
- attractors fully implemented
- collision (between particles) fully implemented
   - you can enable/disable collision per particle, and it will collide with all other particles which have collisin enabled
   - AND/OR you can globally enable or disable collision
   - AND/OR you can manually create a collision constraint between any 2 (or more) specific particles
- you can set world dimenensions for optimized collision (using Zach L.'s binning code) and particle world edge collision
- particles have individual drag
- particles have individual bounce (for collision)
- particles have individual size (for collision and rendering)
- all 'setter' methods return the instance so you can chain them (e.g. myParticle->setMass(1)->setBonuce(0.5)->enableCollision()->makeFree(); )
- replay functionality temporarily disabled while I fix stuff
- intense testing of memory management so should be stable as a rock, lemme know if you see anything weird (turn verbose on to see whats going on)
- using the super fast inverse square root approximation (attributed to john carmack but originally from Silicon Graphics)
- lots of internal restructing and optimization

### v1.2	12/11/08
- attractor constraints added
- serialization (record and playback) added
- debug draw options
- callbacks for updaters and drawers
- getters and setters for a bunch of internal vars
- general internal restructing and optimization

### v1.0	03/10/08
- initial version


