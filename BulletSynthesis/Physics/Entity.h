#ifndef SYNTHESIS_PHYSICS_ENTITY
#define SYNTHESIS_PHYSICS_ENTITY

#include "btBulletDynamicsCommon.h"

#include <vector>

class Entity {
private:
	btRigidBody* body;
public:
	Entity();
	~Entity();
};

#endif
