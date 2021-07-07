#ifndef SYNTHESIS_PHYSICS_MANAGER
#define SYNTHESIS_PHYSICS_MANAGER

#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>

namespace synthesis {

	class PhysicsManager {
	private:
		PhysicsManager();
		~PhysicsManager();

		std::vector<btCollisionShape*> _managedShapes;

		btDefaultCollisionConfiguration* collisionConfiguration;
		btCollisionDispatcher* dispatcher;
		btBroadphaseInterface* overlappingPairCache;
		btSequentialImpulseConstraintSolver* solver;
		btDiscreteDynamicsWorld* dynamicsWorld;
	public:
		// Eh?
		static PhysicsManager& getInstance() {
			static PhysicsManager instance;
			return instance;
		}

		inline btDiscreteDynamicsWorld* getWorld() {
			return dynamicsWorld;
		}

		// Are these functions necessary?
		btCollisionShape* createBoxCollisionShape(btVector3 halfSize);
		btRigidBody* createStaticRigidBody(btCollisionShape* shape);
		btRigidBody* createDynamicRigidBody(btCollisionShape* shape, btScalar mass);

		inline int getRigidBodyActivationState(btRigidBody* body) { return body->getActivationState(); }
		inline void setRigidBodyActivationState(btRigidBody* body, int state) { body->setActivationState(state); }

		/*int getNextId() {
			static int id = 0;
			id++;
			return id;
		}*/

		void Step(float deltaT);
		void Run(int frames, double secondsPerFrame);
	};

}

#endif
