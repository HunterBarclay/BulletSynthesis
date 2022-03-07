#include "PhysicsManager.h"

// #include "../Logger.h"

namespace synthesis {

	PhysicsManager::PhysicsManager() {
		///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
		collisionConfiguration = new btDefaultCollisionConfiguration();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new btCollisionDispatcher(collisionConfiguration);

		///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
		overlappingPairCache = new btDbvtBroadphase();

		///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		solver = new btSequentialImpulseConstraintSolver;

		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

		dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));
		SetNumIteration(4);

		/*nodes = std::map<int, PhysicsNode*>();*/

		// _rigidBodies = std::vector<btRigidBody*>();
		// _managedShapes = std::vector<btCollisionShape*>();
	}

	PhysicsManager::~PhysicsManager() {

		/*int i;
		for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				delete body->getMotionState();
			}
			dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}*/

		/*for (i = _managedShapes.size() - 1; i >= 0; i--) {
			btCollisionShape* shape = _managedShapes.at(i);
			if (shape != nullptr)
				delete shape;
			_managedShapes.pop_back();
		}*/

		/*for (std::map<int, PhysicsNode*>::iterator ptr = nodes.begin(); ptr != nodes.end(); ptr++) {
			delete ptr->second;
		}*/

		//delete collision shapes
		/*for (int j = 0; j < collisionShapes.size(); j++) {
			btCollisionShape* shape = collisionShapes[j];
			collisionShapes[j] = 0;
			delete shape;
		}*/

		//delete dynamics world
		delete dynamicsWorld;

		//delete solver
		delete solver;

		//delete broadphase
		delete overlappingPairCache;

		//delete dispatcher
		delete dispatcher;

		delete collisionConfiguration;
	}

	void PhysicsManager::deleteAssociatedConstraints(btRigidBody* body) {
		for (int i = 0; i < body->getNumConstraintRefs(); i++) {
			btTypedConstraint* constraint = body->getConstraintRef(i);
			if (constraint)
				DestroyConstraint(constraint);
		}
	}

	void PhysicsManager::DestroyRigidbody(btRigidBody* rb) {
		if (!rb)
			return;

		deleteAssociatedConstraints(rb);
		if (rb->getMotionState()) {
			delete rb->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(rb);
		delete rb;
	}

	void PhysicsManager::DestroyConstraint(btTypedConstraint* constraint) {
		dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}

	void PhysicsManager::DestroySimulation() {
		int i;
		for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body) {
				DestroyRigidbody(body);
			}
			else {
				dynamicsWorld->removeCollisionObject(obj);
				delete obj;
			}
		}
	}

	void PhysicsManager::Step(float deltaT) {

		dynamicsWorld->stepSimulation(1.f / 60.f, 10);

		// Synthesis_Log("Test");

		//for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--) {
		//	btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
		//	btRigidBody* body = btRigidBody::upcast(obj);
		//	btTransform trans;
		//	if (body && body->getMotionState()) {
		//		body->getMotionState()->getWorldTransform(trans);
		//	}
		//	else {
		//		trans = obj->getWorldTransform();
		//	}
		//	printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
		//	// msg = msg + "," + std::to_string(trans.getOrigin().getX()) + "," + std::to_string(trans.getOrigin().getY()) + "," + std::to_string(trans.getOrigin().getZ());
		//}

		/*for (auto iter = nodes.begin(); iter != nodes.end(); iter++) {
			PhysicsNode* node = iter->second;

		}*/
	}

	btCollisionShape* PhysicsManager::createBoxCollisionShape(btVector3 halfSize) {
		return new btBoxShape(halfSize);
	}

	btRigidBody* PhysicsManager::createDynamicRigidBody(btCollisionShape* shape, btScalar mass) {
		btTransform transform;
		transform.setIdentity();
		// groundTransform.setOrigin(btVector3(0, -56, 0));
		
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
		
		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);
		
		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		dynamicsWorld->addRigidBody(body);

		return body;
		// body->setRestitution(btScalar(0.9));
		
		//add the body to the dynamics world
		// dynamicsWorld->addRigidBody(body);
	}

	btRigidBody* PhysicsManager::createStaticRigidBody(btCollisionShape* shape) {
		btTransform transform;
		transform.setIdentity();
		// transform.setOrigin(btVector3(0, -56, 0));
		btVector3 localInertia(0, 0, 0);
		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		dynamicsWorld->addRigidBody(body);
		return body;
	}

	btHingeConstraint* PhysicsManager::createHingeConstraint(
		btRigidBody* bodyA, btRigidBody* bodyB, btVector3 pivotA, btVector3 pivotB,
		btVector3 axisA, btVector3 axisB, float lowLim, float highLim, bool internalCollision
	) {
		btHingeConstraint* hinge = new btHingeConstraint(*bodyA, *bodyB, pivotA, pivotB, axisA, axisB);
		hinge->setLimit(lowLim, highLim);
		dynamicsWorld->addConstraint(hinge, !internalCollision); // TODO: Make collision togglable
		return hinge;
	}

	void PhysicsManager::rayCastClosest(btVector3 from, btVector3 to, btCollisionWorld::ClosestRayResultCallback& callback) {
		dynamicsWorld->rayTest(from, to, callback);
	}

	void PhysicsManager::Run(int frames, float secondsPerFrame) {
		// createSampleNode();

		for (int i = 0; i < frames; i++) {
			Step(secondsPerFrame);
		}
	}

}
