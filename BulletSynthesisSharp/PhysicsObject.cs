using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Synthesis {
    public class PhysicsObject {

        #region Fields

        public Vec3 Position {
            get => PhysicsManager.GetRigidBodyPosition(_rigidbody_ptr);
            set {
                PhysicsManager.SetRigidBodyPosition(_rigidbody_ptr, value);
            }
        }
        public Quat Rotation {
            get => PhysicsManager.GetRigidBodyRotation(_rigidbody_ptr);
            set {
                PhysicsManager.SetRigidBodyRotation(_rigidbody_ptr, value);
            }
        }
        public Vec3 LinearVelocity {
            get => PhysicsManager.GetRigidBodyLinearVelocity(_rigidbody_ptr);
            set {
                PhysicsManager.SetRigidBodyLinearVelocity(_rigidbody_ptr, value);
            }
        }
        public Vec3 AngularVelocity {
            get => PhysicsManager.GetRigidBodyLinearVelocity(_rigidbody_ptr);
            set {
                PhysicsManager.SetRigidBodyLinearVelocity(_rigidbody_ptr, value);
            }
        }

        #endregion

        internal VoidPointer _rigidbody_ptr;

        internal PhysicsObject(VoidPointer rb_ptr) {
            _rigidbody_ptr = rb_ptr;
        }

        ~PhysicsObject() {
            PhysicsManager.DeleteRigidBodyShape(_rigidbody_ptr);
        }

        public static PhysicsObject Create(Shape shape, float mass = 0.0f) {
            PhysicsObject physObj;
            if (mass == 0.0f) {
                physObj = new PhysicsObject(PhysicsManager.CreateRigidBodyStatic(shape._shape_ptr));
            } else {
                physObj = new PhysicsObject(PhysicsManager.CreateRigidBodyDynamic(mass, shape._shape_ptr));
            }

            return physObj;
        }

        public void ApplyForce(Vec3 force, Vec3 relativePosition) {
            PhysicsManager.ApplyForce(_rigidbody_ptr, force, relativePosition);
        }
    }
}
