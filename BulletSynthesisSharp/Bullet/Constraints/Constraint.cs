using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Synthesis {
    // TODO: Potential problem with constraints
    public class Constraint {
        internal VoidPointer _constraint_ptr;

        protected Constraint(VoidPointer ptr) {
            _constraint_ptr = ptr;
        }

        // Constraints will automatically be deleted when rigidbodies associated to them are deleted
        public virtual void Delete() { throw new NotImplementedException(); }
    }

    public class HingeConstraint : Constraint {

        public float Angle {
            get => PhysicsHandler.GetHingeAngle(_constraint_ptr);
        }

        public (float low, float high) Limits {
            get => (PhysicsHandler.GetHingeLowLimit(_constraint_ptr), PhysicsHandler.GetHingeHighLimit(_constraint_ptr));
            set => PhysicsHandler.SetHingeLimit(_constraint_ptr, value.low, value.high);
        }
        public float LimitSoftness {
            get => PhysicsHandler.GetHingeLimitSoftness(_constraint_ptr);
            set => PhysicsHandler.SetHingeLimitSoftness(_constraint_ptr, value);
        }

        public bool MotorEnable {
            get => PhysicsHandler.GetHingeMotorEnable(_constraint_ptr);
            set => PhysicsHandler.SetHingeMotorEnable(_constraint_ptr, value);
        }
        public float MotorMaxImpulse {
            get => PhysicsHandler.GetHingeMotorMaxImpulse(_constraint_ptr);
            set => PhysicsHandler.SetHingeMotorMaxImpulse(_constraint_ptr, value);
        }
        public float MotorTargetVelocity {
            get => PhysicsHandler.GetHingeMotorTargetVelocity(_constraint_ptr);
            set => PhysicsHandler.SetHingeMotorTargetVelocity(_constraint_ptr, value);
        }

        internal HingeConstraint(VoidPointer ptr) : base(ptr) { }

        public override void Delete() {
            PhysicsHandler.DeleteHingeConstraint(_constraint_ptr);
        }

        public static HingeConstraint Create(PhysicsObject a, PhysicsObject b, Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB, bool internalCollision = false) {
            var hinge = new HingeConstraint(PhysicsHandler.CreateHingeConstraint(a._rigidbody_ptr, b._rigidbody_ptr, pivotA, pivotB, axisA, axisB, 0.1f, 0, internalCollision));
            return hinge;
        }
        public static HingeConstraint Create(PhysicsObject a, PhysicsObject b, Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB, float lowLim, float highLim, bool internalCollision = false) {
            var hinge = new HingeConstraint(PhysicsHandler.CreateHingeConstraint(a._rigidbody_ptr, b._rigidbody_ptr, pivotA, pivotB, axisA, axisB, lowLim, highLim, internalCollision));
            return hinge;
        }
    }
}
