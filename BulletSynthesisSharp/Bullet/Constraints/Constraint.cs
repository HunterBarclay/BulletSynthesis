using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Synthesis {
    public class Constraint {
        internal VoidPointer _constraint_ptr;

        protected Constraint(VoidPointer ptr) {
            _constraint_ptr = ptr;
        }
    }

    public class HingeConstraint : Constraint {

        public (float low, float high) Limit {
            get => (PhysicsManager.GetHingeLowLimit(_constraint_ptr), PhysicsManager.GetHingeHighLimit(_constraint_ptr));
            set => PhysicsManager.SetHingeLimit(_constraint_ptr, value.low, value.high);
        }

        internal HingeConstraint(VoidPointer ptr) : base(ptr) { }

        ~HingeConstraint() {
            PhysicsManager.DeleteHingeConstraint(_constraint_ptr);
        }

        public static HingeConstraint Create(PhysicsObject a, PhysicsObject b, Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB) {
            var hinge = new HingeConstraint(PhysicsManager.CreateHingeConstraint(a._rigidbody_ptr, b._rigidbody_ptr, pivotA, pivotB, axisA, axisB, 0.1f, 0));
            return hinge;
        }
        public static HingeConstraint Create(PhysicsObject a, PhysicsObject b, Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB, float lowLim, float highLim) {
            var hinge = new HingeConstraint(PhysicsManager.CreateHingeConstraint(a._rigidbody_ptr, b._rigidbody_ptr, pivotA, pivotB, axisA, axisB, lowLim, highLim));
            return hinge;
        }
    }
}
