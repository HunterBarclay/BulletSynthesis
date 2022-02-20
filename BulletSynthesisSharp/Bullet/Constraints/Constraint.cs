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

        public virtual void Destroy() { throw new NotImplementedException(); }
    }

    public class HingeConstraint : Constraint {

        public (float low, float high) Limits {
            get => (PhysicsHandler.GetHingeLowLimit(_constraint_ptr), PhysicsHandler.GetHingeHighLimit(_constraint_ptr));
            set => PhysicsHandler.SetHingeLimit(_constraint_ptr, value.low, value.high);
        }

        public float Softness {
            get => PhysicsHandler.GetHingeLimitSoftness(_constraint_ptr);
            set => PhysicsHandler.SetHingeLimitSoftness(_constraint_ptr, value);
        }

        internal HingeConstraint(VoidPointer ptr) : base(ptr) { }

        ~HingeConstraint() {
            Destroy();
        }

        public override void Destroy() {
            PhysicsHandler.DeleteHingeConstraint(_constraint_ptr);
        }

        public static HingeConstraint Create(PhysicsObject a, PhysicsObject b, Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB) {
            var hinge = new HingeConstraint(PhysicsHandler.CreateHingeConstraint(a._rigidbody_ptr, b._rigidbody_ptr, pivotA, pivotB, axisA, axisB, 0.1f, 0));
            return hinge;
        }
        public static HingeConstraint Create(PhysicsObject a, PhysicsObject b, Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB, float lowLim, float highLim) {
            var hinge = new HingeConstraint(PhysicsHandler.CreateHingeConstraint(a._rigidbody_ptr, b._rigidbody_ptr, pivotA, pivotB, axisA, axisB, lowLim, highLim));
            return hinge;
        }
    }
}
