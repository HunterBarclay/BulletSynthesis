using System;
using System.Collections.Generic;
using System.Text;

namespace Synthesis.Bullet.Collision {
    public class Shape {
        protected VoidPointer _ptr;

        protected Shape(VoidPointer ptr) {

        }
    }

    public class BoxShape : Shape {

        private Vec3 _halfExtents;
        public Vec3 HalfExtents { get => _halfExtents; }

        internal BoxShape(VoidPointer ptr) : base(ptr) {
            _halfExtents = PhysicsManager.NativePhysics.GetBoxShapeExtents(_ptr);
        }

        public static BoxShape Create(Vec3 halfExtents)
            => new BoxShape(PhysicsManager.NativePhysics.CreateBoxShape(halfExtents));
    }
}
