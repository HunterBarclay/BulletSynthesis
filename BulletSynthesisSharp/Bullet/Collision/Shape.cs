using System;
using System.Collections.Generic;
using System.Text;

namespace Synthesis {
    public class Shape {
        internal VoidPointer _shape_ptr;

        protected Shape(VoidPointer ptr) {
            _shape_ptr = ptr;
        }

        ~Shape() {
            Destroy();
        }

        public void Destroy() {
            PhysicsHandler.DeleteCollisionShape(_shape_ptr);
        }
    }

    public class BoxShape : Shape {

        private Vec3 _halfExtents;
        public Vec3 HalfExtents { get => _halfExtents; }

        internal BoxShape(VoidPointer ptr) : base(ptr) {
            _halfExtents = PhysicsHandler.GetBoxShapeExtents(ptr);
        }

        public static BoxShape Create(Vec3 halfExtents)
            => new BoxShape(PhysicsHandler.CreateBoxShape(halfExtents));
    }

    public class CompoundShape : Shape {

        public int ChildrenCount => PhysicsHandler.GetCompoundShapeChildCount(_shape_ptr);
        private List<Shape> _children = new List<Shape>();
        public IReadOnlyCollection<Shape> Children => _children.AsReadOnly();

        internal CompoundShape(VoidPointer ptr) : base(ptr) { }

        public static CompoundShape Create(int initChildCapacity) {
            var shape = new CompoundShape(PhysicsHandler.CreateCompoundShape(initChildCapacity));
            return shape;
        }

        public void AddShape(Shape shape, Vec3 offset, Quat orientation) {
            PhysicsHandler.AddShapeToCompoundShape(_shape_ptr, shape._shape_ptr, offset, orientation);
            _children.Add(shape);
        }
    }

    public class SphereShape : Shape {

        public float Radius => PhysicsHandler.GetSphereShapeRadius(_shape_ptr);

        internal SphereShape(VoidPointer ptr) : base(ptr) { }

        public static SphereShape Create(float radius) {
            var shape = new SphereShape(PhysicsHandler.CreateSphereShape(radius));
            return shape;
        }
    }
}
