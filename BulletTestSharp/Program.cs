using System;

namespace BulletTestSharp {
    class Program {
        static void Main(string[] args) {
            var ptr = Synthesis.PhysicsManager.CreateBoxShape(1, 5, 3);
            var extents = Synthesis.PhysicsManager.GetBoxShapeExtents(ptr);
            Console.WriteLine($"{extents.x}, {extents.y}, {extents.z}");
        }
    }
}
