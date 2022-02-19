using System;
using Synthesis;

namespace BulletTestSharp {
    class Program {
        static void Main(string[] args) {
            var box = BoxShape.Create(new Synthesis.Vec3() { x = 1, y = 5, z = 3 });
            var extents = box.HalfExtents;
            Console.WriteLine($"{extents.x}, {extents.y}, {extents.z}");
        }
    }
}
