using System;
using System.Runtime.InteropServices;

namespace Synthesis {
    /// <summary>
    /// Manager class for interpreting between a c# interface and a c++ interface
    /// </summary>
    public static class PhysicsManager {

        public static unsafe VoidPointer CreateBoxShape(double x, double y, double z) {
            return (VoidPointer)NativePhysics.Create_Box_Shape(new NativePhysics.Vector_3_Native() { x = x, y = y, z = z });
        }
        public static unsafe (double x, double y, double z) GetBoxShapeExtents(VoidPointer ptr) {
            var extents = NativePhysics.Get_Box_Shape_Extents(ptr.Pointer);
            return (extents.x, extents.y, extents.z);
        }

        /// <summary>
        /// Helper containing all native functions and structs
        /// </summary>
        internal static class NativePhysics {

            [StructLayout(LayoutKind.Sequential)]
            internal struct Vector_3_Native {
                public double x;
                public double y;
                public double z;
            }

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Box_Shape(Vector_3_Native halfSize);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Vector_3_Native Get_Box_Shape_Extents(void* shape);

        }
    }

    public struct VoidPointer {
        public unsafe void* Pointer;

        public unsafe VoidPointer(void* pointer) {
            Pointer = pointer;
        }

        public static unsafe explicit operator VoidPointer(void* ptr) => new VoidPointer(ptr);
        public static unsafe explicit operator void*(VoidPointer ptr) => ptr.Pointer;
    }
}
