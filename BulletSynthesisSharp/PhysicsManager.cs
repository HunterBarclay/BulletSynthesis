using System;
using System.Runtime.InteropServices;

namespace Synthesis {
    /// <summary>
    /// Manager class for interpreting between a c# interface and a c++ interface
    /// </summary>
    public static class PhysicsManager {
        /// <summary>
        /// Helper containing all native functions and structs
        /// </summary>
        internal static class NativePhysics {

            [StructLayout(LayoutKind.Sequential)]
            internal struct Vector_3_Native {
                double x;
                double y;
                double z;
            }

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Box_Shape(Vector_3_Native halfSize);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Vector_3_Native Get_Box_Shape_Extents(void* shape);

        }
    }
}
