using System;
using System.Runtime.InteropServices;

namespace Synthesis {

    // TODO: Move these to internal when alternative Vec and Quat classes are available
    [StructLayout(LayoutKind.Sequential)]
    public struct Vec3 {
        public float x;
        public float y;
        public float z;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Quat {
        public float x;
        public float y;
        public float z;
    }

    /// <summary>
    /// Manager class for interpreting between a c# interface and a c++ interface
    /// </summary>
    public static class PhysicsManager {

        /// <summary>
        /// Helper containing all native functions and structs
        /// </summary>
        internal static class NativePhysics {

            #region Creation

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Box_Shape(Vec3 halfSize);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Compound_Shape(int initChildCapacity);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Convex_Shape(float* verts, int len);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_RigidBody_Dynamic(float mass, void* collisionShape);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_RigidBody_Static(void* collisionShape);

            internal static unsafe VoidPointer CreateBoxShape(Vec3 halfExtent)
                => (VoidPointer)Create_Box_Shape(halfExtent);
            internal static unsafe VoidPointer CreateCompoundShape(int initChildCapacity)
                => (VoidPointer)Create_Compound_Shape(initChildCapacity);
            internal static unsafe VoidPointer CreateConvexShape(Vec3[] verts) {
                var v = new float[verts.Length * 3];
                for (int i = 0; i < verts.Length; i++) {
                    int j = i * 3;
                    v[j] = verts[i].x;
                    v[j + 1] = verts[i].y;
                    v[j + 2] = verts[i].z;
                }
                var ptr = Marshal.AllocHGlobal(sizeof(float) * v.Length);
                Marshal.Copy(v, 0, ptr, v.Length);
                var res = (VoidPointer)Create_Convex_Shape((float*)ptr.ToPointer(), v.Length);
                Marshal.FreeHGlobal(ptr);
                return res;
            }
            internal static unsafe VoidPointer CreateRigidBodyDynamic(float mass, VoidPointer collisionShape)
                => (VoidPointer)Create_RigidBody_Dynamic(mass, collisionShape.Pointer);
            internal static unsafe VoidPointer CreateRigidBodyStatic(VoidPointer collisionShape)
                => (VoidPointer)Create_RigidBody_Static(collisionShape.Pointer);

            #endregion

            #region Deletion

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Delete_Collision_Shape(void* collisionShape);

            internal static unsafe void DeleteCollisionShape(VoidPointer collisionShape) {
                Delete_Collision_Shape((void*)collisionShape);
            }

            #endregion

            #region Shape Modification

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Add_Shape_To_Compound_Shape(void* compound, void* shape, Vec3 pos, Quat rot);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Vec3 Get_Box_Shape_Extents(void* shape);

            internal static unsafe void AddShapeToCompoundShape(VoidPointer compound, VoidPointer shape, Vec3 pos, Quat rot) {
                Add_Shape_To_Compound_Shape(compound.Pointer, shape.Pointer, pos, rot);
            }
            internal static unsafe Vec3 GetBoxShapeExtents(VoidPointer ptr)
                => Get_Box_Shape_Extents(ptr.Pointer);

            #endregion

            #region Rigidbody Manipulation

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Apply_Force(void* rigidbody, Vec3 force, Vec3 relativePosition);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Vec3 Get_RigidBody_Linear_Velocity(void* rigidbody);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Set_RigidBody_Linear_Velocity(void* rigidbody, Vec3 vel);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Vec3 Get_RigidBody_Angular_Velocity(void* rigidbody);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Set_RigidBody_Angular_Velocity(void* rigidbody, Vec3 vel);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Quat Get_RigidBody_Rotation(void* rigidbody);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Set_RigidBody_Rotation(void* rigidbody, Quat rotation);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Vec3 Get_RigidBody_Position(void* rigidbody);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Set_RigidBody_Position(void* rigidbody, Vec3 pos);

            #endregion

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
