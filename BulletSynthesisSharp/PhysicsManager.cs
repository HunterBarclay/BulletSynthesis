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
        public float w;
    }

    /// <summary>
    /// Manager class for interpreting between a c# interface and a c++ interface
    /// </summary>
    public static class PhysicsManager {

        #region Simulation Control

        public static void Step(float deltaT) {
            NativePhysics.Physics_Step(deltaT);
        }

        public static void DestroySimulation() {
            NativePhysics.Physics_Destroy_Simulation();
        }

        #endregion

        #region Creation

        public static unsafe VoidPointer CreateBoxShape(Vec3 halfExtent)
                => (VoidPointer)NativePhysics.Create_Box_Shape(halfExtent);
        public static unsafe VoidPointer CreateCompoundShape(int initChildCapacity)
            => (VoidPointer)NativePhysics.Create_Compound_Shape(initChildCapacity);
        public static unsafe VoidPointer CreateConvexShape(Vec3[] verts) {
            var v = new float[verts.Length * 3];
            for (int i = 0; i < verts.Length; i++) {
                int j = i * 3;
                v[j] = verts[i].x;
                v[j + 1] = verts[i].y;
                v[j + 2] = verts[i].z;
            }
            var ptr = Marshal.AllocHGlobal(sizeof(float) * v.Length);
            Marshal.Copy(v, 0, ptr, v.Length);
            var res = (VoidPointer)NativePhysics.Create_Convex_Shape((float*)ptr.ToPointer(), v.Length);
            Marshal.FreeHGlobal(ptr);
            return res;
        }
        public static unsafe VoidPointer CreateSphereShape(float radius)
            => (VoidPointer)NativePhysics.Create_Sphere_Shape(radius);
        public static unsafe VoidPointer CreateRigidBodyDynamic(float mass, VoidPointer collisionShape)
            => (VoidPointer)NativePhysics.Create_RigidBody_Dynamic(mass, collisionShape.Pointer);
        public static unsafe VoidPointer CreateRigidBodyStatic(VoidPointer collisionShape)
            => (VoidPointer)NativePhysics.Create_RigidBody_Static(collisionShape.Pointer);

        #endregion

        #region Deletion

        public static unsafe void DeleteCollisionShape(VoidPointer collisionShape) {
            NativePhysics.Delete_Collision_Shape(collisionShape.Pointer);
        }

        public static unsafe void DeleteRigidBodyShape(VoidPointer rigidbody) {
            NativePhysics.Delete_RigidBody(rigidbody.Pointer);
        }

        public static unsafe void DeleteHingeConstraint(VoidPointer hinge) {
            NativePhysics.Delete_Hinge_Constraint(hinge.Pointer);
        }

        #endregion

        #region Shape Modification

        public static unsafe void AddShapeToCompoundShape(VoidPointer compound, VoidPointer shape, Vec3 pos, Quat rot) {
            NativePhysics.Add_Shape_To_Compound_Shape(compound.Pointer, shape.Pointer, pos, rot);
        }
        public static unsafe int GetCompoundShapeChildCount(VoidPointer compound) {
            return NativePhysics.Get_Compound_Shape_Child_Count(compound.Pointer);
        }
        public static unsafe Vec3 GetBoxShapeExtents(VoidPointer ptr)
            => NativePhysics.Get_Box_Shape_Extents(ptr.Pointer);
        public static unsafe float GetSphereShapeRadius(VoidPointer ptr)
            => NativePhysics.Get_Sphere_Shape_Radius(ptr.Pointer);

        #endregion

        #region Rigidbody Manipulation

        public static unsafe void ApplyForce(VoidPointer rigidbody, Vec3 force, Vec3 relativePosition) {
            NativePhysics.Apply_Force(rigidbody.Pointer, force, relativePosition);
        }

        // Getters
        public static unsafe Vec3 GetRigidBodyLinearVelocity(VoidPointer rigidbody)
            => NativePhysics.Get_RigidBody_Linear_Velocity(rigidbody.Pointer);
        public static unsafe Vec3 GetRigidBodyAngularVelocity(VoidPointer rigidbody)
            => NativePhysics.Get_RigidBody_Angular_Velocity(rigidbody.Pointer);
        public static unsafe Vec3 GetRigidBodyPosition(VoidPointer rigidbody)
            => NativePhysics.Get_RigidBody_Position(rigidbody.Pointer);
        public static unsafe Quat GetRigidBodyRotation(VoidPointer rigidbody)
            => NativePhysics.Get_RigidBody_Rotation(rigidbody.Pointer);

        // Setters
        public static unsafe void SetRigidBodyLinearVelocity(VoidPointer rigidbody, Vec3 velocity) {
            NativePhysics.Set_RigidBody_Linear_Velocity(rigidbody.Pointer, velocity);
        }
        public static unsafe void SetRigidBodyAngularVelocity(VoidPointer rigidbody, Vec3 ang_velocity) {
            NativePhysics.Set_RigidBody_Angular_Velocity(rigidbody.Pointer, ang_velocity);
        }
        public static unsafe void SetRigidBodyPosition(VoidPointer rigidbody, Vec3 position) {
            NativePhysics.Set_RigidBody_Position(rigidbody.Pointer, position);
        }
        public static unsafe void SetRigidBodyRotation(VoidPointer rigidbody, Quat rotation) {
            NativePhysics.Set_RigidBody_Rotation(rigidbody.Pointer, rotation);
        }

        #endregion

        #region Constraints

        public static unsafe VoidPointer CreateHingeConstraint(VoidPointer bodyA, VoidPointer bodyB,
            Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB, float lowLim, float highLim)
            => (VoidPointer)NativePhysics.Create_Hinge_Constraint(bodyA.Pointer, bodyB.Pointer, pivotA, pivotB, axisA, axisB, lowLim, highLim);

        public static unsafe float GetHingeLowLimit(VoidPointer hinge)
            => NativePhysics.Get_Hinge_Low_Limit(hinge.Pointer);
        public static unsafe float GetHingeHighLimit(VoidPointer hinge)
            => NativePhysics.Get_Hinge_High_Limit(hinge.Pointer);
        public static unsafe void SetHingeLimit(VoidPointer hinge, float low, float high) {
            NativePhysics.Set_Hinge_Limit(hinge.Pointer, low, high);
        }

        #endregion

        /// <summary>
        /// Helper containing all native functions and structs
        /// </summary>
        internal static class NativePhysics {

            #region Simulation Control

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Physics_Step(float deltaT);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Physics_Destroy_Simulation();

            #endregion

            #region Creation

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Box_Shape(Vec3 halfSize);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Compound_Shape(int initChildCapacity);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Convex_Shape(float* verts, int len);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Sphere_Shape(float radius);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_RigidBody_Dynamic(float mass, void* collisionShape);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_RigidBody_Static(void* collisionShape);

            #endregion

            #region Deletion

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Delete_Collision_Shape(void* collisionShape);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Delete_RigidBody(void* rigidbody);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Delete_Hinge_Constraint(void* hinge);

            #endregion

            #region Shape Modification

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Add_Shape_To_Compound_Shape(void* compound, void* shape, Vec3 pos, Quat rot);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe int Get_Compound_Shape_Child_Count(void* compound);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe Vec3 Get_Box_Shape_Extents(void* shape);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe float Get_Sphere_Shape_Radius(void* shape);

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

            #region Constraints

            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void* Create_Hinge_Constraint(void* bodyA, void* bodyB, Vec3 pivotA,
                Vec3 pivotB, Vec3 axisA, Vec3 axisB, float lowLim, float highLim);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe float Get_Hinge_Low_Limit(void* hinge);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe float Get_Hinge_High_Limit(void* hinge);
            [DllImport("BulletSynthesis.dll", CallingConvention = CallingConvention.Cdecl)]
            internal extern static unsafe void Set_Hinge_Limit(void* hinge, float low, float high);

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
