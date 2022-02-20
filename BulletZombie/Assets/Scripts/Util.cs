using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Synthesis;

public static class Util {
    public static Vec3 ToBullet(float x, float y, float z) => new Vec3 { x = x, y = y, z = z };
    public static Vec3 ToBullet(Vector3 v) => new Vec3 { x = v.x, y = v.y, z = v.z };
    public static Vector3 ToUnity(Vec3 v) => new Vector3(v.x, v.y, v.z);
    public static Quat ToBullet(float x, float y, float z, float w) => new Quat { x = x, y = y, z = z, w = w };
    public static Quat ToBullet(Quaternion q) => new Quat { x = q.x, y = q.y, z = q.z, w = q.w };
    public static Quaternion ToUnity(Quat q) => new Quaternion(q.x, q.y, q.z, q.w);
}
