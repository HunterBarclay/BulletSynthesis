using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Synthesis;

public class BulletPuppet : MonoBehaviour {
    public PhysicsObject BulletRep;

    public void Start() {
        BulletManager.RegisterPuppet(this);
    }

    public void OnDestroy() {
        BulletManager.UnregisterPuppet(this);
    }
}
