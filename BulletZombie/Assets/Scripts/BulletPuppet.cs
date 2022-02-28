using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Synthesis;

public class BulletPuppet : MonoBehaviour {
    public bool SkipDelete = false;
    public PhysicsObject BulletRep;

    public void Start() {
        BulletManager.RegisterPuppet(this);
    }

    public void OnDestroy() {
        if (!SkipDelete)
            BulletManager.UnregisterPuppet(this);
    }
}
