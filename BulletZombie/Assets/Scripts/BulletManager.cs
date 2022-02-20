using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class BulletManager {
    private static List<BulletPuppet> _puppets;

    public static void RegisterPuppet(BulletPuppet puppet) {
        if (_puppets.Exists(x => x.GetHashCode() == puppet.GetHashCode())) {
            throw new Exception("Puppet is already registered");
        }

        _puppets.Add(puppet);
    }

    public static void UnregisterPuppet(BulletPuppet puppet) {
        if (!_puppets.Exists(x => x.GetHashCode() == puppet.GetHashCode())) {
            throw new Exception("Puppet isn't registered");
        }

        _puppets.Remove(puppet);
    }

    public static void UpdatePuppets() {
        _puppets.ForEach(puppet => {
            if (puppet.gameObject.activeSelf && puppet.enabled) {
                if (puppet.BulletRep != null) {
                    puppet.transform.position = Util.ToUnity(puppet.BulletRep.Position);
                    puppet.transform.rotation = Util.ToUnity(puppet.BulletRep.Rotation);
                }
            }
        });
    }
}
