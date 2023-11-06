using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using UnityEngine;
using MatrixUtils;

public class ClothSphereCollider : ClothCollider {

    private Vector3 center {
        get {
            return transform.position;
        }
    }
    [SerializeField] private float radius = 2.7f;


    override public bool Inside(Vector3 point) {
        float dist = (point - center).magnitude;
        return dist < radius;
    }


    override public Vector3 EnforcedCoord(Vector3 point) {
        Vector3 dir = (point - center).normalized;
        return center + dir * radius;
    }

}
