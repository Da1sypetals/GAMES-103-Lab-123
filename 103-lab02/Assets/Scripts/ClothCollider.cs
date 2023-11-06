using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using UnityEngine;
using MatrixUtils;

public abstract class ClothCollider : MonoBehaviour {

    public abstract bool Inside(Vector3 point);
    public abstract Vector3 EnforcedCoord(Vector3 point);
    
    
}
