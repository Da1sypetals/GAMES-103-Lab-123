using System;
using System.Collections;
using System.Collections.Generic;
using MatrixUtils;
using QuaternionUtils;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.VisualScripting;
using System.Linq;
using UnityEditor;
using UnityEngine.UIElements;


public static class MomentOfInertiaUtils
{


    private static float InertiaEntry(Vector3 a, Vector3 b, Vector3 c, Func<Vector3, float> func)
    {
        // volume / 20 == det(cols) / 120
        return Matrix3.Columns(a, b, c).determinant / 120f * (func(a) + func(b) + func(c) + func(a + b + c));
    }

    private static float Kxx(Vector3 vertex)
    {
        return vertex.y * vertex.y + vertex.z * vertex.z;
    }

    private static float Kyy(Vector3 vertex)
    {
        return vertex.x * vertex.x + vertex.z * vertex.z;
    }

    private static float Kzz(Vector3 vertex)
    {
        return vertex.x * vertex.x + vertex.y * vertex.y;
    }

    private static float Kxy(Vector3 vertex)
    {
        return vertex.x * vertex.y;
    }

    private static float Kyz(Vector3 vertex)
    {
        return vertex.y * vertex.z;
    }

    private static float Kzx(Vector3 vertex)
    {
        return vertex.z * vertex.x;
    }

    public static Matrix3 MomentOfInertiaTensor(Vector3 a, Vector3 b, Vector3 c)
    {
        // suppose mass center at (0, 0, 0), i.e. all vertices are relative coords

        float kxx = InertiaEntry(a, b, c, Kxx);
        float kyy = InertiaEntry(a, b, c, Kyy);
        float kzz = InertiaEntry(a, b, c, Kzz);
        float kxy = InertiaEntry(a, b, c, Kxy);
        float kyz = InertiaEntry(a, b, c, Kyz);
        float kzx = InertiaEntry(a, b, c, Kzx);

        return new Matrix3(
            kxx,
            -kxy,
            -kzx,
            -kxy,
            kyy,
            -kyz,
            -kzx,
            -kyz,
            kzz
        );
    }


}
