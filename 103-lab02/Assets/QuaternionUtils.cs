using UnityEditor.AnimatedValues;
using UnityEngine;

namespace QuaternionUtils {


    public static class QuaternionUtils {
        
        public static Quaternion Conjugate(this Quaternion quaternion) {
            return new Quaternion(-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w);
        }

        public static Quaternion Multiply(this Quaternion a, Quaternion b) {
            float x = a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y;
            float y = a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z;
            float z = a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x;
            float w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;

            return new Quaternion(x, y, z, w);
        }

        public static Quaternion RotateAround(this Quaternion quaternion, Vector3 dir, float angle) {
            dir = dir.normalized;
            float halfAngle = angle / 2;
            float real = Mathf.Cos(halfAngle), imag = Mathf.Sin(halfAngle); // in radians
            Quaternion rot = new Quaternion(imag * dir.x, imag * dir.y, imag * dir.z, real);
            return rot.Multiply(quaternion).normalized;
        }


        public static Quaternion Add(this Quaternion a, Quaternion b) {
            return new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
        }
    }
}
