using System;
using UnityEngine;

namespace MatrixUtils {
    public static class SVDUtils {
        /// <summary>
        /// Returns U, singular and V. Mind that V is NOT transposed.
        /// </summary>
        public static (Matrix3, Matrix3, Matrix3) SVD(this Matrix3 matrix) {

            double[] sig = new double[3];
            double[,] u = new double[3, 3];
            double[,] v = new double[3, 3];

            for (int irow = 0; irow < 3; irow++) {
                for (int icol = 0; icol < 3; icol++) {
                    u[irow, icol] = matrix[irow, icol];
                }
            }

            svd.svdcmp(u, out sig, out v);

            Matrix3 U = new Matrix3((float)u[0, 0],
                (float)u[0, 1],
                (float)u[0, 2],
                (float)u[1, 0],
                (float)u[1, 1],
                (float)u[1, 2],
                (float)u[2, 0],
                (float)u[2, 1],
                (float)u[2, 2]);

            Matrix3 singular = Matrix3.Diagonal((float)sig[0], (float)sig[1], (float)sig[2]);

            Matrix3 Vtransposed = new Matrix3((float)v[0, 0],
                (float)v[0, 1],
                (float)v[0, 2],
                (float)v[1, 0],
                (float)v[1, 1],
                (float)v[1, 2],
                (float)v[2, 0],
                (float)v[2, 1],
                (float)v[2, 2]);


            // Debug.Log(U);
            // Debug.Log(singular);
            // Debug.Log(Vtransposed);

            return (U, singular, Vtransposed);


        }


        public static (Matrix3, Matrix3) PolarDecompose(this Matrix3 matrix) {

            (Matrix3 U, Matrix3 singular, Matrix3 V) = matrix.SVD();

            return (U * V.T, V * singular * V.T);

            // throw new NotImplementedException();
        }


    }
}
