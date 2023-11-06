using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MatrixUtils;

public static class Utils {

    public static (int, int) SortedPair(int x, int y) {
        return (Math.Min(x, y), Math.Max(x, y));
    }


    public static List<int> ToList(this (int, int) tuple) {
        List<int> result = new List<int>();
        result.Add(tuple.Item1);
        result.Add(tuple.Item2);
        return result;
    }

    public static List<int> ToList(this (int, int, int) tuple) {
        List<int> result = new List<int>();
        result.Add(tuple.Item1);
        result.Add(tuple.Item2);
        result.Add(tuple.Item3);
        return result;
    }

    public static void ResizeMesh(Mesh mesh) {
        // Resize the mesh. Code is copied.
        int n = 16;
        Vector3[] X = new Vector3[n * n];
        Vector2[] UV = new Vector2[n * n];
        int[] T = new int[(n - 1) * (n - 1) * 6];
        for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++) {
            X[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
            UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
        }
        int t = 0;
        for (int j = 0; j < n - 1; j++)
        for (int i = 0; i < n - 1; i++) {
            T[t * 6 + 0] = j * n + i;
            T[t * 6 + 1] = j * n + i + 1;
            T[t * 6 + 2] = (j + 1) * n + i + 1;
            T[t * 6 + 3] = j * n + i;
            T[t * 6 + 4] = (j + 1) * n + i + 1;
            T[t * 6 + 5] = (j + 1) * n + i;
            t++;
        }
        mesh.vertices = X;
        mesh.triangles = T;
        mesh.uv = UV;
        mesh.RecalculateNormals();
    }

}
