using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MatrixUtils;

public class MeshInertialProperties {

    private Mesh mesh;
    private float density;
    
    private int numVertex;
    private int numTriangle;
    private int numEdge;
    private List<Vector3> normalList;

    public float volume { get; private set; }
    public float mass { get; private set; }
    public Matrix3 momentOfInertia { get; private set; }

    public Vector3[] vertices;
    public int[] triangles;
    

    public MeshInertialProperties(Mesh _mesh, float _density) {
        mesh = _mesh;
        density = _density;
        
        numVertex = mesh.vertices.Length;
        numTriangle = mesh.triangles.Length / 3;
        this.triangles = mesh.triangles;

        normalList = Enumerable.Repeat(Vector3.zero, numTriangle).ToList();

    }


    public void ComputeNormal() {


        for (int itriangle = 0; itriangle < numTriangle; itriangle++) {
            (Vector3 a, Vector3 b, Vector3 c) = TriangleVertex(itriangle);
            Vector3 ac = c - a, ab = b - a;
            Vector3 normal = Vector3.Cross(ac, ab).normalized;
            normalList[itriangle] = normal;
        }   
        
    }

    public (Vector3, Vector3, Vector3) TriangleVertex(int itriangle) {
        if (itriangle >= numTriangle) {
            throw new IndexOutOfRangeException("Triangle index out of range !");
        }
        (int iv1, int iv2, int iv3) = TriangleVertexIndices(itriangle);
        // Debug.Log($"{iv1}, {iv2}, {iv3}");

        return (this.vertices[iv1], this.vertices[iv2], this.vertices[iv3]);

    }
    
    
    public (int, int, int) TriangleVertexIndices(int itriangle) {
        if (itriangle >= numTriangle) {
            throw new IndexOutOfRangeException("Triangle index out of range !");
        }
        int iv1 = itriangle * 3;
        int iv2 = iv1 + 1;
        int iv3 = iv1 + 2;

        return (this.triangles[iv1], this.triangles[iv2], this.triangles[iv3]);

    }

    public void ComputeMassCenterAndResetOffset() {
        Vector3 massCenterOffset = Vector3.zero;
        this.volume = 0f;
        for (int itriangle = 0; itriangle < numTriangle; itriangle++) {
            (Vector3 a, Vector3 b, Vector3 c) = TriangleVertex(itriangle);

            Vector3 someVertex = this.vertices[this.triangles[itriangle * 3]];
            float sign = Mathf.Sign(Vector3.Dot(normalList[itriangle], someVertex));
            float deltaVolume = sign * TetrahedronVolume(itriangle);
            this.volume += deltaVolume;
            massCenterOffset += deltaVolume * (a + b + c /* + Vector3.zero */) / 4f;
        }
        this.mass = this.volume * density;
        massCenterOffset /= this.volume;

        Vector3[] newVertices = new Vector3[numVertex];
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            newVertices[ivertex] = this.vertices[ivertex] - massCenterOffset;
        }
        mesh.vertices = newVertices;
    }

    
    public void ComputeMomentOfInertiaTensor() {
        this.momentOfInertia = Matrix3.zero;
        for (int itriangle = 0; itriangle < numTriangle; itriangle++) {
            Vector3 someVertex = this.vertices[this.triangles[itriangle * 3]];
            float sign = Mathf.Sign(Vector3.Dot(normalList[itriangle], someVertex));
            this.momentOfInertia += sign * TetrahedronMomentOfInertia(itriangle);
        }
        this.momentOfInertia *= density;
    }
    
    
    public float TetrahedronVolume(int itriangle) {
        (Vector3 a, Vector3 b, Vector3 c) = TriangleVertex(itriangle);
        
        return Matrix3.Columns(a, b, c).determinant / 6f;
    }
    
    public Matrix3 TetrahedronMomentOfInertia(int itriangle) {
        (Vector3 a, Vector3 b, Vector3 c) = TriangleVertex(itriangle);

        return MomentOfInertiaUtils.MomentOfInertiaTensor(a, b, c);
    }


    public void CacheVertices() {
        this.vertices = mesh.vertices;
    }


    public void Run() {
        CacheVertices();
        ComputeNormal();
        Debug.Log("normal done");
        ComputeMassCenterAndResetOffset();
        Debug.Log("mass done");
        CacheVertices();
        ComputeMomentOfInertiaTensor();
        Debug.Log("inertia done");

    }

    
}
