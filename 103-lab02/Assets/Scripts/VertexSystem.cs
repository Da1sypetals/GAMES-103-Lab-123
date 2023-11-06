using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MatrixUtils;
using Unity.Collections.LowLevel.Unsafe;


public class VertexSystem {
    // data class.

    private Mesh mesh;

    public int numVertex { get; private set; }
    public Vector3[] _position;
    public Vector3[] position {
        // Cached.
        get {
            return _position;
        }
        set {
            mesh.vertices = value;
        }
    } // references the mesh vertices.

    public void UpdatePosition() {
        _position = mesh.vertices;
    }
    
    public Vector3[] tempPosition;
    public Vector3[] velocity;
    public int[] degree;

    // spring related
    public int numSpring { get; private set; }
    private List<float> restLengthList;
    private List<(int, int)> iSpringVertexList; // i -> index
    private float DeltaLength(int ispring) {
        (int i1, int i2) = iSpringVertexList[ispring];
        Vector3 x1 = mesh.vertices[i1];
        Vector3 x2 = mesh.vertices[i2];
        return (x2 - x1).magnitude - restLengthList[ispring];
    }
    private float stiffness;


    // System
    public VertexSystem(Mesh _mesh, float _stiffness) {
        mesh = _mesh;
        stiffness = _stiffness;

        numVertex = mesh.vertices.Length;
        velocity = new Vector3[numVertex];
        tempPosition = new Vector3[numVertex];
        degree = new int[numVertex];
        this.UpdatePosition();

        gradientList = Enumerable.Repeat(Vector3.zero, numVertex).ToList();
        stepList = Enumerable.Repeat(Vector3.zero, numVertex).ToList();
        hessianList = Enumerable.Repeat(Matrix3.zero, numVertex).ToList();
        naiveHessian = (1 / Mathf.Pow(Config.deltaTime, 2) + 4 * stiffness) * Matrix3.identity;

        BuildSprings();

        // Debug.Log($"vertex count : {numVertex}");
        // foreach (var pos in position) {
        // Debug.Log(pos);
        // }

    }

    // a desired way using diffable programming should be: 
    // ComputePotential();
    // Backward();
    // StoreGrad();
    // Backward();
    // StoreHessian();

    public float Potential() {
        float potential = 0f;
        for (int ispring = 0; ispring < numSpring; ispring++) {
            potential += .5f * stiffness * Mathf.Pow(DeltaLength(ispring), 2);
        }
        return potential;
    }

    private List<Vector3> gradientList;
    public List<Vector3> stepList;
    private List<Matrix3> hessianList;
    private Matrix3 naiveHessian;
    public void UpdatePotentialGradient() {

        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            // grad of gravitational energy = -g (negative g)
            gradientList[ivertex] = -Config.gravity;
        }

        for (int ispring = 0; ispring < numSpring; ispring++) {
            (int iv1, int iv2) = iSpringVertexList[ispring];
            float dist = (tempPosition[iv2] - tempPosition[iv1]).magnitude;
            // if (dist > 6 * restLengthList[ispring]) {
            // Debug.Log($"Spring connecting {iv1} and {iv2}");
            // }
            gradientList[iv1] += stiffness * (1 - restLengthList[ispring] / dist) * (tempPosition[iv1] - tempPosition[iv2]);
            gradientList[iv2] += stiffness * (1 - restLengthList[ispring] / dist) * (tempPosition[iv2] - tempPosition[iv1]);
        }

    }

    public void UpdatePotentialHessian() {
        throw new NotImplementedException();
    }

    public Vector3 PotentialGradient(int ivertex) {
        if (ivertex >= numVertex || ivertex < 0) {
            throw new IndexOutOfRangeException("vertex index must be within [0, numVertex)!");
        }

        // for each spring related to the vertex, compute the spring force applied to the vertex.
        // use the provided position.
        return gradientList[ivertex];

    }

    public Matrix3 PotentialHessian(int ivertex) {
        return naiveHessian;
    }

    // todo: apply gradient in an inverse fasion
    // i.e. loop through all edges instead of vertices and accumulate grad WRT the potential of each edge (spring).


    // Precomputing Methods
    private void BuildSprings() {
        HashSet<(int, int)> edgeSet = new HashSet<(int, int)>();
        int numTriangle = mesh.triangles.Length / 3;
        for (int itriangle = 0; itriangle < numTriangle; itriangle++) {
            int ibase = itriangle * 3;

            (int, int) edge1 = Utils.SortedPair(mesh.triangles[ibase], mesh.triangles[ibase + 1]);
            (int, int) edge2 = Utils.SortedPair(mesh.triangles[ibase + 1], mesh.triangles[ibase + 2]);
            (int, int) edge3 = Utils.SortedPair(mesh.triangles[ibase], mesh.triangles[ibase + 2]);

            edgeSet.Add(edge1);
            edgeSet.Add(edge2);
            edgeSet.Add(edge3);
        }
        iSpringVertexList = edgeSet.ToList();
        numSpring = iSpringVertexList.Count;
        restLengthList = Enumerable.Repeat(0f, numSpring).ToList();
        for (int ispring = 0; ispring < numSpring; ispring++) {
            (int iv1, int iv2) = iSpringVertexList[ispring];
            restLengthList[ispring] = (position[iv1] - position[iv2]).magnitude;

        }
    }


    public void UpdateConstraints() {

        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            degree[ivertex] = 0;
            stepList[ivertex] = tempPosition[ivertex] * .2f;
        }
        
        for (int ispring = 0; ispring < numSpring; ispring++) {
            (int iv1, int iv2) = iSpringVertexList[ispring];

            Vector3 dirJI = (tempPosition[iv1] - tempPosition[iv2]).normalized;
            // stepList[iv1] += .5f * (tempPosition[iv1] + tempPosition[iv2] + restLengthList[ispring] * dirJI);
            // stepList[iv2] += .5f * (tempPosition[iv1] + tempPosition[iv2] - restLengthList[ispring] * dirJI);
            float dist = (tempPosition[iv1] - tempPosition[iv2]).magnitude;
            stepList[iv1] += tempPosition[iv1] - .5f * (dist - restLengthList[ispring]) * dirJI;
            stepList[iv2] += tempPosition[iv2] + .5f * (dist - restLengthList[ispring]) * dirJI;
            degree[iv1] += 1;
            degree[iv2] += 1;
        }

        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            stepList[ivertex] /= ((float)degree[ivertex] + .2f);
        }

    }

}
