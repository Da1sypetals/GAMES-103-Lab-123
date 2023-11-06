using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Linq;
using MatrixUtils;


public class TetraMesh {

    public int numTetra;
    public int numVertex;

    public int[,] tetraVertexIndices;
    public Vector3[] position;
    public Vector3[] velocity;
    public Vector3[] force;
    public Matrix3[] PKStress;
    // public Vector3[] potentialGradients;

    public Matrix3[] restMatrices;
    public float[] restMatricesDeternimant;
    public Matrix3[] restMatricesTransposed;
    public Matrix3[] deformedMatrices;
    public Matrix3[] FMatrices;
    

    public Vector3[] triangleVertices;
    public int[] triangles;

    public void Load(Mesh visualMesh) {
        {
            string fileContent = File.ReadAllText("Assets/house2.ele");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            numTetra = int.Parse(Strings[0]);
            tetraVertexIndices = new int[numTetra, 4];

            for (int itetra = 0; itetra < numTetra; itetra++) {
                tetraVertexIndices[itetra, 0] = int.Parse(Strings[itetra * 5 + 4]) - 1;
                tetraVertexIndices[itetra, 1] = int.Parse(Strings[itetra * 5 + 5]) - 1;
                tetraVertexIndices[itetra, 2] = int.Parse(Strings[itetra * 5 + 6]) - 1;
                tetraVertexIndices[itetra, 3] = int.Parse(Strings[itetra * 5 + 7]) - 1;
            }
        }
        {
            string fileContent = File.ReadAllText("Assets/house2.node");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            numVertex = int.Parse(Strings[0]);
            this.position = new Vector3[numVertex];
            for (int ivertex = 0; ivertex < numVertex; ivertex++) {
                position[ivertex].x = float.Parse(Strings[ivertex * 5 + 5]) * 0.4f;
                position[ivertex].y = float.Parse(Strings[ivertex * 5 + 6]) * 0.4f;
                position[ivertex].z = float.Parse(Strings[ivertex * 5 + 7]) * 0.4f;
            }

            //Centralize the model.
            Vector3 center = Vector3.zero;
            for (int ivertex = 0; ivertex < numVertex; ivertex++) {
                center += position[ivertex];
            }

            center = center / numVertex;

            for (int ivertex = 0; ivertex < numVertex; ivertex++) {
                position[ivertex] -= center;
                float temp = position[ivertex].y;
                position[ivertex].y = position[ivertex].z;
                position[ivertex].z = temp;
            }
        }

        //Create triangle mesh.
        triangleVertices = new Vector3[numTetra * 12];
        triangles = new int[numTetra * 12];
        
        this.UpdateTriangleMesh();
        this.ExportTriangleMesh(visualMesh);
        
        // visualMesh.vertices = triangleVertices;
        // visualMesh.triangles = triangles;
        visualMesh.RecalculateNormals();

        velocity = new Vector3[numVertex];
        force = new Vector3[numVertex];
        // potentialGradients = new Vector3[numVertex];
        
        
        // init
        restMatrices = new Matrix3[numTetra];
        deformedMatrices = new Matrix3[numTetra];
        FMatrices = new Matrix3[numTetra];
        PKStress = new Matrix3[numTetra];
        restMatricesTransposed = new Matrix3[numTetra];
        restMatricesDeternimant = new float[numTetra];
        

    }

    public List<List<int>> graphTopology;
    public void BuildTopology() {
        // Build and adjacency list to present the topology of tetramesh.
        graphTopology = Enumerable.Range(0, numVertex)
            .Select(_ => new List<int>())
            .ToList();
        List<HashSet<int>> graphTopologySet = Enumerable.Range(0, numVertex)
            .Select(_ => new HashSet<int>())
            .ToList();
        for (int itetra = 0; itetra < numTetra; itetra++) {
            (int iv1, int iv2, int iv3, int iv4) = TetraVertexIndices(itetra);

            graphTopologySet[iv1].Add(iv2);
            graphTopologySet[iv1].Add(iv3);
            graphTopologySet[iv1].Add(iv4);
            
            graphTopologySet[iv2].Add(iv1);
            graphTopologySet[iv2].Add(iv3);
            graphTopologySet[iv2].Add(iv4);
            
            graphTopologySet[iv3].Add(iv2);
            graphTopologySet[iv3].Add(iv1);
            graphTopologySet[iv3].Add(iv4);
            
            graphTopologySet[iv4].Add(iv2);
            graphTopologySet[iv4].Add(iv3);
            graphTopologySet[iv4].Add(iv1);

        }
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            graphTopology[ivertex] = graphTopologySet[ivertex].ToList();
        }
    }
    

    public void ComputeRestMatrix() {
        for (int itetra = 0; itetra < numTetra; itetra++) {
            (Vector3 a, Vector3 b, Vector3 c, Vector3 d) = TetraVertices(itetra);

            Matrix3 restMatrixInv = Matrix3.Columns(b - a, c - a, d - a);
            Matrix3 restMatrix = restMatrixInv.inverse;

            restMatrices[itetra] = restMatrix;
            restMatricesTransposed[itetra] = restMatrix.T;
            restMatricesDeternimant[itetra] = restMatrix.determinant;

        }
    }

    public void UpdateTriangleMesh() {
        int vertexIndex = 0;
        for (int itetra = 0; itetra < numTetra; itetra++) {
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 0]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 2]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 1]];

            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 0]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 3]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 2]];

            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 0]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 1]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 3]];

            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 1]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 2]];
            triangleVertices[vertexIndex++] = position[tetraVertexIndices[itetra, 3]];
        }

        for (int itriangle = 0; itriangle < numTetra * 4; itriangle++) {
            triangles[itriangle * 3 + 0] = itriangle * 3 + 0;
            triangles[itriangle * 3 + 1] = itriangle * 3 + 1;
            triangles[itriangle * 3 + 2] = itriangle * 3 + 2;
        }
    }


    public void ExportTriangleMesh(Mesh visualMesh) {
        visualMesh.vertices = this.triangleVertices;
        visualMesh.triangles = this.triangles;
    }

    public (Vector3, Vector3, Vector3, Vector3) TetraVertices(int itetra) {
        int iv0 = tetraVertexIndices[itetra, 0];
        int iv1 = tetraVertexIndices[itetra, 1];
        int iv2 = tetraVertexIndices[itetra, 2];
        int iv3 = tetraVertexIndices[itetra, 3];

        return (position[iv0], position[iv1], position[iv2], position[iv3]);

    }
    
    public (int, int, int, int) TetraVertexIndices(int itetra) {
        int iv0 = tetraVertexIndices[itetra, 0];
        int iv1 = tetraVertexIndices[itetra, 1];
        int iv2 = tetraVertexIndices[itetra, 2];
        int iv3 = tetraVertexIndices[itetra, 3];

        return (iv0, iv1, iv2, iv3);
    }

    public void Run(Mesh visualMesh) {
        this.Load(visualMesh);
        this.ComputeRestMatrix();
        this.ExportTriangleMesh(visualMesh);

    }

    public void UpdateVisual(Mesh visualMesh) {
        this.UpdateTriangleMesh();
        this.ExportTriangleMesh(visualMesh);
        visualMesh.RecalculateBounds();
        visualMesh.RecalculateNormals();
    }


}
