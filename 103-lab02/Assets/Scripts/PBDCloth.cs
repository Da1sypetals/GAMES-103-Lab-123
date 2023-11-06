using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using UnityEngine;
using MatrixUtils;
using UnityEditor.Profiling.Memory.Experimental;
using UnityEngine.UIElements;

public class PBDCloth : MonoBehaviour {

    // runner class.

    [SerializeField] private GameObject sphere;
    [SerializeField, Range(8, 128)] private int numIter = 64;
    [SerializeField, Range(0f, 0.03f)] private float velocityDamping = 0.01f;
    [SerializeField] private List<uint> fixedVertexIndices;
    [SerializeField, Range(.1f, 3f)] private float stiffnessScale;

    [SerializeField] private List<ClothCollider> clothColliderList;

    private float stiffnessPreset = 8000;


    private MeshFilter meshFilter;
    private Mesh mesh; // mesh is only used to construct VertexSystem. Do not modify mesh directly in this class.

    // Vertices
    private VertexSystem vertexSystem;
    private List<Vector3> vertexGradientList;
    private List<Matrix3> vertexHessianList;


    // intialization
    private void Awake() {

        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        Utils.ResizeMesh(mesh);

        vertexSystem = new VertexSystem(mesh, stiffnessScale * stiffnessPreset);
        vertexGradientList = Enumerable.Repeat(Vector3.zero, vertexSystem.numVertex).ToList();
        vertexHessianList = Enumerable.Repeat(Matrix3.zero, vertexSystem.numVertex).ToList();


    }


    private void UpdateVerticesPositionsSingleIter() {
        for (int ivertex = 0; ivertex < vertexSystem.numVertex; ivertex++) {
            vertexSystem.tempPosition[ivertex] = vertexSystem.stepList[ivertex];
        }
    }

    private void SolveIteration() {
        // Apply external forces and initial guess
        for (int ivertex = 0; ivertex < vertexSystem.numVertex; ivertex++) {
            vertexSystem.velocity[ivertex] *= Mathf.Exp(-velocityDamping * vertexSystem.velocity[ivertex].magnitude);
            vertexSystem.velocity[ivertex] += Config.gravity * Config.deltaTime;
            vertexSystem.tempPosition[ivertex] = vertexSystem.position[ivertex] + Config.deltaTime * vertexSystem.velocity[ivertex]; // why .2f is hardcoded here?
        }

        for (int iter = 0; iter < numIter; iter++) {
            vertexSystem.UpdateConstraints();
            UpdateVerticesPositionsSingleIter();
        }

    }

    private void Collide() {
        foreach (ClothCollider collider in clothColliderList) {
            for (int ivertex = 0; ivertex < vertexSystem.numVertex; ivertex++) {
                if (collider.Inside(vertexSystem.tempPosition[ivertex])) {
                    Vector3 newPosition = collider.EnforcedCoord(vertexSystem.tempPosition[ivertex]);
                    // vertexSystem.velocity[ivertex] += (newPosition - vertexSystem.tempPosition[ivertex]) / Config.deltaTime;

                    vertexSystem.tempPosition[ivertex] = newPosition;
                }
            }
        }
    }


    private void UpdateVelocities() {
        for (int ivertex = 0; ivertex < vertexSystem.numVertex; ivertex++) {
            vertexSystem.velocity[ivertex] = (vertexSystem.tempPosition[ivertex] - vertexSystem.position[ivertex]) / Config.deltaTime;

        }
    }


    private void UpdateMeshVertices() {
        // copy new x to original x (mesh vertices)
        foreach (uint ifixedVertices in fixedVertexIndices) {
            vertexSystem.velocity[ifixedVertices] = Vector3.zero;
            vertexSystem.tempPosition[ifixedVertices] = vertexSystem.position[ifixedVertices];
        }
        /* Mind that this line is value assignment rather than referencing */
        vertexSystem.position = vertexSystem.tempPosition;
    }

    // run
    private void Update() {
        for (int step = 0; step < Config.stepsPerFramePBD; step++) {
            vertexSystem.UpdatePosition();
            SolveIteration();
            Collide();
            UpdateVelocities();
            UpdateMeshVertices();
        }
    }

}
