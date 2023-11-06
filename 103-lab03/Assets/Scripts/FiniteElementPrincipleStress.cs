using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using MatrixUtils;
using Unity.VisualScripting;
using UnityEngine.UIElements;

public class FiniteElementPrincipleStress : MonoBehaviour {


    [SerializeField] private float lambda = 19999;
    [SerializeField] private float mu = 9999;

    [SerializeField, Range(0.0005f, 0.01f)]
    private float velocityDamping;

    [SerializeField, Range(0f, .99f)] private float velocityConservationRatio;
    [SerializeField, Range(0, 5f)] private float verticalVelocity = 4f;
    
    
    [Space(16)]
    [Header("Laplacian Smoothing")]
    [SerializeField] private bool useLaplacianSmoothing = false;
    // it is experimented that settting the parameter to .02f solves the oscillation problem effectively
    [SerializeField, Range(0f, .08f)] private float laplacinaSmoothingRatio = .02f;


    private MeshFilter meshFilter;
    private Mesh mesh;
    private TetraMesh tetraMesh;

    private float totalPotentialEnergy;


    private void Awake() {

        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        tetraMesh = new TetraMesh();
        tetraMesh.Run(mesh);
        if (useLaplacianSmoothing) {
            tetraMesh.BuildTopology();
        }
        // throw new NotImplementedException();
    }

    private void InitFrame() {
        for (int ivertex = 0; ivertex < tetraMesh.numVertex; ivertex++) {

            tetraMesh.force[ivertex] = Vector3.zero;

        }


        for (int itetra = 0; itetra < tetraMesh.numTetra; itetra++) { }
    }

    private void ComputeF() {
        for (int itetra = 0; itetra < tetraMesh.numTetra; itetra++) {
            (Vector3 a, Vector3 b, Vector3 c, Vector3 d) = tetraMesh.TetraVertices(itetra);


            tetraMesh.deformedMatrices[itetra] = Matrix3.Columns(b - a, c - a, d - a);

            // matmul
            tetraMesh.FMatrices[itetra] = tetraMesh.deformedMatrices[itetra] * tetraMesh.restMatrices[itetra];

        }
    }

    private void ComputeStressForce() {
        for (int itetra = 0; itetra < tetraMesh.numTetra; itetra++) {

            Matrix3 F = tetraMesh.FMatrices[itetra];
            
            (Matrix3 U, Matrix3 singular, Matrix3 V) = F.SVD();
            Matrix3 differentiatedSingular = StVKEnergyDifferentiated(singular);

            tetraMesh.PKStress[itetra] = U * differentiatedSingular * V.T;

            (int iv0, int iv1, int iv2, int iv3) = tetraMesh.TetraVertexIndices(itetra);
            Matrix3 forceCols = -1 / (6f * tetraMesh.restMatricesDeternimant[itetra]) * tetraMesh.PKStress[itetra] * tetraMesh.restMatricesTransposed[itetra];
            (Vector3 f1, Vector3 f2, Vector3 f3) = forceCols.DeconstructColumns();
            Vector3 f0 = Vector3.zero - f1 - f2 - f3;

            tetraMesh.force[iv0] += f0;
            tetraMesh.force[iv1] += f1;
            tetraMesh.force[iv2] += f2;
            tetraMesh.force[iv3] += f3;
        }
    }


    private void ApplyDynamics() {

        // differentiate and propagate gradients to tetramesh.potentialGradients;

        for (int ivertex = 0; ivertex < tetraMesh.numVertex; ivertex++) {

            tetraMesh.velocity[ivertex] += Config.gravity * Config.deltaTime;
            tetraMesh.velocity[ivertex] += tetraMesh.force[ivertex] * Config.deltaTime;
            tetraMesh.velocity[ivertex] *= Mathf.Exp(-velocityDamping * Mathf.Pow(tetraMesh.velocity[ivertex].magnitude, .8f));


        }
        
        if (useLaplacianSmoothing) {
            LaplacianSmoothingBlend();
        }

        for (int ivertex = 0; ivertex < tetraMesh.numVertex; ivertex++) {
            tetraMesh.position[ivertex] += tetraMesh.velocity[ivertex] * Config.deltaTime;
        }

    }

    public void Collide() {
        for (int ivertex = 0; ivertex < tetraMesh.numVertex; ivertex++) {
            if (transform.position.y + tetraMesh.position[ivertex].y < Config.groundY) {
                tetraMesh.position[ivertex].y = Config.groundY - transform.position.y;
                tetraMesh.velocity[ivertex].y *= -velocityConservationRatio;
            }
        }
    }


    private void Step() {
        InitFrame();
        ComputeF();
        ComputeStressForce();
        ApplyDynamics();
        Collide();

        tetraMesh.UpdateVisual(mesh);
    }

    private void Update() {
        for (int iter = 0; iter < Config.stepsPerFrame; iter++) {
            JumpAction();
            Step();
        }
    }

    private void JumpAction() {
        if (Input.GetKeyDown(KeyCode.Space)) {
            Jump(KeyCode.Space);
        }
        if (Input.GetKeyDown(KeyCode.W)) {
            Jump(KeyCode.W);
        }
        if (Input.GetKeyDown(KeyCode.A)) {
            Jump(KeyCode.A);
        }
        if (Input.GetKeyDown(KeyCode.S)) {
            Jump(KeyCode.S);
        }
        if (Input.GetKeyDown(KeyCode.D)) {
            Jump(KeyCode.D);
        }
    }


    public void Jump(KeyCode keyCode) {
        for (int ivertex = 0; ivertex < tetraMesh.numVertex; ivertex++) {
            tetraMesh.velocity[ivertex].y += verticalVelocity;
            if (keyCode == KeyCode.W) {
                tetraMesh.velocity[ivertex].z += .5f;
            }
            if (keyCode == KeyCode.A) {
                tetraMesh.velocity[ivertex].x -= .5f;
            }
            if (keyCode == KeyCode.S) {
                tetraMesh.velocity[ivertex].z -= .5f;
            }
            if (keyCode == KeyCode.D) {
                tetraMesh.velocity[ivertex].x += .5f;
            }
        }
        
    }


    public Matrix3 StVKEnergyDifferentiated(Matrix3 singular) {
        // expression is given by sympy.
        float s0 = lambda, s1 = mu;
        float lambda0 = singular[0, 0];
        float lambda1 = singular[1, 1];
        float lambda2 = singular[2, 2];
        float sumLambdaSquared = lambda0 * lambda0 + lambda1 * lambda1 + lambda2 * lambda2;
        // throw new NotImplementedException();
        
        float d0 = lambda0 * (2 * s0 * (sumLambdaSquared - 3) + s1 * (lambda0 * lambda0 - 1));
        float d1 = lambda1 * (2 * s0 * (sumLambdaSquared - 3) + s1 * (lambda1 * lambda1 - 1));
        float d2 = lambda2 * (2 * s0 * (sumLambdaSquared - 3) + s1 * (lambda2 * lambda2 - 1));
        
        return Matrix3.Diagonal(d0, d1, d2);
    }

    public Matrix3 NeoHookeanEnergyDifferentiated(Matrix3 singular) {
        float s0 = lambda, s1 = mu;
        float lambda0 = singular[0, 0];
        float lambda1 = singular[1, 1];
        float lambda2 = singular[2, 2];
        float sumLambdaSquared = lambda0 * lambda0 + lambda1 * lambda1 + lambda2 * lambda2;
        throw new NotImplementedException();
        
        
        // return Matrix3.Diagonal(d0, d1, d2);
    }
    
    
    public void LaplacianSmoothingBlend() {
        for (int ivertex = 0; ivertex < tetraMesh.numVertex; ivertex++) {
            // ii: index of index.
            Vector3 neighborVelocity = Vector3.zero;
            int numNeighbors = tetraMesh.graphTopology[ivertex].Count;
            for (int iiadjVertex = 0; iiadjVertex < numNeighbors; iiadjVertex++) {
                int iadjVertex = tetraMesh.graphTopology[ivertex][iiadjVertex];
                neighborVelocity += tetraMesh.velocity[iadjVertex];
            }
            neighborVelocity /= (float)numNeighbors;
            tetraMesh.velocity[ivertex] = laplacinaSmoothingRatio * neighborVelocity + (1 - laplacinaSmoothingRatio) * tetraMesh.velocity[ivertex];
        }
    }

}

