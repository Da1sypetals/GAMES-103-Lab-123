using System;
using System.CodeDom.Compiler;
using System.Collections;
using System.Collections.Generic;
using MatrixUtils;
using Unity.VisualScripting;
using UnityEditor.Profiling;
using UnityEngine;
using UnityEngine.SceneManagement;

public class ShapeMatching : MonoBehaviour {
    [SerializeField] private float KFrictionTangent = .6f;
    [SerializeField] private float KVelocityConservationNormal = .22f;
    [SerializeField] private Vector3 initialVelocity;
    [SerializeField, Range(0f, 0.001f)] private float damping;

    private MeshFilter meshFilter;
    private Mesh mesh;

    /* Vertex-wise properties */
    private Vector3[] relativeOriginal;
    private Vector3[] velocity;
    private Vector3[] position;
    private Vector3[] tempPosition;

    // others
    private int numVertex;
    private Matrix3 ArightInversed;

    private Vector3 center;

    private Matrix3 rotationMatrix {
        get {
            return Matrix3.Rotate(transform.rotation);
        }
    }

    private Action CollideWall, CollideFloor;
    private Action EnforceWall, EnforceFloor;


    private void Awake() {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        numVertex = mesh.vertices.Length;

        Vector3 verticesAccumulated = Vector3.zero;
        var verticesCache = mesh.vertices;
        foreach (Vector3 vertex in verticesCache) {
            verticesAccumulated += vertex;
        }
        Vector3 verticesAverage = verticesAccumulated / (float)numVertex;
        Debug.Log(verticesAverage);
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            verticesCache[ivertex] -= verticesAverage;
        }
        mesh.vertices = verticesCache;

        /* Mesh initialized */

        position = new Vector3[numVertex];
        tempPosition = new Vector3[numVertex];
        velocity = new Vector3[numVertex];
        relativeOriginal = mesh.vertices;

        Matrix3 rot = this.rotationMatrix;
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            position[ivertex] = transform.position + rot * relativeOriginal[ivertex];
            velocity[ivertex] = initialVelocity;
        }

        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            ArightInversed += Matrix3.OuterProduct(relativeOriginal[ivertex], relativeOriginal[ivertex]);
        }

        CollideFloor = CollideActionGetter(Config.InsideFloor, Config.floorNormal, Config.FloorEnforcedPosition);
        CollideWall = CollideActionGetter(Config.InsideWall, Config.wallNormal, Config.WallEnforcedPosition);
        
        EnforceFloor = EnforceActionGetter(Config.InsideFloor, Config.floorNormal, Config.FloorEnforcedPosition);
        EnforceWall = EnforceActionGetter(Config.InsideWall, Config.wallNormal, Config.WallEnforcedPosition);

    }


    private void IndependentUpdateVelocity() {
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            velocity[ivertex] += Config.gravity * Config.deltaTime;
            velocity[ivertex] *= Mathf.Exp(-velocity[ivertex].magnitude * damping);
        }
    }

    private void IndependentUpdatePosition() {
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            tempPosition[ivertex] = position[ivertex] + velocity[ivertex] * Config.deltaTime;
        }
    }


    private void Rigidify() {
        center = Vector3.zero;
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            center += tempPosition[ivertex];
        }
        center /= (float)numVertex;
        Matrix3 Aleft = Matrix3.zero;
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            Aleft += Matrix3.OuterProduct(tempPosition[ivertex] - center, relativeOriginal[ivertex]);
        }
        Matrix3 A = Aleft * ArightInversed;
        (Matrix3 R, Matrix3 S) = A.PolarDecompose();

        transform.position = center;
        transform.rotation = Matrix3.ToQuaternion(R);

    }

    private void UpdateKinematics() {
        Matrix3 rot = this.rotationMatrix;
        for (int ivertex = 0; ivertex < numVertex; ivertex++) {
            velocity[ivertex] += (transform.position + rot * relativeOriginal[ivertex] - tempPosition[ivertex]) / Config.deltaTime;
            position[ivertex] = transform.position + rot * relativeOriginal[ivertex];
        }
    }

    private void Update() {
        if (Input.GetKeyDown(KeyCode.Space)) {
            SceneManager.LoadScene(0);
        }
        for (int iter = 0; iter < Config.stepsPerFrame; iter++) {
            IndependentUpdateVelocity();

            CollideFloor();
            CollideWall();
            IndependentUpdatePosition();

            Rigidify();
            UpdateKinematics();

        }
    }

    private Action CollideActionGetter(Predicate<Vector3> Inside, Vector3 normal, Func<Vector3, Vector3> EnforcedPosition) {
        return () => {

            for (int ivertex = 0; ivertex < numVertex; ivertex++) {
                if (!Inside(tempPosition[ivertex] + Config.margin)) {
                    continue;
                }
                float _velocityAgainstMagnitude = Vector3.Dot(velocity[ivertex], normal);
                if (_velocityAgainstMagnitude >= 0) {
                    continue;
                }
                tempPosition[ivertex] = EnforcedPosition(tempPosition[ivertex]);

                Vector3 _velocityNormal = _velocityAgainstMagnitude * normal;
                Vector3 _velocityTangent = velocity[ivertex] - _velocityNormal;
                float a = Mathf.Max(0, 1 - KFrictionTangent * (1 + KVelocityConservationNormal) * _velocityNormal.magnitude / _velocityTangent.magnitude);
                Vector3 _velocityNewNormal = -KVelocityConservationNormal * _velocityNormal;
                Vector3 _velocityNewTangent = a * _velocityTangent;
                velocity[ivertex] = _velocityNewNormal + _velocityNewTangent;

            }

        };
    }
    
    
    private Action EnforceActionGetter(Predicate<Vector3> Inside, Vector3 normal, Func<Vector3, Vector3> EnforcedPosition) {
        return () => {

            for (int ivertex = 0; ivertex < numVertex; ivertex++) {
                if (!Inside(tempPosition[ivertex])) {
                    continue;
                }
                // float _velocityAgainstMagnitude = Vector3.Dot(velocity[ivertex], normal);
                // if (_velocityAgainstMagnitude >= 0) {
                    // continue;
                // }
                tempPosition[ivertex] = EnforcedPosition(tempPosition[ivertex]);

            }

        };
    }

}
