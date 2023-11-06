using System;
using System.Collections.Generic;
using MatrixUtils;
using QuaternionUtils;
using UnityEngine;
using Unity.VisualScripting;
using System.Linq;
using UnityEditor;
using UnityEngine.SceneManagement;

public class MeshRigidbody : MonoBehaviour {

    // Serialized properties
    [SerializeField] private float density = 1f;
    [SerializeField] private float KFrictionTangent = .6f;
    [SerializeField] private float KVelocityConservationNormal = .22f;
    [SerializeField] private Vector3 initialVelocity = new Vector3(64f, 32f, 8f);
    [SerializeField] private Vector3 initialAngularVelocity = new Vector3(3f, 3f, 3f);
    [SerializeField, Range(0, 0.0003f)] private float velocityDamping = 1e-4f;
    [SerializeField, Range(0, 0.0003f)] private float angularDamping = 1e-4f;


    private float mass;

    private Matrix3 inertiaRefInversed;
    // rotationMatrix is orthonormal
    private Matrix3 inertiaInversed {
        get {
            return this.rotationMatrix * this.inertiaRefInversed * this.rotationMatrix.T;
        }
    }

    // mesh
    private MeshFilter meshFilter;
    private Mesh mesh;
    private int numVertex;
    private int numTriangle;
    private List<Vector3> vertexOffsetOriginalList;


    // States

    private Vector3 velocity;

    private Vector3 angularVelocity;


    private Matrix3 rotationMatrix {
        get {
            return Matrix3.Rotate(transform.rotation);
        }
    }

    // variables to be computed:
    private Vector3 impulse;
    private Vector3 impulseTorque; // this is actually the grad of angular velocity to time.


    // Physics
    private int numCollisions;

    private Action CollideFloor, CollideWall;


    /* Initialization */
    private void Awake() {

        // init vertex-related variables
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        this.numVertex = mesh.vertices.Length;
        this.numTriangle = mesh.triangles.Length / 3;

        this.SolveInertialProperties();
        this.angularVelocity = initialAngularVelocity;
        this.velocity = initialVelocity;

        vertexOffsetOriginalList = Enumerable.Repeat(Vector3.zero, numVertex).ToList();

        if (transform.localScale != Vector3.one) {
            for (int ivertex = 0; ivertex < numVertex; ivertex++) {
                vertexOffsetOriginalList[ivertex] = Vector3.Scale(mesh.vertices[ivertex], transform.localScale);
            }
        } else {
            vertexOffsetOriginalList = new List<Vector3>(mesh.vertices);
        }

        CollideFloor = CheckCollisionAction(Config.InsideFloor, Config.floorNormal, Config.FloorEnforcedPosition);
        CollideWall = CheckCollisionAction(Config.InsideWall, Config.wallNormal, Config.WallEnforcedPosition);

        Debug.Log("awake");

    }

    private void SolveInertialProperties() {
        MeshInertialProperties inertialProperties = new MeshInertialProperties(mesh, density);
        inertialProperties.Run();

        this.mass = inertialProperties.mass;
        this.inertiaRefInversed = inertialProperties.momentOfInertia.inverse;
    }


    /* Simulation */
    private void InitFrameSimulation() {

        this.impulse = Vector3.zero;
        this.impulseTorque = Vector3.zero;
    }

    private void UpdateDynamics() {
        this.velocity += this.impulse / this.mass + Config.gravity * Config.deltaTime;
        this.angularVelocity += this.impulseTorque;

        this.velocity *= Mathf.Exp(-velocityDamping);
        this.angularVelocity *= Mathf.Exp(-angularDamping);
    }


    private void Step() {
        // symplectic euler
        transform.position += Config.deltaTime * this.velocity;
        float theta = angularVelocity.magnitude * Config.deltaTime;
        transform.rotation = transform.rotation.RotateAround(angularVelocity, theta);
    }

    private void Update() {

        if (Input.GetKeyDown(KeyCode.Space)) {
            SceneManager.LoadScene(0);
        }

        for (int step = 0; step < Config.stepsPerFrame; step++) {

            InitFrameSimulation();

            CollideWall();
            
            CollideFloor();

            UpdateDynamics();

            Step();
        }

    }


    private Action CheckCollisionAction(Predicate<Vector3> Inside, Vector3 normal, Func<Vector3, Vector3> EnforcedPosition) {

        return () => {
            int numCollisions = 0;
            Matrix3 rot = rotationMatrix;
            Vector3 impulseFloor = Vector3.zero, impulseTorqueFloor = Vector3.zero;
            Vector3 avgPenetration = Vector3.zero;
            for (int ivertex = 0; ivertex < numVertex; ivertex++) {
                Vector3 _relative = rot * vertexOffsetOriginalList[ivertex];
                Vector3 _position = transform.position + _relative;
                Vector3 _velocity = this.velocity + Vector3.Cross(this.angularVelocity, _relative);
                if (!Inside(_position)) {
                    continue;
                }
                float _velocityAgainstMagnitude = Vector3.Dot(_velocity, normal);
                if (_velocityAgainstMagnitude >= 0) {
                    continue;
                }
                // Debug.Log("collide");
                numCollisions += 1;
                Vector3 _velocityNormal = _velocityAgainstMagnitude * normal;
                Vector3 _velocityTangent = _velocity - _velocityNormal;

                float a = Mathf.Max(0, 1 - KFrictionTangent * (1 + KVelocityConservationNormal) * _velocityNormal.magnitude / _velocityTangent.magnitude);
                Vector3 _velocityNewNormal = -KVelocityConservationNormal * _velocityNormal;
                Vector3 _velocityNewTangent = a * _velocityTangent;
                Vector3 _velocityNew = _velocityNewNormal + _velocityNewTangent;

                Matrix3 _crossRelative = _relative.CrossMatrix();
                Matrix3 _KMatrix = Matrix3.identity / this.mass - _crossRelative * inertiaInversed * _crossRelative;
                Vector3 _impulse = _KMatrix.inverse * (_velocityNew - _velocity);
                Vector3 _impulseTorque = inertiaInversed * Vector3.Cross(_relative, _impulse);

                impulseFloor += _impulse;
                impulseTorqueFloor += _impulseTorque;
                avgPenetration += _position - EnforcedPosition(_position);
            }

            if (numCollisions != 0) {
                // Debug.Log(numCollisions);
                // Debug.Log(this.numCollisions);
                // Debug.Log(impulseFloor);
                this.impulse += impulseFloor / numCollisions;
                this.impulseTorque += impulseTorqueFloor / numCollisions;
                avgPenetration /= (float)numCollisions;
                transform.position -= avgPenetration;

            }

        };


    }


    


}
