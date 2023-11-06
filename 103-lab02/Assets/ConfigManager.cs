using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConfigManager : MonoBehaviour {

    public static ConfigManager instance;

    private void OnEnable() {
        instance = this;
    }

    [SerializeField, Range(8f, 24f)] public float gravityScale;
    [SerializeField] public float deltaTime;
    [SerializeField, Range(1, 20)] public int stepsPerFrameMassSpring;
    [SerializeField, Range(1, 20)] public int stepsPerFramePBD;


}
