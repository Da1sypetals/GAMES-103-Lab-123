using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConfigManager : MonoBehaviour {

    public static ConfigManager instance;

    private void OnEnable() {
        instance = this;
    }

    [SerializeField, Range(.5f, 4f)] public float gravityScale;
    [SerializeField, Range(1, 10)] public int stepsPerFrame;
    [SerializeField] public float deltaTime;
    


}
