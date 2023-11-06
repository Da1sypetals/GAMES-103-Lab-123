using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MatrixUtils;

public static class Config {
    public static float deltaTime => ConfigManager.instance.deltaTime;

    public static Vector3 gravity => Vector3.down * 9.8f * ConfigManager.instance.gravityScale;

    public static int stepsPerFrame => ConfigManager.instance.stepsPerFrameMassSpring;
    public static int stepsPerFramePBD => ConfigManager.instance.stepsPerFramePBD;


}
