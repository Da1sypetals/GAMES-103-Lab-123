using UnityEngine;

public static class Config {
    public static Vector3 gravity = Vector3.down * 9.8f * 2;
    public static float deltaTime = 1e-3f;
    public static float stepsPerFrame = 10;
    public static float groundY = -3f;
}