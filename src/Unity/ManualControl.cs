using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Globalization;

public class ManualControl : MonoBehaviour
{
    [Header("移动参数")]
    public float maxSpeed = 0.35f;            // 最大前进/后退速度 (m/s)
    public float accel = 0.1f;               // 平移加速度 (m/s²)

    [Header("旋转参数")]
    public float maxTurnSpeed = 1800;         // 最大旋转速度 (°/s)
    public float angularAccel = 3600;        // 角加速度 (°/s²)

    [Header("贴地设置")]
    public float heightOffset = 1.0f;
    public Transform robot;

    private float currentSpeed = 0f;
    private float currentAngularSpeed = 0f;
    private float pitchAngle = -10f;
    private float rollAngle = 0f;
    private float yawAngle = 0f; // 代替 euler.y，控制绕 Y 轴的旋转

    private float terrainY = 0f;

    private bool enable_manual_control = false;
    private CameraAutoCapture captureScript;
    void Start()
    {
        // ✅ 找到 Main Camera 并获取脚本
        GameObject camObj = GameObject.FindWithTag("MainCamera");

        if (camObj != null)
        {
            captureScript = camObj.GetComponent<CameraAutoCapture>();

            if (captureScript == null)
                Debug.LogError("CameraAutoCapture 脚本未挂在 Main Camera 上！");
        }
        else
        {
            Debug.LogError("找不到名为 'Main Camera' 的 GameObject！");
        }
    }
    void Update()
    {
        enable_manual_control = captureScript.enable_manual_control;
        if (enable_manual_control)
        {
            if (robot == null) return;

            // ===== 1. 判断目标速度（线速度 & 角速度） =====
            float targetSpeed = 0f;
            float targetAngular = 0f;

            if (Input.GetKey(KeyCode.W)) targetSpeed = maxSpeed;
            else if (Input.GetKey(KeyCode.S)) targetSpeed = -maxSpeed;

            if (Input.GetKey(KeyCode.A)) targetAngular = -maxTurnSpeed;
            else if (Input.GetKey(KeyCode.D)) targetAngular = maxTurnSpeed;

            // ===== 2. 平滑速度插值 =====
            currentSpeed = Mathf.MoveTowards(currentSpeed, targetSpeed, accel * Time.deltaTime);
            currentAngularSpeed = Mathf.MoveTowards(currentAngularSpeed, targetAngular, angularAccel * Time.deltaTime);

            // ===== 3. 执行旋转（绕 Y 轴） =====
            if (Mathf.Abs(currentAngularSpeed) > 0.01f)
            {
                robot.Rotate(Vector3.up, currentAngularSpeed * Time.deltaTime);
            }

            // ===== 4. 执行移动（带地形贴地） =====
            if (Mathf.Abs(currentSpeed) > 0.001f)
            {
                Vector3 direction = robot.forward;
                Vector3 newPosXZ = robot.position + direction * currentSpeed * Time.deltaTime;


                if (Terrain.activeTerrain != null)
                {
                    terrainY = Terrain.activeTerrain.SampleHeight(newPosXZ)
                             + Terrain.activeTerrain.transform.position.y
                             + heightOffset;
                }

                // 添加“上下颠簸”：正弦波扰动 + 噪声
                float bumpAmplitude = 0.05f; // 颠簸强度
                float bumpFrequency = 5.0f;  // 颠簸频率
                float bumpNoise = Mathf.PerlinNoise(Time.time * 10f, 0.0f); // 加入一点随机性
                float bumpOffset = Mathf.Sin(Time.time * bumpFrequency) * bumpAmplitude + bumpNoise * 0.05f; ;
                terrainY += bumpOffset;

                // 更新位置
                robot.position = new Vector3(newPosXZ.x, terrainY, newPosXZ.z);

                // === Pitch（前后坡度） ===


                // ==== 强化车辆姿态摆动 ====
                
                float pitchAmplitude = 0.2f; // 前后俯仰
                float rollAmplitude = 0.3f;  // 左右侧倾
                float pitch = Mathf.Sin(Time.time * 10f) * pitchAmplitude + 10;
                float roll = Mathf.Sin(Time.time * 12f + 1f) * rollAmplitude;
                float yaw = Mathf.Sin(Time.time * 8f) * 0.5f;  // 轻微摇头（可调小）
                // Vector3 euler = robot.rotation.eulerAngles;
                // robot.rotation = Quaternion.Euler(pitch, (euler.y + yaw), roll);
            }
        }

    }
}
