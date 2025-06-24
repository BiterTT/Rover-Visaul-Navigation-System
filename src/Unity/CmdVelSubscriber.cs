using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CmdVelSubscriber : MonoBehaviour
{
    public string cmdVelTopic = "/cmd_vel";      // ROS 话题名
    public Transform targetTransform; // 拖入你的 MainCamera 或机器人对象
    public float linearScale = 1.0f;           // 缩放速度（单位转换）
    public float angularScale = 1.0f;          // 缩放角速度
    public float linearSpeedMultiplier = 2.0f;     // 线速度缩放（单位转换或调节响应）
    public float angularSpeedMultiplier = 2.0f;    // 角速度缩放（单位转换）
    public float heightOffset = 2.0f;              // 离地高度（例如摄像机安装在机器人上方）
    public float linearAccel = 1.0f;               // 最大线加速度（单位/s²）
    public float angularAccel = 1.0f;              // 最大角加速度（单位 rad/s²）

    // ROS 实际速度目标值（实时接收）
    private float targetLinear = 0f;
    private float targetAngular = 0f;

    // 当前速度（插值平滑）
    private float currentLinear = 0f;
    private float currentAngular = 0f;

    private ROSConnection ros;

    void Start()
    {
        if (targetTransform == null)
            targetTransform = this.transform;

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, CmdVelCallback);
    }

    void Update()
    {
        if (targetTransform == null) return;

        Vector3 euler = targetTransform.eulerAngles;
        Debug.Log($"[CmdVel] Linear: {currentLinear:F3} m/s, Angular: {currentAngular:F3} rad/s");
        Debug.Log($"[Position] {targetTransform.position}");

        // 1. 平滑过渡当前线速度和角速度（避免突变）
        currentLinear = Mathf.MoveTowards(currentLinear, targetLinear, linearAccel * Time.deltaTime);
        currentAngular = Mathf.MoveTowards(currentAngular, targetAngular, angularAccel * Time.deltaTime);

        // 2. 计算平面移动方向（Unity中forward为 +Z，正好对应ROS的 +X）
        Vector3 forwardMove = targetTransform.forward * currentLinear * linearSpeedMultiplier * Time.deltaTime;

        // 3. 预测下一个平面位置（仅XZ）
        Vector3 flatNextPos = targetTransform.position + forwardMove;

        // 4. 贴地：使用地形高度采样 + 偏移量确定 Y 值
        float terrainY = Terrain.activeTerrain.SampleHeight(flatNextPos) + Terrain.activeTerrain.transform.position.y;
        flatNextPos.y = terrainY + heightOffset;

        // 5. 更新位置
        targetTransform.position = flatNextPos;

        // 6. 执行旋转（绕 Y 轴，单位：角度）
        float deltaYaw = currentAngular * angularSpeedMultiplier * Mathf.Rad2Deg * Time.deltaTime;
        float currentYaw = targetTransform.eulerAngles.y;
        float newYaw = currentYaw + deltaYaw;
        targetTransform.rotation = Quaternion.Euler(euler.x, newYaw, euler.z);  // ✅ 强制仅保持绕 Y 轴的旋转

    }

    // 当 ROS 发布 /cmd_vel 消息时调用
    void CmdVelCallback(TwistMsg msg)
    {
        // 提取线速度（只用 X）和角速度（只用 Z）
        targetLinear = (float)msg.linear.x;
        targetAngular = -(float)msg.angular.z;

        // 加限位
        targetLinear = Mathf.Clamp(targetLinear, -0.1f, 0.1f);        // 线速度限制 ±0.1
        targetAngular = Mathf.Clamp(targetAngular, -0.06f, 0.06f);    // 角速度限制 ±0.05 弧度/秒
    }
}


