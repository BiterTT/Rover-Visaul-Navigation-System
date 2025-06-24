using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Globalization;

public class CameraAutoCapture : MonoBehaviour
{
    public Transform startPoint;        // 起点
    public Transform endPoint;          // 终点
    public float moveSpeed = 1.0f;      // 摄像头移动速度
    public float captureInterval = 0.05f; // 每几米拍一张
    public float heightOffset = 1.5f;   // 摄像头高出地形的高度
    private Vector3 direction;  //方法向量
    private float totalDistance;    //总距离
    private float movedDistance = 0f;   //已经移动的距离
    private float lastCaptureDistance = 0f; //上一次拍照时的位置
    //private int imageCount = 0;   //拍照次数
    private StreamWriter poseWriter;    //用于写入 CSV 姿态文件的流
    public bool enable_manual_control = false;

    void Start()
    {
        // 初始化路径方向与距离
        direction = (endPoint.position - startPoint.position).normalized;
        totalDistance = Vector3.Distance(startPoint.position, endPoint.position);

        // 初始化摄像头位置
        transform.position = GetHeightAdjustedPosition(startPoint.position);
        transform.LookAt(endPoint.position);
    }

    void Update()
    {
        if (movedDistance < totalDistance)
        {
            // 摄像头移动
            // Vector3 newPos = transform.position + direction * moveSpeed * Time.deltaTime;
            // movedDistance += Vector3.Distance(transform.position, newPos);

            // transform.position = GetHeightAdjustedPosition(newPos);

            // 计算下一个目标点
            Vector3 newPos = transform.position + direction * moveSpeed * Time.deltaTime;
            movedDistance += Vector3.Distance(transform.position, newPos);

            // 获取当前地形位置
            Vector3 adjustedPos = GetHeightAdjustedPosition(newPos);

            // 添加“上下颠簸”：正弦波扰动 + 噪声
            float bumpAmplitude = 0.05f; // 颠簸强度
            float bumpFrequency = 15.0f;  // 颠簸频率
            float bumpNoise = Mathf.PerlinNoise(Time.time * 10f, 0.0f); // 加入一点随机性
            float bumpOffset = Mathf.Sin(Time.time * bumpFrequency) * bumpAmplitude + bumpNoise * 0.05f;;
            adjustedPos.y += bumpOffset;
            transform.position = adjustedPos;

            // ==== 强化车辆姿态摆动 ====
            float pitchAmplitude = 0.2f; // 前后俯仰
            float rollAmplitude = 0.3f;  // 左右侧倾
            float pitch = Mathf.Sin(Time.time * 10f) * pitchAmplitude + 10;
            float roll = Mathf.Sin(Time.time * 12f + 1f) * rollAmplitude;
            float yaw = Mathf.Sin(Time.time * 8f) * 0.5f;  // 轻微摇头（可调小）

            // 设置摄像机朝向：带轻微左右摆动的前向方向
            // float swayAmplitude = 2f;   // 摇摆角度范围（度）
            // float pitch = 10.0f + Mathf.Sin(Time.time * 1.5f) * swayAmplitude;  // 前后轻摆
            // float yaw = Mathf.Sin(Time.time * 0.7f) * swayAmplitude * 0.5f;     // 左右小摆动

            Quaternion targetRot = Quaternion.LookRotation(direction) *
                                Quaternion.Euler(pitch, yaw, roll);

            transform.rotation = targetRot;

            // 判断是否需要拍照
            if (movedDistance - lastCaptureDistance >= captureInterval)
            {
                //CaptureImage();
                lastCaptureDistance = movedDistance;
            }
        }
        else
        {
            // // 到达终点，关闭文件
            // if (poseWriter != null)
            // {
            //     poseWriter.Close();
            //     //Debug.Log("拍摄完成，保存位置和图像于：" + saveFolder);
            // }
            enable_manual_control = true;
            enabled = false;
        }
    }

    // 获取当前地形高度 + 偏移后的真实位置
    Vector3 GetHeightAdjustedPosition(Vector3 pos)
    {
        float terrainHeight = Terrain.activeTerrain.SampleHeight(pos);
        float terrainBaseY = Terrain.activeTerrain.transform.position.y;
        return new Vector3(pos.x, terrainBaseY + terrainHeight + heightOffset, pos.z);

    }
}