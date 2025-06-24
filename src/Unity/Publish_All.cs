using System;
using System.IO;
using System.Globalization;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System.Collections;

using RosMessageTypes.Tf2;
using RosMessageTypes.Geometry;

public class Publish_All : MonoBehaviour
{
    public Camera imageCamera;         // Unity 中绑定的相机
    public int width = 640;            // 图像宽度
    public int height = 480;           // 图像高度

    private ROSConnection ros;

    private RenderTexture renderTextureRGB;     // 用于 RGB 图渲染
    private Texture2D rgbTexture;               // 存放 RGB 像素数据

    private RenderTexture renderTextureDepth;   // 用于深度图渲染
    private Texture2D depthTexture;             // 存放深度图原始 float 数据
    private Material depthMaterial;                   // 用于渲染真实深度图的 Shader 材质


    private Rect rect;      // 图像读取区域

    private string saveFolder = "Captures";     // 保存图像与位姿的本地文件夹
    private string poseFilePath;                // pose.csv 文件路径
    private int imageIndex = 0;                 // 图像编号计数器

    private string tfParent = "odom";
    private string tfChild = "base_link";

    private bool enable_originPos = true;
    private Vector3 odomOriginPos;
    private Quaternion odomOriginRot;
    public static TimeMsg Now()
    {
        DateTime now = DateTime.UtcNow;
        long unixTimeSeconds = ((DateTimeOffset)now).ToUnixTimeSeconds();
        long nanoseconds = now.Ticks % TimeSpan.TicksPerSecond * 100; // 每 tick 是 100ns

        return new TimeMsg
        {
            sec = (uint)unixTimeSeconds,
            nanosec = (uint)nanoseconds
        };
    }


    // Start is called before the first frame update
    IEnumerator  Start()
    {
        // 初始化 ROS 发布
        ros = ROSConnection.GetOrCreateInstance();
        yield return new WaitForSeconds(5.0f);  // 不用 IsConnected，直接等 1 秒

        ros.RegisterPublisher<ImageMsg>("/camera/image_raw");
        ros.RegisterPublisher<ImageMsg>("/camera/depth/image_raw");
        ros.RegisterPublisher<CameraInfoMsg>("/camera/depth/camera_info");
        ros.RegisterPublisher<TFMessageMsg>("/tf");



        // 初始化两个 RenderTexture：一个用于 RGB，一个用于 Depth
        renderTextureRGB = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        //renderTextureDepth = new RenderTexture(width, height, 24, RenderTextureFormat.Depth);
        renderTextureDepth = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);   //ARGBFloat
        renderTextureDepth.Create();

        // 创建纹理缓存
        rgbTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        //depthTexture = new Texture2D(width, height, TextureFormat.RFloat, false);
        depthTexture = new Texture2D(width, height, TextureFormat.RFloat, false);
        rect = new Rect(0, 0, width, height);

        // 加载自定义深度 Shader 并构建材质
        Shader depthShader = Shader.Find("Custom/DepthToFloat");
        if (depthShader == null)
        {
            Debug.LogError("Shader 'Custom/DepthToFloat' not found!");
        }
        else
        {
            depthMaterial = new Material(depthShader);
        }

        // 启用深度采样模式
        imageCamera.depthTextureMode = DepthTextureMode.Depth;

        // 创建保存路径与 pose.csv 文件
        string folderPath = Path.Combine(Application.dataPath, saveFolder);
        Directory.CreateDirectory(folderPath);
        poseFilePath = Path.Combine(folderPath, "pose.csv");
        if (!File.Exists(poseFilePath))
            File.WriteAllText(poseFilePath, "image,x,y,z,roll,pitch,yaw\n");

        // 每 0.1 秒采集一次图像
        InvokeRepeating(nameof(Publish), 1.0f, 0.033f); // 10 Hz
    }
    // 主函数：每帧发布和保存 RGB + Depth + Pose
    void Publish()
    {
        // 构造统一时间戳的消息头
        HeaderMsg header = new HeaderMsg
        {
            stamp = Now(),
            frame_id = "base_link"
        };

        PublishRGB(header);                 // 发布 RGB 图像
        PublishDepth(header);               // 发布 Depth 图像
        PublishCameraInfo(header);          // 发布相机内参
        //PublishTF(header.stamp);

        //SaveImageAndPose(rgbTexture, imageIndex);         // 保存 RGB 图像和位姿
        //SaveDepthGrayImage(depthTexture, imageIndex);     // 保存灰度深度图（仅用于可视化）
        imageIndex++;
    }

    // 发布 RGB 图像（已翻转Y轴）
    void PublishRGB(HeaderMsg header)
    {
        imageCamera.targetTexture = renderTextureRGB;
        RenderTexture.active = renderTextureRGB;
        imageCamera.Render();
        rgbTexture.ReadPixels(rect, 0, 0);
        rgbTexture.Apply();
        RenderTexture.active = null;

        byte[] raw = rgbTexture.GetRawTextureData();
        byte[] flipped = new byte[raw.Length];
        int rowSize = width * 3;
        for (int y = 0; y < height; y++)
            Array.Copy(raw, y * rowSize, flipped, (height - 1 - y) * rowSize, rowSize);

        ImageMsg msg = new ImageMsg
        {
            header = header,
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",               // ORB-SLAM3 配置需设 Camera.RGB: 1
            is_bigendian = 0,
            step = (uint)(width * 3),
            data = flipped
        };

        ros.Publish("/camera/image_raw", msg);
    }

    // 发布深度图（单位米，float32）
    void PublishDepth(HeaderMsg header)
    {
        if (depthMaterial == null)
        {
            Debug.LogError("Depth material not initialized!");
            return;
        }
        else
        {
            //Debug.Log($"✅ 使用的 Shader: {depthMaterial.shader.name}");
        }
        // 使用 Shader 渲染相机深度图到 float32 渲染纹理
        Graphics.Blit(null, renderTextureDepth, depthMaterial);

        //imageCamera.targetTexture = renderTextureDepth;
        RenderTexture.active = renderTextureDepth;
        //imageCamera.Render();
        depthTexture.ReadPixels(rect, 0, 0);
        depthTexture.Apply();
        RenderTexture.active = null;
        
        // 读取深度值（float），并做垂直翻转
        Color[] pixels = depthTexture.GetPixels();
        float[] depthArray = new float[pixels.Length];
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int srcIndex = y * width + x;
                int dstIndex = (height - 1 - y) * width + x;
                depthArray[dstIndex] = pixels[srcIndex].r;
            }
        }
        byte[] depthBytes = new byte[depthArray.Length * 4];
        Buffer.BlockCopy(depthArray, 0, depthBytes, 0, depthBytes.Length);

        float centerDepth = depthArray[(height / 2) * width + (width / 2)];
        // Debug.Log($"📏 中心深度 = {centerDepth:F4} 米");

        ImageMsg depthmsg = new ImageMsg
        {
            header = header,
            height = (uint)height,
            width = (uint)width,
            encoding = "32FC1",              // ORB-SLAM3 要求格式
            is_bigendian = 0,
            step = (uint)(width * 4),
            data = depthBytes
        };

        ros.Publish("/camera/depth/image_raw", depthmsg);
    }

    // 发布 CameraInfo（简化无畸变模型）
    void PublishCameraInfo(HeaderMsg header)
    {
        float fx = 0.5f * width / Mathf.Tan(imageCamera.fieldOfView * Mathf.Deg2Rad / 2);
        float fy = fx;
        float cx = width / 2.0f;
        float cy = height / 2.0f;

        CameraInfoMsg info = new CameraInfoMsg
        {
            header = header,
            height = (uint)height,
            width = (uint)width,
            distortion_model = "plumb_bob",
            d = new double[5],
            k = new double[9],
            r = new double[9] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            p = new double[12]
        };

        info.k[0] = fx; info.k[2] = cx;
        info.k[4] = fy; info.k[5] = cy;
        info.k[8] = 1;

        info.p[0] = fx; info.p[2] = cx;
        info.p[5] = fy; info.p[6] = cy;
        info.p[10] = 1;

        ros.Publish("/camera/depth/camera_info", info);
    }

    // 保存 RGB 图像为 PNG + 相机位姿为 CSV
    void SaveImageAndPose(Texture2D tex, int index)
    {
        string folderPath = Path.Combine(Application.dataPath, saveFolder);
        string imageName = $"image_{index:D4}.png";
        string imagePath = Path.Combine(folderPath, imageName);
        File.WriteAllBytes(imagePath, tex.EncodeToPNG());

        Vector3 pos = imageCamera.transform.position;
        Vector3 rot = imageCamera.transform.eulerAngles;

        string line = string.Format(CultureInfo.InvariantCulture,
            "{0},{1:F4},{2:F4},{3:F4},{4:F2},{5:F2},{6:F2}",
            imageName, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z);

        File.AppendAllText(poseFilePath, line + "\n");
    }

    // 保存深度图为灰度 PNG（用于调试可视化）
    void SaveDepthGrayImage(Texture2D tex, int index)
    {
        float maxDepthMeters = 5.0f;
        Texture2D grayTex = new Texture2D(width, height, TextureFormat.R8, false);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                float z = tex.GetPixel(x, y).r;
                float linear = LinearizeDepth(z, imageCamera.nearClipPlane, imageCamera.farClipPlane);
                float norm = Mathf.Clamp01(linear / maxDepthMeters);
                grayTex.SetPixel(x, height - 1 - y, new Color(norm, norm, norm));
            }
        }

        grayTex.Apply();
        string fileName = $"image_{index:D4}_depth.png";
        string savePath = Path.Combine(Application.dataPath, saveFolder, fileName);
        File.WriteAllBytes(savePath, grayTex.EncodeToPNG());
    }

    // 将 Unity 非线性深度值还原为线性深度（单位米）
    float LinearizeDepth(float zBuffer, float near, float far)
    {
        return (2.0f * near * far) / (far + near - zBuffer * (far - near));
    }

    void PublishTF(TimeMsg time)
    {
        // if (enable_originPos)
        // {
        //     odomOriginPos = imageCamera.transform.position;
        //     odomOriginRot = imageCamera.transform.rotation;
        //     enable_originPos = false;
        // }
        odomOriginPos = new Vector3(61.31f,106.52f,25.65f);
        odomOriginRot = Quaternion.Euler(10.43f, 9.12f, -0.22f);

        Vector3 pos = imageCamera.transform.position;
        Quaternion rot = imageCamera.transform.rotation;

        // 相对 odom 原点的位移 + 旋转
        Vector3 relativePos = pos - odomOriginPos;
        Quaternion relativeRot = Quaternion.Inverse(odomOriginRot) * rot;

        // 坐标变换：Unity (X,Y,Z) → ROS (Z,-X,Y)
        Vector3 rosPos = new Vector3(
            relativePos.z,
            -relativePos.x,
            relativePos.y
        );

        Quaternion rosRot = new Quaternion(
            relativeRot.z,
            -relativeRot.x,
            relativeRot.y,
            relativeRot.w
        );

        // 构造 TF 消息
        TransformStampedMsg tf = new TransformStampedMsg();
        tf.header.stamp = time;
        tf.header.frame_id = tfParent;     // "odom"
        tf.child_frame_id = tfChild;       // "base_link"

        tf.transform.translation.x = rosPos.x;
        tf.transform.translation.y = rosPos.y;
        tf.transform.translation.z = rosPos.z;

        tf.transform.rotation.x = rosRot.x;
        tf.transform.rotation.y = rosRot.y;
        tf.transform.rotation.z = rosRot.z;
        tf.transform.rotation.w = rosRot.w;

        TFMessageMsg tfMsg = new TFMessageMsg(new TransformStampedMsg[] { tf });
        ros.Publish("/tf", tfMsg);
    }
}
