
#include<iostream>
#include<fstream>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void PublishPoseFromTcw(const cv::Mat& Tcw,
                        const ros::Time& stamp,
                        ros::Publisher& pose_pub,
                        ros::Publisher& odom_pub,
                        tf2_ros::TransformBroadcaster& tf_broadcaster);

    void PublishMapPoints(ORB_SLAM3::System* pSLAM);

    void PublishInitialTF(tf2_ros::TransformBroadcaster& tf_broadcaster);

    ORB_SLAM3::System* mpSLAM;

    ros::Publisher pose_pub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster* tf_broadcaster;

    ros::Publisher path_pub;
    ros::Publisher map_pub;

    std::set<ORB_SLAM3::MapPoint*> accumulated_map_points;
    cv::Mat T_odom0;      // 初始 SLAM 位姿
    bool init_odom = false;  // 是否已初始化


    //static int frame_id;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // 创建 ORB-SLAM3 系统对象，使用 RGBD 模式，启用可视化界面
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);
    //创建 ROS 节点句柄
    ros::NodeHandle nh;
    // 发布建图点云话题（/map_points）
    igb.map_pub  = nh.advertise<sensor_msgs::PointCloud2>("/map_points", 1);
    // 发布相机位姿（/orb_slam3/pose）
    igb.pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam3/pose", 10);
    
    igb.odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    // 初始化 TF 广播器，用于发布 odom → base_link 坐标变换
    igb.tf_broadcaster = new tf2_ros::TransformBroadcaster();

    
    // 创建 ROS 图像订阅者，订阅 RGB 图像（Unity 发布  or Saliency）
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/image_raw", 100);
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/saliency", 100);

    // 创建 ROS 图像订阅者，订阅深度图像
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    // 使用同步策略构造同步器，将两个话题同步处理
    message_filters::Synchronizer<sync_pol> sync(sync_pol(30), rgb_sub,depth_sub);
    // 同步回调函数，即当两帧图像到达时调用 ImageGrabber::GrabRGBD
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    SLAM.Shutdown();

    //  camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // std::vector<ORB_SLAM3::MapPoint*> trackedMPs = SLAM.GetTrackedMapPoints();
    // SaveTrackedMapPoints(trackedMPs, "/home/robot/Desktop/SLAM_ws/src/ORB_SLAM3/TrackedMapPoints.txt");

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
static bool tf_init_sent = false;

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    if (!tf_init_sent)
    {
        //PublishInitialTF(*tf_broadcaster);  //  传入 broadcaster 引用
        tf_init_sent = true;
    }
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Sophus::SE3f Tcw  = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    Eigen::Matrix4f eigTcw = Tcw.matrix();
    cv::Mat Tcw_cv(4, 4, CV_32F);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            Tcw_cv.at<float>(i, j) = eigTcw(i, j);

    int state = mpSLAM->GetTrackingState();
    //print
    ROS_INFO("Image size: %d x %d", cv_ptrRGB->image.cols, cv_ptrRGB->image.rows);
    ROS_INFO("Tracking state: %d", state);
    ROS_INFO("Received timestamp: %.9f", cv_ptrRGB->header.stamp.toSec());



    PublishPoseFromTcw(Tcw_cv,
                    ros::Time::now(),//msgRGB->header.stamp,
                    pose_pub,
                    odom_pub,
                    *tf_broadcaster);

    PublishMapPoints(mpSLAM);
    mpSLAM->SaveMapPointsPLY("map_points.ply");

}

void ImageGrabber::PublishInitialTF(tf2_ros::TransformBroadcaster& tf_broadcaster)
{
    geometry_msgs::TransformStamped tf_init;

    tf_init.header.stamp = ros::Time::now();  // 当前时间
    tf_init.header.frame_id = "map";
    tf_init.child_frame_id = "odom";

    // 初始位置为 (0, 0, 0)
    tf_init.transform.translation.x = 0.0;
    tf_init.transform.translation.y = 0.0;
    tf_init.transform.translation.z = 0.0;

    // 单位四元数（无旋转）
    tf_init.transform.rotation.x = 0.0;
    tf_init.transform.rotation.y = 0.0;
    tf_init.transform.rotation.z = 0.0;
    tf_init.transform.rotation.w = 1.0;

    tf_broadcaster.sendTransform(tf_init);

    // ROS_WARN("[InitTF] ✅ 发布初始 map → base_link TF");
}


// void ImageGrabber::PublishPoseFromTcw(const cv::Mat& Tcw,
//                         const ros::Time& stamp,
//                         ros::Publisher& pose_pub,
//                         ros::Publisher& odom_pub,
//                         tf2_ros::TransformBroadcaster& tf_broadcaster)
// {
//     // 计算 Twc = inverse(Tcw)
//     cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
//     cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

//     cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);
//     Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
//     twc.copyTo(Twc.rowRange(0, 3).col(3));

//     // 初始化 T_odom0（只保存第一次的 Twc）
//     if (!init_odom) {
//         T_odom0 = Twc.clone();
//         init_odom = true;
//     }

//     // 计算 Tmap_odom = Twc * inv(T_odom0)
//     cv::Mat Tmo = Twc * T_odom0.inv();
//     cv::Mat Rmo = Tmo.rowRange(0, 3).colRange(0, 3);
//     cv::Mat tmo = Tmo.rowRange(0, 3).col(3);

//     // 转 ROS 四元数
//     Eigen::Matrix3f eigR;
//     for (int i = 0; i < 3; ++i)
//         for (int j = 0; j < 3; ++j)
//             eigR(i, j) = Rmo.at<float>(i, j);
//     Eigen::Quaternionf q(eigR);
//     q.normalize();

//     // 发布 map → odom 的 TF（用了 Unity → ROS 坐标变换）
//     geometry_msgs::TransformStamped tf_msg;
//     tf_msg.header.stamp = stamp;
//     tf_msg.header.frame_id = "map";
//     tf_msg.child_frame_id = "odom";

//     tf_msg.transform.translation.x = tmo.at<float>(2);      // Unity Z → ROS X
//     tf_msg.transform.translation.y = -tmo.at<float>(0);     // Unity X → ROS -Y
//     tf_msg.transform.translation.z = -tmo.at<float>(1);     // Unity Y → ROS Z

//     tf_msg.transform.rotation.x = q.x();
//     tf_msg.transform.rotation.y = q.y();
//     tf_msg.transform.rotation.z = q.z();
//     tf_msg.transform.rotation.w = q.w();

//     tf_broadcaster.sendTransform(tf_msg);

//     // 也可以发布当前 camera pose（map → base_link）
//     geometry_msgs::PoseStamped pose_msg;
//     pose_msg.header.stamp = stamp;
//     pose_msg.header.frame_id = "map";
//     pose_msg.pose.position.x = tf_msg.transform.translation.x;
//     pose_msg.pose.position.y = tf_msg.transform.translation.y;
//     pose_msg.pose.position.z = tf_msg.transform.translation.z;
//     pose_msg.pose.orientation = tf_msg.transform.rotation;
//     pose_pub.publish(pose_msg);
// }

// igb.pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam3/pose", 10);
// igb.odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
// igb.tf_broadcaster = new tf2_ros::TransformBroadcaster();

//发布相对位置坐标 由tcw转换
void ImageGrabber::PublishPoseFromTcw(const cv::Mat& Tcw,
                        const ros::Time& stamp,
                        ros::Publisher& pose_pub,
                        ros::Publisher& odom_pub,
                        tf2_ros::TransformBroadcaster& tf_broadcaster)
{
    // 计算 Twc
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
      
    // 转换为四元数
    Eigen::Matrix3f eigR;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            eigR(i, j) = Rwc.at<float>(i, j);
    Eigen::Quaternionf q(eigR);
    q.normalize();

    // PoseStamped
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    // pose_msg.header.frame_id = "map";
    pose_msg.header.frame_id = "map";
    // pose_msg.pose.position.x = twc.at<float>(0);
    // pose_msg.pose.position.y = twc.at<float>(1);
    // pose_msg.pose.position.z = twc.at<float>(2);
    // // ROS坐标 = Unity Twc坐标重新映射
    pose_msg.pose.position.x = twc.at<float>(2);     // Unity Z → ROS X
    pose_msg.pose.position.y = -twc.at<float>(0);    // Unity X → ROS -Y
    pose_msg.pose.position.z = -twc.at<float>(1);     // -Unity Y → ROS Z

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub.publish(pose_msg);

    // // Odom
    // nav_msgs::Odometry odom;
    // odom.header = pose_msg.header;
    // odom.child_frame_id = "base_link";
    // odom.pose.pose = pose_msg.pose;
    // odom_pub.publish(odom);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = pose_msg.pose.position.x;
    tf_msg.transform.translation.y = pose_msg.pose.position.y;
    tf_msg.transform.translation.z = pose_msg.pose.position.z;
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster.sendTransform(tf_msg);
}

void ImageGrabber::PublishMapPoints(ORB_SLAM3::System* pSLAM)
{
    // 每帧叠加当前可见 MapPoints
    std::vector<ORB_SLAM3::MapPoint*> trackedMPs = pSLAM->GetTrackedMapPoints();
    for (auto mp : trackedMPs)
    {
        if (mp && !mp->isBad())
            accumulated_map_points.insert(mp);  // 自动去重
    }

    // 构建 PCL 点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (auto mp : accumulated_map_points)
    {
        Eigen::Vector3f pos = mp->GetWorldPos();
        pcl::PointXYZRGB p;
        // p.x = pos.x();
        // p.y = pos.y();
        // p.z = pos.z();
        p.x = pos.z();
        p.y = -pos.x();
        p.z = -pos.y();        
        p.r = 255; 
        p.g = 255; 
        p.b = 255;
        cloud.points.push_back(p);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;

    // 发布 RViz
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    map_pub.publish(output);

    // 保存到 PLY（可选）
    pcl::io::savePLYFileBinary("/home/robot/Desktop/map_points_accumulated.ply", cloud);
}