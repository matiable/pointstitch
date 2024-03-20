#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <functional>
#include <map>
#include <mutex>
// PCL specific includes 
#include <sensor_msgs/PointCloud2.h>
//引入多话题共同回调函数
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//引入PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
//滤波的头文件
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include "geometry_msgs/PoseStamped.h"
#include "my_pcl_tutorial/AprilTagDetection.h"
#include "my_pcl_tutorial/AprilTagDetectionArray.h"

using namespace std;
ros::Publisher pointcloud_pub;
//vector<Eigen::Matrix4f> T_t(5);
std::vector<Eigen::Matrix4f> T(5, Eigen::Matrix4f::Identity());
std::mutex t_mutex; // 保护T的互斥锁



//写一个处理函数，假设我们能获取到世界标签到IMU(body),输入一组标定得出的RT，将其转化为相机对于机器人的坐标系
Eigen::Isometry3d getPoseInBody(const Eigen::Isometry3d& camera_RT, const Eigen::Matrix4f& T_tag2body, int i)
{
    Eigen::Isometry3d B_2_T_iso = Eigen::Isometry3d(T_tag2body.cast<double>());
    // 计算逆变换
    Eigen::Isometry3d T_2_B = B_2_T_iso.inverse();
    Eigen::Isometry3d C_2_B = T_2_B * camera_RT;
    T[i-1] = C_2_B.matrix().cast<float>();
    // 返回相机在body坐标系下的位姿
    return C_2_B;
}

//从四元数获取RT，传入指针，并返回Eigen::Isometry3d格式的结果。四元数获取格式为msg中定义的新话题
Eigen::Isometry3d getFromAprilTagDetectionArray(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array, int i)
{
    //AprilTagDetectionArray tag_detection_array;
    Eigen::Isometry3d Rt_ios = Eigen::Isometry3d::Identity();
    if (tag_detection_array->detections.empty()) {
        
        return Rt_ios; // 如果没有检测到，则返回一个单位矩阵
    }
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose = tag_detection_array->detections[0].pose.pose.pose;
    Eigen::Vector3d position(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    // 将四元数信息转换为Eigen::Quaterniond
    Eigen::Quaterniond q(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);
    Eigen::Quaterniond q_inverse = q.inverse();
    Eigen::Vector3d rotatedPosition = -(q_inverse * position);
    // 创建一个Eigen::Isometry3d对象
    // 设置位置和方向
    Rt_ios.translate(rotatedPosition);
    Rt_ios.rotate(q_inverse);
    std::lock_guard<std::mutex> guard(t_mutex);
    T[i-1] = Rt_ios.matrix().cast<float>();
    return Rt_ios;
}

void tagDetection_1_Callback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array) {
        // 处理消息
        //订阅话题得到Array之后，将其转化为Matrix4d的矩阵T
	Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, 1);
    	//如果通过相机IMU标定得到了body与世界的RT ，则使用以下代码
		/*
		Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, nullptr);
		Eigen::Isometry3d pose_in_body = getRotationInBody(pose, const Eigen::Matrix4f& T_tag2body, &T[i])
		*/
    }

void tagDetection_2_Callback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array) {
        // 处理消息
        //订阅话题得到Array之后，将其转化为Matrix4d的矩阵T
	Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, 2);
    	//如果通过相机IMU标定得到了body与世界的RT ，则使用以下代码
		/*
		Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, nullptr);
		Eigen::Isometry3d pose_in_body = getRotationInBody(pose, const Eigen::Matrix4f& T_tag2body, &T[i])
		*/
    }
    
void tagDetection_3_Callback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array) {
        // 处理消息
        //订阅话题得到Array之后，将其转化为Matrix4d的矩阵T
	Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, 3);
    	//如果通过相机IMU标定得到了body与世界的RT ，则使用以下代码
		/*
		Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, nullptr);
		Eigen::Isometry3d pose_in_body = getRotationInBody(pose, const Eigen::Matrix4f& T_tag2body, &T[i])	
		*/
    }
    
void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg1, const sensor_msgs::PointCloud2ConstPtr &cloud_msg2, const sensor_msgs::PointCloud2ConstPtr &cloud_msg3)
{
	 //1.订阅msgs::PointCloud2点云信息，转化为pcl：：PointXYZRGBL点云格式
	vector<pcl::PointCloud<pcl::PointXYZRGB>> pc(9);//[0][1][2]init pclpointcloud [3][4][5]ground pointcloud, [6][7][8]translation of init pointcloud 
	pcl::PointCloud<pcl::PointXYZRGB> cloud_global;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pc_T(pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud_msg1, pc[0]);
	pcl::fromROSMsg(*cloud_msg2, pc[1]);
	pcl::fromROSMsg(*cloud_msg3, pc[2]);
	//step:translation and point stitching
	std::lock_guard<std::mutex> guard(t_mutex);
	for(int i = 0; i < 3; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_after(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr pc_afterR(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(pc[i], *pc_after, T[i]);
		//pcl::transformPointCloud(*pc_after,*pc_afterR ,T_R);
		pc[i+5] = *pc_after;	
	}
	cloud_global = pc[7]+pc[5];
	pointcloud_pub.publish(cloud_global);
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "point_stitch_Apriltag"); // 声明节点的名称
	ros::NodeHandle nh;

        //创建订阅器订阅Apriltag发出的话题，使用此类，使得每五秒跟新一次姿态值。
	ros::Subscriber sub1 = nh.subscribe("/camera_1/tag_detections", 1000, tagDetection_1_Callback);
	ros::Subscriber sub2 = nh.subscribe("/camera_2/tag_detections", 1000, tagDetection_2_Callback);
	ros::Subscriber sub3 = nh.subscribe("/camera_3/tag_detections", 1000, tagDetection_3_Callback);
	//step2: 使用同步工具message_filters，多路topic同时触发回调
	message_filters::Subscriber<sensor_msgs::PointCloud2> image1_sub(nh, "/nebula_1/mtof_points2", 10);
	message_filters::Subscriber<sensor_msgs::PointCloud2> image2_sub(nh, "/nebula_2/mtof_points2", 10);
	message_filters::Subscriber<sensor_msgs::PointCloud2> image3_sub(nh, "/nebula_3/mtof_points2", 10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub, image3_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));
	// Create a ROS publisher for the output point cloud
	
	//step3: 创建ROS的发布节点
	pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 10);
	// Spin
	// 回调
	ros::spin();
}
