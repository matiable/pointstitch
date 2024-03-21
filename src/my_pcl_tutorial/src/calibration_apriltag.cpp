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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Geometry>
#include "geometry_msgs/PoseStamped.h"
#include "my_pcl_tutorial/AprilTagDetection.h"
#include "my_pcl_tutorial/AprilTagDetectionArray.h"
#include "my_pcl_tutorial/qualityparameter.h"
#include <fstream>
using namespace std;
ros::Publisher qual_publisher;
Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
bool detect_seccess = false;

std::mutex t_mutex; // 保护T的互斥锁

Eigen::Isometry3d getFromAprilTagDetectionArray(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array, int i)
{
    //AprilTagDetectionArray tag_detection_array;
    Eigen::Isometry3d Rt_ios = Eigen::Isometry3d::Identity();
    if (tag_detection_array->detections.empty()) {
        ROS_WARN("%s have no detect data", ros::this_node::getName().c_str());
        detect_seccess = false;
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
    detect_seccess = true;
    return Rt_ios;
}

void tagDetection_Callback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array) {
	Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, 1);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud) {
    my_pcl_tutorial::qualityparameter msg;
	pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
     //点云语义分割部分两种做法，第一种，在进行点云的平面提取之前，率除掉某一维度上超过某值的点，从而保留大部分的tag上的点云
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_pointcloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(pcl_pointcloud);
    if(detect_seccess == true){
        //将点云转到世界坐标系下（apriltag坐标系）
        pcl::fromROSMsg(*pointcloud, pcl_pointcloud);
        pcl::transformPointCloud(pcl_pointcloud, pcl_pointcloud, T);
        //分割，获取法向量（后续还需要提取能检验T的指标）
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setInputCloud(pcl_pointcloud.makeShared()); // 使用 makeShared() 将实体点云转换为智能指针
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setDistanceThreshold(0.01); // 设置一个适当的阈值
        seg.segment(*inliers, *coefficients);
        for(int i=0;i<4;i++){
            for(int j=0;i<4;j++){
                msg.T[i*4+j] = T(i,j);
            }
        }
        msg.vector_tag.push_back(coefficients->values[0]);
        msg.vector_tag.push_back(coefficients->values[1]);
        msg.vector_tag.push_back(coefficients->values[2]);
        qual_publisher.publish(msg);
    }
    //第二种，就是分割制定法向量的点云
}

void caliberror_Callback(const my_pcl_tutorial/CalibrationQuality::ConstPtr& caliberror){
    if(caliberror->success_flag){

    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibration_apriltag"); // 声明节点的名称
	ros::NodeHandle nh;
	qual_publisher = nh.advertise<my_pcl_tutorial::qualityparameter>("/qual_para", 1);
    std::string camera_num;
    nh.getParam("camera_num", camera_num);
    std::string pointcloud_topic = "/camera_" + camera_num + "mtof_points2";
    std::string detect_topic = "/camera_" + camera_num + "/tag_detections";
	ros::Subscriber detect_sub = nh.subscribe(detect_topic, 1000, tagDetection_Callback);
    ros::Subscriber error_sub = nh.subscribe("/calib_error", 1000, caliberror_Callback);
    ros::Subscriber pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 10, pointCloudCallback);
	ros::spin();
}