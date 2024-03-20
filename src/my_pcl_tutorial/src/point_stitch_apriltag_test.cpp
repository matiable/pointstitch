#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <functional>
#include <map>
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

#include "geometry_msgs/PoseStamped.h"
#include "my_pcl_tutorial/AprilTagDetection.h"
#include "my_pcl_tutorial/AprilTagDetectionArray.h"

Eigen::Matrix4f T;

void tagDetectionCallback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& msg)
{
    Eigen::Isometry3d Rt_ios = Eigen::Isometry3d::Identity();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose = msg->detections[0].pose.pose.pose;
    Eigen::Vector3d position(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    // 将四元数信息转换为Eigen::Quaterniond
    Eigen::Quaterniond orientation(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);
    // 创建一个Eigen::Isometry3d对象
    // 设置位置和方向
    Rt_ios.translate(position);
    Rt_ios.rotate(orientation);
    T = Rt_ios.matrix().cast<float>();
    std::stringstream t;
    t << T;
    ROS_INFO("Eigen Matrix4f T[1]:\n%s", t.str().c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detection_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera_3/tag_detections", 1000, tagDetectionCallback);

    // 进入ROS事件循环，直到节点被关闭。
    ros::spin();

    return 0;
}
