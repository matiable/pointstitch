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
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "my_pcl_tutorial/AprilTagDetection.h"
#include "my_pcl_tutorial/AprilTagDetectionArray.h"
#include "my_pcl_tutorial/qualityparameter.h"

#include <yaml-cpp/yaml.h>
#include <fstream>
std::mutex t_mutex; // 保护v_normal的互斥锁
std::unordered_map<int,std::vector<int>> v_normal;
std::unordered_map<int,Eigen::Matrix4f> T;
std::vector<ros::Subscriber> subscribers;
std::vector<int> refer_angle;
ros::Publisher = error_pub;

void cameraCallback(const my_pcl_tutorial::qualityparameter::ConstPtr& msg, const int camera_index) {
    std::lock_guard<std::mutex> guard(t_mutex);
    if(msg->vector_tag.size()==3 && msg->T.size()==16){
        Eigen::Matrix4f transform_t;
        std::vector<int> v(3);
        v[0] = msg->vector_tag[0];
        v[1] = msg->vector_tag[1];
        v[2] = msg->vector_tag[2];
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                transform_t(i,j) = msg->T[i*4+j];
            }
        }
        if(!v_noramal.count(camera_index)){
            v_normal.insert(std::make_pair(camera_index,v));
            T.insert(std::make_pair(camera_index,transform_t));
        }
        else{
            v_normal[camera_index] = v;
            T[camera_index] = transform_t;
        }
    }
    angleerror(camera_index);
}

void angleerror(const int camera_index){
    if(T.size()==3){
        int num = v_normal.size();
        int error = 0;
        for(int i = 0 ; i < num-1 ; i++){
            int angle = calculateAngle(v_normal[i],v_normal[i+1]);
            error += angle - refer_angle;
        }
        error = error/(num-1);
        if(error > 5.0){
            ROS_WARN("the angle error of this calibration is %d", error);
        }
        else{
            ROS_INFO("calibration success!");
            //将T中的camera_index数值写入参数文件
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "camera_id" << YAML::Value << camera_id;
            out << YAML::Key << "camera_T" << YAML::Value << YAML::Flow;
            out << YAML::BeginSeq;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    out << camera_T(i, j);
                }
            }
            out << YAML::EndSeq;
            out << YAML::EndMap;
            // 将YAML数据写入文件
            std::ofstream fout("parameters.yaml");
            fout << out.c_str();
        }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibration_quality"); // 声明节点的名称
    ros::NodeHandle nh;
    std::int camera_num;
    nh.getParam("camera_num", camera_num);
    nh.getParam("refer_num", refer_angle);
    error_pub = nh.advertise<my_pcl_tutorial::CalibrationQuality>("/calib_error", 1);   
    for (int i = 1; i <= camera_num; ++i){
        std::stringstream ss;
        ss << "camera_" << i << "/qual_para";
        // 使用boost::bind将循环变量i作为参数传递给回调函数
        ros::Subscriber sub = nh.subscribe<my_pcl_tutorial::qualityparameter>(ss.str(), 1000, boost::bind(cameraCallback, _1, i));
        subscribers.push_back(sub);
    }
	ros::spin();
}