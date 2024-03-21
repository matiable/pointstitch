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
#include <fstream>
using namespace std;
ros::Publisher pointcloud_pub;
//vector<Eigen::Matrix4f> T_t(5);
std::vector<Eigen::Matrix4f> T(5, Eigen::Matrix4f::Identity());
std::mutex t_mutex; // 保护T的互斥锁
vector<pcl::PointCloud<pcl::PointXYZ>> pc(9);//[0][1][2]init pclpointcloud [3][4][5]ground pointcloud, [6][7][8]translation of init pointcloud 
std::vector<Eigen::Vector3d> V_normal;
ros::Publisher filtered_cloud_publisher;
bool cali_success = false;
double next_max_Angle;
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



//Point cloud quality assessment
pcl::ModelCoefficients::Ptr extractPlaneFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud){
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    // 设置分割参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.02);

    // 执行分割
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    // 创建提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);  // 设置为true则提取非平面上的点

    // 提取平面上的点云
    extract.filter(*output_cloud);

    // 返回平面的法向量
    return coefficients;
}

Eigen::Vector3d getNormalVector(const pcl::ModelCoefficients::Ptr &coefficients){
       Eigen::Vector3d plane_coefficients;
       double A = coefficients->values[0];
       double B = coefficients->values[1];
       double C = coefficients->values[2];
       if (C>0)
       	plane_coefficients << A, B, C;
       else 
       	plane_coefficients << -A, -B, -C;
       plane_coefficients.normalize();
       return plane_coefficients;
}

double calculateAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    return std::acos(v1.normalized().dot(v2.normalized()));
}

double maxAngleBetweenVectors(const std::vector<Eigen::Vector3d>& vectors) {
    double maxAngle = 0.0;
    for (size_t i = 0; i < vectors.size(); ++i) {
        for (size_t j = i + 1; j < vectors.size(); ++j) {
            double angle = calculateAngle(vectors[i], vectors[j]);
            if (angle > maxAngle) {
                maxAngle = angle;
            }
        }
    }
    return maxAngle;
}

void tagDetection_1_Callback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array) {
        // 处理消息
        //订阅话题得到Array之后，将其转化为Matrix4d的矩阵T
	Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, 1);
    }

void tagDetection_2_Callback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array) {
        // 处理消息
        //订阅话题得到Array之后，将其转化为Matrix4d的矩阵T
	Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, 2);
    }
    
void tagDetection_3_Callback(const my_pcl_tutorial::AprilTagDetectionArray::ConstPtr& tag_detection_array) {
        // 处理消息
        //订阅话题得到Array之后，将其转化为Matrix4d的矩阵T
	Eigen::Isometry3d pose = getFromAprilTagDetectionArray(tag_detection_array, 3);
    }
    
void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg1, const sensor_msgs::PointCloud2ConstPtr &cloud_msg2, const sensor_msgs::PointCloud2ConstPtr &cloud_msg3)
{
	//1.订阅msgs::PointCloud2点云信息，转化为pcl：：PointXYZ点云格式
	std::vector<Eigen::Vector3d> vectorOfPlane;
	//vector<pcl::PointCloud<pcl::PointXYZ>> pc(9);//[0][1][2]init pclpointcloud [3][4][5]ground pointcloud, [6][7][8]translation of init pointcloud 
	pcl::fromROSMsg(*cloud_msg1, pc[0]);
	pcl::fromROSMsg(*cloud_msg2, pc[1]);
	pcl::fromROSMsg(*cloud_msg3, pc[2]);	
	std::lock_guard<std::mutex> guard(t_mutex);
	for(int i = 0; i < 3; i++)
	{
		if(true){
			pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(pc[i], *input_cloud, T[i]);	
			pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ModelCoefficients::Ptr plane_coefficients = extractPlaneFromPointCloud(input_cloud, output_cloud);
			Eigen::Vector3d v = getNormalVector(plane_coefficients);
			vectorOfPlane.push_back(v);
			pc[i+3] = *output_cloud;
		}
	}
	double this_max_Angle = maxAngleBetweenVectors(vectorOfPlane);
	if (cali_success==false)//cali_success == false && maxangle <0.01
	{
		ROS_INFO("The biggest angle of this calibration is: %f \n",this_max_Angle);
		ROS_INFO("calibration success!\n");
		cali_success = true;
	}
	if (cali_success == true)
	{
		ROS_INFO("start write\n");
		ros::NodeHandle nh_setT;
		for (size_t i = 0; i < 3; ++i) {
        		std::stringstream param_name;
        		param_name << "/T_" << i;

        		// 将 Eigen::Matrix 转换为 std::vector<float>
        		std::vector<float> matrix_data(T[i].data(), T[i].data() + T[i].size());

        		// 写入到 Parameter Server
        		nh_setT.setParam(param_name.str(), matrix_data);
   	 	}
		std::string cmd = "rosparam dump /home/firefly/mingxi_ma/pointstich_ws/src/my_pcl_tutorial/config/example.yaml /";
    		system(cmd.c_str());
    		
    		ROS_INFO("Parameters written to file.");
	}
	//judge the number of pointcloud enough 
	
	//compute the angle of two vector
	
	std::size_t num_points = pc[4].size();
	ROS_INFO("The number of point in plane :%zu\n", num_points);
	filtered_cloud_publisher.publish(pc[4]);
}


int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "calibration_quality"); // 声明节点的名称
	ros::NodeHandle nh;
	
	filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1);
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
	ros::spin();
}
