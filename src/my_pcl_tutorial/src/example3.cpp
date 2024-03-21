#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
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

using namespace std;
ros::Publisher pointcloud_pub;
//vector<Eigen::Matrix4f> T_t(5);
vector<Eigen::Matrix4f> T(5);
Eigen::Matrix4f T_R;
vector<Eigen::Affine3f> transform_test(5);
//定义转移矩阵获取函数
//#define A2R(angle) 3.1415926535*(angle/360)
Eigen::Matrix4f getRomateTMat1(float yaw, float pitch, float roll, float x, float y, float z){
    float yaw_r = M_PI*(yaw/180);
    float pitch_r = M_PI*(pitch/180);
    float roll_r = M_PI*(roll/180);
    float cx = cos(yaw_r);
    float sx = sin(yaw_r);
    float cy = cos(pitch_r);
    float sy = sin(pitch_r);
    float cz = cos(roll_r);
    float sz = sin(roll_r);
    Eigen::Matrix4f Rx, Ry, Rz, T, mat, tran;
    tran << 0, 1, 0, 0,
     	    0,  0, 1, 0,
     	    -1, 0, 0, 0,
     	    0, 0, 0, 1; 
     	    	    
    Rx << 1,  0,  0,  0,
          0,  cx, sx,0,
          0,  -sx, cx, 0,
		0,  0,  0,  1;
 
    Ry << cy, 0, sy,  0,
           0, 1, 0,   0,
         -sy, 0, cy,  0,
		   0, 0, 0,   1;
 
    Rz << cz, sz, 0, 0, 
          -sz,  cz, 0, 0,
          0 ,   0, 1, 0,
		  0 ,   0, 0, 1;
 
    T << 0, 0, 0, -x,
		 0, 0, 0, -y,
		 0, 0, 0, -z,
		 0, 0, 0,  0;
    mat = Rz * Ry * Rx + T;
    return mat;
}

Eigen::Matrix4f getRomateTMat2(float yaw, float pitch, float roll, float x, float y, float z){
    float yaw_r = M_PI*(yaw/180);
    float pitch_r = M_PI*(pitch/180);
    float roll_r = M_PI*(roll/180);
    float cx = cos(yaw_r);
    float sx = sin(yaw_r);
    float cy = cos(pitch_r);
    float sy = sin(pitch_r);
    float cz = cos(roll_r);
    float sz = sin(roll_r);
    Eigen::Matrix4f Rx, Ry, Rz, T, mat, tran;
    tran << 0, 1, 0, 0,
     	    0,  0, 1, 0,
     	    -1, 0, 0, 0,
     	    0, 0, 0, 1; 
     	    	    
    Rx << 1,  0,  0,  0,
          0,  cx, sx,0,
          0,  -sx, cx, 0,
		0,  0,  0,  1;
 
    Ry << cy, 0, -sy,  0,
           0, 1, 0,   0,
          sy, 0, cy,  0,
		   0, 0, 0,   1;
 
    Rz << cz, sz, 0, 0, 
          -sz,  cz, 0, 0,
          0 ,   0, 1, 0,
	  0 ,   0, 0, 1;
 
    T << 0, 0, 0, -x,
		 0, 0, 0, -y,
		 0, 0, 0, -z,
		 0, 0, 0,  0;
    mat = Rz * Rx * Ry + T;
    return mat;
}

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg1, const sensor_msgs::PointCloud2ConstPtr &cloud_msg2, const sensor_msgs::PointCloud2ConstPtr &cloud_msg3)
{
	 //1.订阅msgs::PointCloud2点云信息，转化为pcl：：PointXYZRGBL点云格式
	vector<pcl::PointCloud<pcl::PointXYZRGB>> pc(10);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_global;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pc_T(pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud_msg1, pc[0]);
	pcl::fromROSMsg(*cloud_msg2, pc[1]);
	pcl::fromROSMsg(*cloud_msg3, pc[2]);
	for(int i = 0; i < 3; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_after(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_afterR(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(pc[i], *pc_after, T[i]);
		pcl::transformPointCloud(*pc_after,*pc_afterR ,T_R);
		pc[i+5] = *pc_afterR;	
	}
	cloud_global = pc[7]+pc[5]+pc[6];
	//pointcloud_pub.publish(pc[5]);
	pointcloud_pub.publish(cloud_global);
	/*cloud_global = pc[5];
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_global(new pcl::PointCloud<pcl::PointXYZ>);
	pc_global = cloud_global.makeShared();
	cout << "the number of this :" <<  pc_global->size() <<endl;
	*/
	
	/*sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(cloud_global,output);
	output.header = cloud_msg1->header;
	output.is_dense = cloud_msg1->is_dense;
	output.width = cloud_msg1->width;
	output.height = count;
	pointcloud_pub.publish(output); */
	
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "my_pcl_tutorial"); // 声明节点的名称
	ros::NodeHandle nh;
	//给转移矩阵赋值
	vector<float> tran1,tran2,tran3,tran4,tran5;
	nh.getParam("/camera1_ypr_t", tran1);
	nh.getParam("/camera2_ypr_t", tran2);
	nh.getParam("/camera3_ypr_t", tran3);
	nh.getParam("/camera4_ypr_t", tran4);
	nh.getParam("/camera5_ypr_t", tran5);
	T[2] = getRomateTMat2(tran1[0],tran1[1],tran1[2],tran1[3],tran1[4],tran1[5]);
	T[0] = getRomateTMat2(tran2[0],tran2[1],tran2[2],tran2[3],tran2[4],tran2[5]);
	//T[0] = getRomateTMat1(tran3[0],tran3[1],tran3[2],tran3[3],tran3[4],tran3[5]);
	T[1] = getRomateTMat1(tran4[0],tran4[1],tran4[2],tran4[3],tran4[4],tran4[5]);
	//T[4] = getRomateTMat1(tran5[0],tran5[1],tran5[2],tran5[3],tran5[4],tran5[5]);
	/*T[3] = getRomateTMat2(-33,-17,-90, 0.0376, 0.02177, -0.40759);
	T[1] = getRomateTMat2(33,-17,-90, -0.0376, 0.02177, -0.40759);
	T[0] = getRomateTMat1(30,90,0, -0.18669, 0.02998, 0.08386);
	T[2] = getRomateTMat1(30,-180,0,-0.01562, -0.00934, 0.39654);
	T[4] = getRomateTMat1(30,-90, 0,0.18705, 0.02998, 0.11614);*/
	T_R = getRomateTMat1(90,0,90,0,0,0);
	
	
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
