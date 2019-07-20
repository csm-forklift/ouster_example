#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/Vector3Stamped.h"

typedef pcl::PointXYZ PointT;

class Extract
{
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher filtered_pub;
	//tf::TransformListener tf_listener;
	float filter_z_low;  // m
	float filter_z_high;   //m
	double resolution;
	pcl::PointCloud<PointT>::Ptr ouster_points;
	pcl::PointCloud<PointT>::Ptr scene_cloud;
    std::string lidar_name;

     //===== Range and Resolution Data
    PointT min_pt; // minimum point in pointcloud
    PointT max_pt; // maximum point in pointcloud
    float x_max;
    float x_min;
    float y_max;
    float y_min;
    float z_max;
    float z_min;
    

public:
    Extract() :
    nh("~"),
    ouster_points(new pcl::PointCloud<PointT>),
    scene_cloud(new pcl::PointCloud<PointT>)
    {
          
	 // nh.param<std::string>("lidar",lidar_name,"lidar");
	  //lidar_frame = "os1_link";
	  //nh_.param<std::string>("target_frame", target_frame, lidar_frame); 
     
     std::string point_topic = "/os1_node1/points"; // + lidar_name + "/points";
     points_sub = nh.subscribe(point_topic.c_str(),1, &Extract::pcCallback, this);
     filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_points",1);

    }

    void pcCallback(const sensor_msgs::PointCloud2 &msg)
    {     


	    filter_z_low = -0.150; // m
        filter_z_high = 0.150; // m
         //===== Convert PointCloud and Transform Data =====//
         // Convert ROS PointCloud2 message into PCL pointcloud
	    rosMsgToPCL(msg, ouster_points);

         //===== Preprocessing (filter, segment, etc.) =====//
         // Try to remove the ground layer (find the minimum z level and remove a few centimeters up)
        pcl::PassThrough<PointT> pass; // passthrough filter
        pass.setInputCloud(ouster_points);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_z_low, filter_z_high);
        pass.filter(*scene_cloud);

        sensor_msgs::PointCloud2 pc_msg;
        pclToROSMsg(scene_cloud, pc_msg);
        filtered_pub.publish(pc_msg);
    }

    void rosMsgToPCL(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>::Ptr cloud)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    }

    void pclToROSMsg(pcl::PointCloud<PointT>::Ptr cloud, sensor_msgs::PointCloud2& msg) {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*cloud, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, msg);
    }



};

int main(int argc, char** argv)
{
      
     ros::init(argc, argv, "3Dpointcloud_extract");
    Extract detector;
    ros::spin();

    return 0;
}