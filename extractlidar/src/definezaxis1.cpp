#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include "map_builder.lua"
#include "trajectory_builder.lua"
#include <iostream>
#include <ros/ros.h>
#include <string>


typedef pcl::PointXYZ PointT;

class Extract
{
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher extract_points_pub;
	tf::TransformListener tf_listener;
	float filter_z_low;  // m
	float filter_z_high;   //m
	double resolution;
	pcl::PointCLoud<PointT>::Ptr scene_cloud_optical_frame;
	pcl::PointCloud<PointT>::Ptr scene_cloud_unfiltered;
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
    scene_cloud_optical_frame(new pcl::PointCloud<PointT>),
    scene_cloud_unfiltered(new pcl::PointCloud<PointT>),
    scene_cloud(new pcl::PointCloud<PointT>)
    {
          
	  nh.param<std::string>("lidar",lidar_name,"lidar");
	  lidar_frame = lidar_name + "_link";
	  nh_.param<std::string>("target_frame", target_frame, lidar_frame); 
     
     std::string point_topic = "/" + lidar_name + "//points";
     ROS_INFO("Transforming cloud to '%s' frame", camera_frame.c_str());
       
     points_sub = nh.subscribe(point_topic.c_str(), &Extract::pcCallback, this);
     filtered_pub = nh.advertise<sensor::msgs::POintCloud2>("filtered_points",1);

    }

    void pcCallback(const sensor_msgs::POintCloud2 &msg)
    {     


	    filter_z_low = -0.100; // m
        filter_z_high = 0.500; // m
         //===== Convert PointCloud and Transform Data =====//
         // Convert ROS PointCloud2 message into PCL pointcloud
	   rosMsgToPCL(msg, scene_cloud_optical_frame);
         // Transform pointcloud into camera frame
        transformPointCloud(scene_cloud_optical_frame, scene_cloud_unfiltered, msg.header.frame_id, camera_frame);

         // Get the bounds of the point cloud
        pcl::getMinMax3D(*scene_cloud_unfiltered, min_pt, max_pt);
	        x_max = max_pt.x;
	        x_min = min_pt.x;
	        y_max = max_pt.y;
	        y_min = min_pt.y;
	        z_max = max_pt.z;
	        z_min = min_pt.z;
           // DEBUG: Print bounds
         std::cout << "Bounds [min, max]: \n";
         std::cout << "x: [" << x_min << ", " << x_max << "]\n";
         std::cout << "y: [" << y_min << ", " << y_max << "]\n";
         std::cout << "z: [" << z_min << ", " << z_max << "]\n";
         std::cout << std::endl;
         //===== Preprocessing (filter, segment, etc.) =====//
         // Try to remove the ground layer (find the minimum z level and remove a few centimeters up)
        pcl::PassThrough<PointT> pass; // passthrough filter
        pass.setInputCloud(scene_cloud_unfiltered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_z_low, filter_z_high);
        pass.filter(*scene_cloud);

        sensor_msgs::PointCloud2 pc_msg;
        pclToROSMsg(scene_cloud, pc_msg);
        filtered_pub.publish(pc_msg);
    }
};

int main(int argc, char** argv)
{
      
     ros::init(argc, argv, "3Dpointcloud_extract");
    CylinderDetector detector;
    ros::spin();

    return 0;
}