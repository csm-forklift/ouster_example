#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <string>
#include <math.h>
#include <limits.h>
typedef pcl::PointXYZ PointT;

class FilterLidar {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_pub;
        tf::TransformListener tf_listener;
        std::string lidar_frame_name;
        std::string back_point_cloud_name;


        Eigen::Vector3f translation;
        Eigen::Quaternionf rotation;
        Eigen::Affine3f affine_transform;
    public:
        FilterLidar() : nh_("~"),  translation(0,0,0), rotation(1,0,0,0) {
            back_point_cloud_name = "/os1_node2/points";
            pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("/back_lidar_points",1);
            pcl_sub = nh_.subscribe(back_point_cloud_name.c_str(),1, &FilterLidar::PCLCallback,this);

        }
        void PCLCallback(const sensor_msgs::PointCloud2 &msg) {
            pcl::PointCloud<PointT>::Ptr scene_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr rot1_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr rot2_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr rot3_cloud(new pcl::PointCloud<PointT>);
            pcl::PCLPointCloud2 pcl_pc2a;
            pcl_conversions::toPCL(msg, pcl_pc2a);
            pcl::fromPCLPointCloud2(pcl_pc2a, *scene_cloud);

            float theta_min = 5.0 * (M_PI/180.0); // convert degrees to radians
            float theta_max = 50.0 * (M_PI/180.0); // convert degrees to radians
            float phi_min = (M_PI/2) - theta_min;
            float phi_max = (M_PI/2) - theta_max;

            translation.x() = 0.0;
            translation.y() = 0.0;
            translation.z() = 0.0;

            rotation = Eigen::AngleAxisf(-1*phi_min, Eigen::Vector3f::UnitZ());

            //transformPointCloud(scene_cloud, temp_cloud, msg.header.frame_id, "os1");

            pcl::transformPointCloud(*scene_cloud, *rot1_cloud, translation, rotation);


            pcl::PassThrough<PointT> pass; // passthrough filter

            pass.setInputCloud(scene_cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(0.0, 120.0);
            pass.filter(*rot1_cloud);

            rotation = Eigen::AngleAxisf((phi_min+phi_max), Eigen::Vector3f::UnitZ());

            pcl::transformPointCloud(*rot1_cloud, *rot2_cloud, translation, rotation);
            pass.setInputCloud(rot1_cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(0.0, 120.0);
            pass.filter(*rot2_cloud);

            rotation = Eigen::AngleAxisf(-1*phi_max, Eigen::Vector3f::UnitZ());

            pcl::transformPointCloud(*rot2_cloud, *rot3_cloud, translation, rotation);
            pass.setInputCloud(rot2_cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(-0.05, 0.15);
            pass.filter(*rot3_cloud);

            sensor_msgs::PointCloud2 pc_msg;
            pcl::PCLPointCloud2 pcl_pc2;
            pcl::toPCLPointCloud2(*rot3_cloud, pcl_pc2);
            pcl_conversions::fromPCL(pcl_pc2, pc_msg);
            pcl_pub.publish(pc_msg);
            //std::cout<<"CLEAN: "<<pc_msg.data.size() << '\n';
            //std::cout<<"Init: "<<msg.data.size() << '\n';
            std::cout << "Initial Point count: " <<  msg.data.size() << '\n';

            std::cout << "Final Point count: " <<  pc_msg.data.size() << '\n';


        }

};

int main(int argc, char** argv){
    ROS_INFO("Initializing the lidar_filter_node");
    ros::init(argc,argv, "lidar_filter_node");

    FilterLidar FL;
    ros::spin();
    return 0;
}
