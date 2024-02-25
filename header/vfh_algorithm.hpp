#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;


class VFH{
        bool                                is_visualization;
        int                                 width, height, range_max, distance_to_obstacle, free_distance;
        double                              fov, map_detail;
        ros::NodeHandle                     n;
        ros::Subscriber                     current_position_sub, depth_camera_sub, desired_position_sub;
        ros::Publisher                      desired_position_pub, marker_pub;
        geometry_msgs::PoseStamped          current_position, desired_position;
        visualization_msgs::Marker          marker_msgs;
        geometry_msgs::Quaternion           q;
        pcl::PointCloud<pcl::PointXYZ>      point_cloud;
        vector<vector<int>>                 map_countable;
        vector<vector<double>>              map_distance;
        vector<vector<bool>>                histogram;
        geometry_msgs::Point                visualization_msgs;
        geometry_msgs::PoseStamped          position_msgs; 
    public:
        VFH(ros::NodeHandle n);
        void                        update();
        void                        choose_direction(double *direction);
        bool                        choose_free_direction(double *direction);
        void                        send_visualization_message(double* point);
        void                        send_position_message(double* direction);
        void                        desired_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void                        current_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void                        create_histogram(pcl::PointCloud<pcl::PointXYZ> msg);
        void                        rotate_direction(double* point);
        void                        rosNodeInit();
        void                        camera_callback(const sensor_msgs::PointCloud2 msg);
        

};