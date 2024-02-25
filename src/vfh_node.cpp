#include <vfh_algorithm.hpp>
#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "vfh_by_depth_cam_node");
    ros::NodeHandle n;
    ros::Rate rate(60);

    VFH vfh = VFH(n);

    while(ros::ok()){
        vfh.update();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}