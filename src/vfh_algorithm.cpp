#include <vfh_algorithm.hpp>

/*
    rostopic pub /vehicle/desPoseVFH geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 100.0
    y: 100.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"  
    */

VFH::VFH(ros::NodeHandle n_){
    n = n_;
    
    // положение добавляемое к текущему
    free_distance = 6;
    // максимальный обзор камеры в метрах
    range_max = 9;
    // если дистанция до препятствия больше чем distance_to_obstacle то область на гистограмме будет закрашена
    distance_to_obstacle = 9;
    // размер карты = 1/map_detail
    map_detail = 2; 
    // визуализация в rviz
    is_visualization = false; 
    
    width = 640;
    height = 480;
    fov = 1.3962634;
    // create map and histogram
    vector<vector<bool>> histogram_(range_max*2*map_detail , vector<bool> (range_max*2*map_detail)); 
    histogram = histogram_;
    vector<vector<int>> map_countable_(range_max*2*map_detail , vector<int> (range_max*2*map_detail)); 
    map_countable = map_countable_;
    vector<vector<double>> map_distance_(range_max*2*map_detail , vector<double> (range_max*2*map_detail)); 
    map_distance = map_distance_;
    // create message
    marker_msgs.action = visualization_msgs::Marker::ADD;
    marker_msgs.header.frame_id = "map";
    marker_msgs.pose.orientation.w = 0;
    marker_msgs.pose.orientation.x = 0;
    marker_msgs.pose.orientation.y = 0;
    marker_msgs.pose.orientation.z = 0;
    marker_msgs.color.a = 1.0f;
    marker_msgs.color.g = 1.0;

    marker_msgs.scale.x = 1.0/map_detail;
    marker_msgs.scale.y = 1.0/map_detail;
    marker_msgs.scale.z = 1.0/map_detail;

    marker_msgs.type = visualization_msgs::Marker::CUBE_LIST;
    marker_msgs.id = 1;

    // set base position
    desired_position.pose.position.x = 0;
    desired_position.pose.position.y = 0;
    desired_position.pose.position.z = 5;  

    rosNodeInit();
}

void VFH::update(){
    if (point_cloud.size()){
        create_histogram(point_cloud);
        double direction[4];
        choose_direction(direction);
        if (choose_free_direction(direction)){
            rotate_direction(direction);
            if (is_visualization){
                send_visualization_message(direction);
            }
            send_position_message(direction);
        }
        else{
            double angle = atan(direction[1]/direction[0])*180/M_PI;
            if (direction[1]<= 0 and direction[0]>=0){angle+=360;}
            if (direction[1]<0 and direction[0]<0){angle+=180;}
            if (direction[1]>=0 and direction[0]<0){angle+=180;}

            direction[0] = current_position.pose.position.x;
            direction[1] = current_position.pose.position.y;
            direction[2] = current_position.pose.position.z > 1? current_position.pose.position.z: 1;
            direction[3] = angle;

            send_position_message(direction);
            cout << "No direction to move!" << endl;
        }
    }
}

void VFH::rosNodeInit(){
    cout << "VFH algorithm initialized" << endl;
    desired_position_sub = n.subscribe<geometry_msgs::PoseStamped>("vehicle/desPoseVFH", 10, &VFH::desired_position_callback, this);
    current_position_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &VFH::current_position_callback, this);
    depth_camera_sub = n.subscribe<sensor_msgs::PointCloud2>("/realsense_d435_depth/points", 10, &VFH::camera_callback, this);
    marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    desired_position_pub = n.advertise<geometry_msgs::PoseStamped>("vehicle/desPose", 10);
    
}


void VFH::desired_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    desired_position = *msg;

}

void VFH::current_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
    q = msg->pose.orientation;
}

void VFH::camera_callback(const sensor_msgs::PointCloud2 msg){
    pcl::fromROSMsg(msg,point_cloud);
}

void VFH::choose_direction(double* direction){
    double x, y, z;
    double static z_last = 0, x_last = 0, y_last = 0;
    x = desired_position.pose.position.x-current_position.pose.position.x;
    y = desired_position.pose.position.y-current_position.pose.position.y;
    z = desired_position.pose.position.z-current_position.pose.position.z;

    if (!(x > -0.2 and x < 0.2 and z < 0.2 and z > -0.2 and y > -0.2 and y < 0.2)){
        //z = (z_last*4/5 + z*1/5);
        //y = (y_last*4/5 + y*1/5);
        //x = (x_last*4/5 + x*1/5);
        direction[0] = abs(x) > free_distance? x/abs(x)*free_distance: int(x);
        direction[1] = abs(y) > free_distance? y/abs(y)*free_distance: int(y);
        direction[2] = abs(z) > free_distance? z/abs(z)*free_distance: int(z);

        x_last = x;
        y_last = y;
        z_last = z;
    }
    else{cout << "We are near Point" << endl;}
}

// choose free direction
bool VFH::choose_free_direction(double *direction){
    static int orientationY = 1, timer = 2;
    bool is_free = false;
    double orientationZ = 1, x, y;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    // global to local
    x = direction[0]*cos(-yaw) - direction[1]*sin(-yaw);
    for (int z = direction[2]*map_detail; z < range_max*map_detail && -range_max*map_detail < z;z+= orientationZ){
        if (z == range_max*map_detail){
            z = direction[2]-1;
            orientationZ = -1;
            }
        y = (direction[1]*cos(-yaw) + direction[0]*sin(-yaw))*map_detail; 
        for (;y < range_max*map_detail and y > -range_max*map_detail;y+= orientationY){
            is_free = true;
            for (int i = -map_detail*1.5; i < map_detail*1.5+1; i++){
                for (int j = -map_detail; j < map_detail+1; j++){
                    if ((y+i) >= range_max*map_detail || (y+i) <= -range_max*map_detail || (z+j) >= range_max*map_detail || (z+j) <= -range_max*map_detail || !histogram[j + round(z) + range_max*map_detail][i + round(y) + range_max*map_detail]){
                        is_free = false;
                        break;
                    }
                }
                if (!is_free){break;}
            }
            if (is_free){
                if (timer != 2){
                    timer++;
                }
                // local to global
                y/= map_detail; z /= map_detail;
                direction[0] = x*cos(yaw) - y*sin(yaw);
                direction[1] = y*cos(yaw) + x*sin(yaw);
                direction[2] = z; 
                return true;
            }
        }
    }
    timer-=1;
    if (!timer){
        orientationY = -orientationY;
        timer = 2;
    }
    return false;
}


void VFH::create_histogram(pcl::PointCloud<pcl::PointXYZ> msg){
    marker_msgs.points.clear();
    // clear all maps
    for (int i = 0; i < range_max*2*map_detail; i++){
        for (int j = 0; j < range_max*2*map_detail; j++){
                map_distance[i][j] = 0;
                map_countable[i][j] = 0;
                histogram[i][j] = false;
        }
    }
    
    // fill distance and countable maps
    for (long int i = 0; i < width*height; i++){
        if (!isnan(msg[i].z) && !isnan(-msg[i].x) && !isnan(-msg[i].y)){
            map_distance[(int)((-msg[i].y+range_max)*map_detail)][(int)((-msg[i].x+range_max)*map_detail)] += msg[i].z;
            map_countable[(int)((-msg[i].y+range_max)*map_detail)][(int)((-msg[i].x+range_max)*map_detail)] += 1;
        }
    }

    //  create a histogram
    double distance = 0;
    for (int i = 0; i < range_max*2*map_detail; i++){
        for (int j = 0; j < range_max*2*map_detail; j++){
            if (!map_countable[i][j]){
                distance = 0;
            }
            else{
                distance = map_distance[i][j]/map_countable[i][j]; 
            }
            if (distance < (double)distance_to_obstacle){
                histogram[i][j] = false; // obstacle = 0
            }
            else{
                histogram[i][j] = true;// free = 1
                
            }
        }
    }
}

void VFH::send_visualization_message(double* point){
    marker_msgs.points.clear();    
    visualization_msgs.x = point[0];
    visualization_msgs.y = point[1];
    visualization_msgs.z = point[2];
    marker_msgs.points.push_back(visualization_msgs);
    marker_pub.publish(marker_msgs); 
}

void VFH::send_position_message(double* direction){  
    position_msgs.pose.position.x = direction[0];
    position_msgs.pose.position.y = direction[1];
    position_msgs.pose.position.z = direction[2];
    position_msgs.pose.orientation.w = direction[3];
    desired_position_pub.publish(position_msgs);
}

void VFH::rotate_direction(double* direction){
    direction[3] = atan(direction[1]/direction[0])*180/M_PI;
    if (direction[1]<= 0 and direction[0]>=0){direction[3]+=360;}
    if (direction[1]<0   and direction[0]<0){direction[3]+=180;}
    if (direction[1]>=0  and direction[0]<0){direction[3]+=180;}
    direction[0] += current_position.pose.position.x;
    direction[1] += current_position.pose.position.y;
    direction[2] += current_position.pose.position.z;
}