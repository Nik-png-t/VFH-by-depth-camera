#include <vfh_algorithm.hpp>


VFH::VFH(ros::NodeHandle n_){
    n = n_;
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
    z: 5.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"  
    */
    free_distance = 4;
    range_max = 9;
    distance_to_obstacle = 9;
    map_detail = 2; // eat more resurses
    is_visualization = true;
    width = 640;
    height = 480;
    fov = 1.3962634;
    // create map and histogram
    vector<vector<bool>> histogram_( range_max*2*map_detail , vector<bool> (range_max*2*map_detail)); 
    histogram = histogram_;
    vector<vector<int>> map_countable_( range_max*2*map_detail , vector<int> (range_max*2*map_detail)); 
    map_countable = map_countable_;
    vector<vector<double>> map_distance_( range_max*2*map_detail , vector<double> (range_max*2*map_detail)); 
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
    
    
    //marker_msgs.type = visualization_msgs::Marker::POINTS;
    // marker_msgs.id = 0;

    marker_msgs.scale.x = 1.0/map_detail;
    marker_msgs.scale.y = 1.0/map_detail;
    marker_msgs.scale.z = 1.0/map_detail;

    marker_msgs.type = visualization_msgs::Marker::CUBE_LIST;
    marker_msgs.id = 1;
    
    // marker_msgs.scale.x = voxel_size;
    // marker_msgs.scale.y = voxel_size;
    // marker_msgs.scale.z = voxel_size;

    //                   ///                         //
    desired_position.pose.position.x = 0;
    desired_position.pose.position.y = 0;
    desired_position.pose.position.z = 5;  
    //                   ///                         //
    

    rosNodeInit();
}

void VFH::update(){
    if (point_cloud.size()){
        create_histogram(point_cloud);
        double direction[4];
        choose_direction(direction);
        if (choose_free_direction(direction)){
            rotate_direction(direction);
            //send_visualization_message(direction);
            send_position_message(direction);
        }
        else{
            double x = direction[0]/map_detail-range_max, y = direction[1]/map_detail-range_max;
            double angle = atan(y/x)*180/M_PI;
            if (y<= 0 and x>=0){angle+=360;}
            if (y<0 and x<0){angle+=180;}
            if (y>=0 and x<0){angle+=180;}

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
    double x, y, z, r, need_z, need_x, need_y, need_angle, angle;
    double static angle_last = 0, z_last = 0, x_last = 0, y_last = 0;
    int i, j;
    x = desired_position.pose.position.x-current_position.pose.position.x;
    y = desired_position.pose.position.y-current_position.pose.position.y;
    z = desired_position.pose.position.z-current_position.pose.position.z;
    cout << x << " " << y << " " << z << endl;
    r = sqrt(pow(x, 2) + pow(y, 2));
    if (!(r > -0.2 and r < 0.2 and z < 0.2 and z > -0.2)){
        need_z = (z_last*4/5 + z*1/5);
        need_y = (y_last*4/5 + y*1/5);
        need_x = (x_last*4/5 + x*1/5);
        direction[0] = (range_max + (abs(need_x) > free_distance? need_x/abs(need_x)*free_distance: int(need_x)))*map_detail;//int(distance*cos(need_angle/180.0*M_PI));
        direction[1] = (range_max + (abs(need_y) > free_distance? need_y/abs(need_y)*free_distance: int(need_y)))*map_detail;//int(distance*sin(need_angle/180.0*M_PI));
        direction[2] = (range_max + (abs(need_z) > free_distance? need_z/abs(need_z)*free_distance: int(need_z)))*map_detail;
        
        cout << "Desired position: " << direction[0]- map_detail*range_max << " " << direction[1]- map_detail*range_max << " " << direction[2]- map_detail*range_max << endl;
        x_last = need_x;
        y_last = need_y;
        z_last = need_z;
    }
    else{cout << "We are near Point" << endl;}
}

// choose free direction
bool VFH::choose_free_direction(double *direction){
    static int orientationY = 1, timer = 10;
    bool is_free = false;
    int orientationZ = 1, x, y;

    marker_msgs.points.clear();
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    direction[0] -= map_detail*range_max; direction[1] -= map_detail*range_max;
    x = round(direction[0]*cos(-yaw) - direction[1]*sin(-yaw))+range_max*map_detail;
    cout << "choose free direction" << endl;
    cout << "start position: " << direction[0] << " " << direction[1] << " " << direction[2] << endl;
    cout << "reverse position: " << x << " " << round(direction[1]*cos(yaw) + direction[0]*sin(yaw)) << " " << direction[2] << endl;
        for (int z = direction[2]; z < 2*range_max*map_detail && -1 < z;){
            y = round(direction[1]*cos(-yaw) + direction[0]*sin(-yaw))+range_max*map_detail;
            
            for (;y < 2*range_max*map_detail and y > -1;){
                x -= range_max*map_detail; y-=range_max*map_detail;

                visualization_msgs.x = current_position.pose.position.x+(round(x*cos(yaw) - y*sin(yaw)))/map_detail;
                visualization_msgs.y = current_position.pose.position.y+(round(y*cos(yaw) + x*sin(yaw)))/map_detail;
                visualization_msgs.z = (z/map_detail - range_max) +current_position.pose.position.z;
                marker_msgs.points.push_back(visualization_msgs);
                x += range_max*map_detail; y+=range_max*map_detail;

                is_free = true;
                for (int i = -2; i < 3; i++){
                    for (int j = -2; j < 3; j++){
                        if ((y+i) >= 2*range_max*map_detail || (y+i) <= -1 || (z+j) >= 2*range_max*map_detail || (z+j) <= -1 || !histogram[z + j][y + i]){
                            is_free = false;
                            break;
                        }
                    }
                    if (!is_free){break;}
                }
                if (is_free){
                    if (timer != 10){
                        timer++;
                    }
                    //int distance = map_distance[z][y]/map_countable[z][y] > x ? x:map_distance[z][y]/map_countable[z][y]; 
                    //x = abs(distance) > free_distance ? distance/abs(distance)*free_distance: distance;
                    
                    //  transfer from the global coordinate system to the local one
                    x -= range_max*map_detail; y-=range_max*map_detail;
                    direction[0] = round(x*cos(yaw) - y*sin(yaw)) + range_max*map_detail;
                    direction[1] = round(y*cos(yaw) + x*sin(yaw)) + range_max*map_detail;
                    direction[2] = z;

                    cout << "Checked reverse: " << x << " " << y << " " << z << endl;
                    cout << "Checked: " << direction[0] << " " << direction[1] << " " << direction[2] << endl;
                    //marker_pub.publish(marker_msgs); 
                    return true;
                }
                
                
                cout << "Was checked: " << y << " " << z << endl;
                y+= orientationY;
                }
            z+= orientationZ;
            if (z == 2*range_max*map_detail){
                z = direction[2]-1;
                orientationZ = -1;
                }
        }
    timer-=1;
    if (!timer){
        orientationY = -orientationY;
        timer = 10;
    }
    direction[0] += map_detail*range_max; direction[1] += map_detail*range_max;
    //marker_pub.publish(marker_msgs); 
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
    
    // fill all maps
    for (long int i = 0; i < width*height; i++){
        if (!isnan(msg[i].z) && !isnan(-msg[i].x) && !isnan(-msg[i].y)){
            map_distance[(int)((-msg[i].y+range_max)*map_detail)][(int)((-msg[i].x+range_max)*map_detail)] += msg[i].z;
            map_countable[(int)((-msg[i].y+range_max)*map_detail)][(int)((-msg[i].x+range_max)*map_detail)] += 1;
        }
    }

    //  create a histogram
    double distance_ = 0;
    for (int i = 0; i < range_max*2*map_detail; i++){
        for (int j = 0; j < range_max*2*map_detail; j++){
            if (!map_countable[i][j]){
                distance_ = 0;
            }
            else{
                distance_ = map_distance[i][j]/map_countable[i][j]; 
            }
            if (distance_ < distance_to_obstacle){
                histogram[i][j] = false; // obstacle == 0
            }
            else{
                histogram[i][j] = true;// free == 1
                
            }
            if (distance_ > 1){
                visualization_msgs.x = distance_;
                visualization_msgs.y = j/map_detail-range_max;
                visualization_msgs.z = i/map_detail - range_max;
                marker_msgs.points.push_back(visualization_msgs);
            }
        }
    }
    
    // send point    
    //marker_pub.publish(marker_msgs); 
    
}

void VFH::send_visualization_message(double* point){
    marker_msgs.points.clear();
    // send point    
    
    visualization_msgs.x = point[0];
    visualization_msgs.y = point[1];
    visualization_msgs.z = point[2];

    marker_msgs.color.r = 220;
    marker_msgs.color.g = 20;
    marker_msgs.color.b = 60;
    marker_msgs.color.a = 1;
    marker_msgs.id = 1;

    marker_msgs.points.push_back(visualization_msgs);
    //marker_pub.publish(marker_msgs); 

    marker_msgs.id = 0;
    marker_msgs.color.a = 1.0f;
    marker_msgs.color.g = 1.0;
}

void VFH::send_position_message(double* direction){
    // send point    
    position_msgs.pose.position.x = direction[0];
    position_msgs.pose.position.y = direction[1];
    position_msgs.pose.position.z = direction[2];
    position_msgs.pose.orientation.w = direction[3];
    desired_position_pub.publish(position_msgs);
}

void VFH::rotate_direction(double* direction){
    direction[0] = direction[0]/map_detail - range_max + current_position.pose.position.x;
    direction[1] = direction[1]/map_detail - range_max + current_position.pose.position.y;
    direction[2] = direction[2]/map_detail - range_max + current_position.pose.position.z;
    double y = direction[1] - current_position.pose.position.y;
    double x = direction[0] - current_position.pose.position.x;
    double angle = atan(y/x)*180/M_PI;
    if (y<= 0 and x>=0){angle+=360;}
    if (y<0  and x<0){angle+=180;}
    if (y>=0 and x<0){angle+=180;}

    direction[3] = angle;
    cout << "Current position: " << current_position.pose.position.x << " " << current_position.pose.position.y << " " << current_position.pose.position.z << endl;
    cout << "Desired position: " << direction[0] << " " << direction[1] << " " << direction[2] << " " << direction[3] << endl; 
}