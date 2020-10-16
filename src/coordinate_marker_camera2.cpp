#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>



class MarkerCoordinate{
    public:
        int u = 0;
        int v = 0;
        float cam_x, cam_y, cam_z;
        float X, Y, Z;
        // set camera internal param
        float fx = 528.89;          // focal length
        float fy = 528.52;
        float cx = 662.885;         // principal points
        float cy = 360.4255;

        ros::NodeHandle nh;
        ros::Subscriber subCenter;
        ros::Subscriber subDepth;
    
        MarkerCoordinate();
        void centerCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
        void coordinateCallback(const sensor_msgs::Image::ConstPtr& msg);
};


MarkerCoordinate::MarkerCoordinate(){
    subCenter = nh.subscribe("/robot2/camera2_red_center", 10, &MarkerCoordinate::centerCallback, this);
	subDepth = nh.subscribe("/robot2/zed_nodelet/depth/depth_registered", 10, &MarkerCoordinate::coordinateCallback, this);
}


void MarkerCoordinate::centerCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    // Get marker center of gravity from node "color_extract"
    u = msg->data[0];
    v = msg->data[1];

    // If find red center
    if(u && v){
        //ROS_INFO("Marker Point: (%d, %d)", u, v);
    }
    // If not find marker
    else{
        ROS_INFO("Marker is NOT found!");
    }
}


void MarkerCoordinate::coordinateCallback(const sensor_msgs::Image::ConstPtr& msg) {

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    // Linear index of the center pixel
    int markerIdx = u + msg->width * v;

    // Get marker XYZ coordinate
    cam_z = depths[markerIdx];
    cam_x = cam_z / fx * (u - cx);
    cam_y = cam_z / fy * (v - cy);

    // Transform to ROS coordinate system
    X = cam_z;
    Y = -cam_x;
    Z = -cam_y;

    // if marker is detected
    if(u && v){
        // Output the depth
        //ROS_INFO("Marker Depth : %g m", depths[centerIdx]);
        // Output the XYZ coordinate
        ROS_INFO("Marker : (x: %g, y: %g, z: %g)", X, Y, Z);
    }
    
}



int main(int argc, char** argv) {
    
    ros::init(argc, argv, "camera2_coordinate_marker");

    MarkerCoordinate det_coordinate;

    ros::Rate loop_rate(5);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
