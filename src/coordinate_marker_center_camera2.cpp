#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>



class MarkerCoordinate{
    public:
        float u_left = 0;
        float v_left = 0;
        float u_right = 0;
        float v_right = 0;

        float cam_x, cam_y, cam_z;
        float X, Y, Z;

        // set camera internal param
        float fx_left = 528.89;          // focal length
        float fy_left = 528.52;
        float cx_left = 662.885;         // principal points
        float cy_left = 360.4255;

        float fx_right = 529.12;          // focal length
        float fy_right = 528.865;
        float cx_right = 655.525;         // principal points
        float cy_right = 362.9415;

        float b = 0.12;

        float f = (fx_left + fx_right + fy_left + fy_right) / 4;

        ros::NodeHandle nh;

        ros::Subscriber subPointLeft;
        ros::Subscriber subPointRight;

        ros::Publisher pubCoordinate;
    
        MarkerCoordinate();
        void callbackLeft(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callbackRight(const std_msgs::Float32MultiArray::ConstPtr& msg);
        //void coordinateCallback(const sensor_msgs::Image::ConstPtr& msg);
};


MarkerCoordinate::MarkerCoordinate(){
    subPointLeft = nh.subscribe("/ar_point_left", 10, &MarkerCoordinate::callbackLeft, this);
    subPointRight = nh.subscribe("/ar_point_right", 10, &MarkerCoordinate::callbackRight, this);
	//subDepth = nh.subscribe("/robot2/zed_nodelet/depth/depth_registered", 10, &MarkerCoordinate::coordinateCallback, this);
    pubCoordinate = nh.advertise<geometry_msgs::Point>("/marker_coordinate", 1);
}


void MarkerCoordinate::callbackLeft(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // Get marker
    u_left = msg->data[0];
    v_left = msg->data[1];

    // If find marker
    if(u_left && v_left){
        //ROS_INFO("Marker Point (Left): (%d, %d)", u_left, v_left);
    }
    // If not find marker
    else{
        ROS_INFO("Marker is NOT found! (Left)");
    }
}

void MarkerCoordinate::callbackRight(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // Get marker
    u_right = msg->data[0];
    v_right = msg->data[1];

    // If find marker
    if(u_right && v_right){
        //ROS_INFO("Marker Point (Right): (%d, %d)", u_right, v_right);
    }
    // If not find marker
    else{
        ROS_INFO("Marker is NOT found! (Right)");
    }

    // Get marker XYZ coordinate (Left camera)
    cam_z = b*f / (u_left - u_right);     //stereo depth
    cam_x = cam_z / fx_left * (u_left - cx_left);
    cam_y = cam_z / fy_left * (v_left - cy_left);

    // Transform to ROS coordinate system
    X = cam_z;
    Y = -cam_x;
    Z = -cam_y + 0.36;  // marker point -> camera point

    // if marker is detected
    if(u_left+v_left && u_right+v_right){
        geometry_msgs::Point point;

        // Output the depth
        //ROS_INFO("Marker Depth : %g m", depths[centerIdx]);

        // Output the XYZ coordinate
        ROS_INFO("2 -> 1 : (x: %g, y: %g, z: %g)", X, Y, Z);

        // Publish the XYZ coodinate
        point.x = X;
        point.y = Y;
        point.z = Z;
        pubCoordinate.publish(point);
    }

}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "coordinate_marker_center_camera2");

    MarkerCoordinate det_coordinate;

    ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
