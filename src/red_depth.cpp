#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>



class RedDepth{
    public:
        int u, v;

        ros::NodeHandle nh;
        ros::Subscriber subCenter;  //  = nh.subscribe("/red_center", 10, centerCallback);
        ros::Subscriber subDepth;   // = nh.subscribe("/zed2/zed_node/depth/depth_registered", 10, depthCallback);
    
        void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
        void centerCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
        RedDepth();
};


RedDepth::RedDepth(){
    subCenter = nh.subscribe("/red_center", 10, &RedDepth::centerCallback, this);
	subDepth = nh.subscribe("/zed2/zed_node/depth/depth_registered", 10, &RedDepth::depthCallback, this);
}


void RedDepth::centerCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    // Get red center of gravity from node "color_extract"
    u = msg->data[0];
    v = msg->data[1];

    // If find red center
    if(u && v){
        //ROS_INFO("Marker Point: (%d, %d)", u, v);
    }
    // If not find center
    else{
        ROS_INFO("Marker is NOT found!");
    }
    
}


void RedDepth::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);
    // Linear index of the center pixel
    int centerIdx = u + msg->width * v;
    if(u && v){
        // Output the measure
        ROS_INFO("Marker Depth : %g m", depths[centerIdx]);
    }
    
}



int main(int argc, char** argv) {
    
    ros::init(argc, argv, "maker_depth_meter");

    RedDepth det_depth;

    ros::Rate loop_rate(5);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}


    return 0;
}
