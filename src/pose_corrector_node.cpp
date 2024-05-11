#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <iostream>

#include <pose_corrector/utils.h>


using namespace sensor_msgs;
using namespace message_filters;


void image_callback(const ImageConstPtr& colorImageMsg, const ImageConstPtr& depthImageMsg)
{

    cv_bridge::CvImagePtr colorImagePtr;
    cv_bridge::CvImagePtr depthImagePtr;
    
	colorImagePtr = cv_bridge::toCvCopy(colorImageMsg, sensor_msgs::image_encodings::RGB8);
	depthImagePtr = cv_bridge::toCvCopy(depthImageMsg, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat colorImageRgb = colorImagePtr->image;
    cv::Mat depthImage = depthImagePtr->image;


}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_corrector");

	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);

	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, depth_sub, 10);
	sync.registerCallback(boost::bind(&image_callback, _1, _2));

  	ros::spin();

	return 0;
}