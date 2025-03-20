#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h> // sensor_msgs::Image
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp> // cvtColor, ...
#include <opencv2/highgui/highgui.hpp> // imshow

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // Convert ROS Image message to OpenCV image
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Show the image
    cv::imshow("Received Image", cv_image->image);
    cv::waitKey(1); // Important for OpenCV to process the window events
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert image: %s", e.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber_cpp");

  ros::NodeHandle handle;

  // Wait for an image message to arrive
  auto sample = ros::topic::waitForMessage<sensor_msgs::Image>("image", handle, ros::Duration(3));
  
  if (!sample) {
    ROS_ERROR("Could not receive an image message");
    return 0;
  }

  ROS_INFO("Received an image sample");

  // Subscribe to the "image" topic
  ros::Subscriber subscriber = handle.subscribe("image", 10, imageCallback); 

  // Spin to keep the node alive
  ros::spin();

  return 0;
}
