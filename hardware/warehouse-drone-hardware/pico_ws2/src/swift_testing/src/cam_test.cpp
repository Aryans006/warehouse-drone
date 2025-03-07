#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("cam_test");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  // TransportHints does not actually declare the parameter
  node->declare_parameter<std::string>("image_transport", "raw");
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::TransportHints hints(node.get());
  image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback, &hints);
  rclcpp::spin(node);
  cv::destroyWindow("view");

  return 0;
}
