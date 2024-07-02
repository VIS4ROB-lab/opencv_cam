#ifndef OPENCV_CAM_HPP
#define OPENCV_CAM_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv_cam/camera_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace opencv_cam {

class OpencvCamNode : public rclcpp::Node {
  CameraContext cxt_;

  std::thread thread_;
  std::atomic<bool> canceled_;

  std::shared_ptr<cv::VideoCapture> capture_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int publish_fps_;
  rclcpp::Time next_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

 public:
  explicit OpencvCamNode(const rclcpp::NodeOptions &options);
  ~OpencvCamNode() override;

 private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void validate_parameters();
  void loop();
};

}  // namespace opencv_cam

#endif  // OPENCV_CAM_HPP
