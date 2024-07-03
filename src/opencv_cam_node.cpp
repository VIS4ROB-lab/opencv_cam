#include "opencv_cam/opencv_cam_node.hpp"

#include <iostream>

#include "camera_calibration_parsers/parse.hpp"

namespace opencv_cam {

std::string mat_type2encoding(int mat_type) {
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("unsupported encoding type");
  }
}

OpencvCamNode::OpencvCamNode(const rclcpp::NodeOptions &options)
    : Node("opencv_cam", options), canceled_(false) {
  RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d",
              options.use_intra_process_comms());

  // Initialize parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) \
  CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
  CXT_MACRO_INIT_PARAMETERS(OPENCV_CAM_ALL_PARAMS, validate_parameters)

  // Register for parameter changed. NOTE at this point nothing is done when
  // parameters change.
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
  CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, OPENCV_CAM_ALL_PARAMS,
                                        validate_parameters)

  // Log the current parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
  CXT_MACRO_LOG_SORTED_PARAMETERS(
      RCLCPP_INFO, get_logger(), "opencv_cam Parameters", OPENCV_CAM_ALL_PARAMS)

  // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
  CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), OPENCV_CAM_ALL_PARAMS)

  RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

  publish_fps_ = cxt_.fps_;
  next_stamp_ = now();

  if (!cxt_.info_only_) {
    // Open file or device
    if (cxt_.file_) {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.filename_);

      if (!capture_->isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open file %s",
                     cxt_.filename_.c_str());
        return;
      }

      if (cxt_.fps_ > 0) {
        // Publish at the specified rate
        publish_fps_ = cxt_.fps_;
      } else {
        // Publish at the recorded rate
        publish_fps_ = static_cast<int>(capture_->get(cv::CAP_PROP_FPS));
      }

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      RCLCPP_INFO(get_logger(),
                  "file %s open, width %g, height %g, publish fps %d",
                  cxt_.filename_.c_str(), width, height, publish_fps_);

    } else {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.index_);

      if (!capture_->isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open device %d", cxt_.index_);
        return;
      }

      if (cxt_.height_ > 0) {
        capture_->set(cv::CAP_PROP_FRAME_HEIGHT, cxt_.height_);
      }

      if (cxt_.width_ > 0) {
        capture_->set(cv::CAP_PROP_FRAME_WIDTH, cxt_.width_);
      }

      if (cxt_.fps_ > 0) {
        capture_->set(cv::CAP_PROP_FPS, cxt_.fps_);
      }

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      double fps = capture_->get(cv::CAP_PROP_FPS);
      RCLCPP_INFO(get_logger(),
                  "device %d open, width %g, height %g, device fps %g",
                  cxt_.index_, width, height, fps);
    }
  }

  auto qos_profile =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  auto qos_override_options = rclcpp::QosOverridingOptions({
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
  });

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = qos_override_options;
  rclcpp::SubscriptionOptions sub_options;
  sub_options.qos_overriding_options = qos_override_options;

  assert(!cxt_.camera_info_path_
              .empty());  // readCalibration will crash if file_name is ""
  std::string camera_name;
  if (camera_calibration_parsers::readCalibration(
          cxt_.camera_info_path_, camera_name, camera_info_msg_)) {
    RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
    camera_info_msg_.header.frame_id = cxt_.camera_frame_id_;
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_info", qos_profile);
  } else {
    RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
    camera_info_pub_ = nullptr;
  }

  if (!cxt_.info_only_) {
    image_pub_ = image_transport::create_publisher(
        this, "image_raw", rmw_qos_profile_default, pub_options);
  } else {
    image_sub_ = image_transport::create_subscription(
        this, "image_raw",
        std::bind(&OpencvCamNode::image_callback, this, std::placeholders::_1),
        cxt_.in_transport_, rmw_qos_profile_default, sub_options);
  }

  // Run loop on it's own thread
  thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

  RCLCPP_INFO(get_logger(), "start publishing");
}

OpencvCamNode::~OpencvCamNode() {
  // Stop loop
  canceled_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void OpencvCamNode::validate_parameters() {}

void OpencvCamNode::loop() {
  cv::Mat frame;

  auto stamp = now();

  while (rclcpp::ok() && !canceled_.load()) {
    // Read a frame, if this is a device block until a frame is available
    if (!cxt_.info_only_) {
      if (!capture_->read(frame)) {
        RCLCPP_INFO(get_logger(), "EOF, stop publishing");
        break;
      }

      // Avoid copying image message if possible
      sensor_msgs::msg::Image::UniquePtr image_msg(
          new sensor_msgs::msg::Image());

      // Convert OpenCV Mat to ROS Image
      image_msg->header.stamp = stamp;
      image_msg->header.frame_id = cxt_.camera_frame_id_;
      image_msg->height = frame.rows;
      image_msg->width = frame.cols;
      image_msg->encoding = mat_type2encoding(frame.type());
      image_msg->is_bigendian = false;
      image_msg->step =
          static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      image_msg->data.assign(frame.datastart, frame.dataend);

      // Publish
      image_pub_.publish(std::move(image_msg));

      if (camera_info_pub_) {
        camera_info_msg_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_msg_);
      }
    }

    // Sleep if required
    if (cxt_.file_ || cxt_.info_only_) {
      using namespace std::chrono_literals;
      next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000ns / publish_fps_};
      auto wait = next_stamp_ - stamp;
      if (wait.nanoseconds() > 0) {
        std::this_thread::sleep_for(
            static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
      }
    }
  }
}

void OpencvCamNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  camera_info_msg_.header.stamp = msg->header.stamp;
  camera_info_pub_->publish(camera_info_msg_);
}
}  // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)