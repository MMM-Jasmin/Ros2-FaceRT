//
// Created by dave on 10.01.22.
//

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
//native ros interfaces and libs
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
//custom interfaces
#include "rtface_pkg/msg/list_id_image.hpp"
#include "rtface_pkg/msg/id_image.hpp"
// opencv and bridge
#include "cv_bridge/cv_bridge.h"
#include "rtface_pkg/msg/id_name.hpp"
#include "rtface_pkg/msg/list_id_name.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "image_"
#include <utility>

//#include <FaceDetector.h>

static
std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
    {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
    {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

static
std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
    {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
    {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
};
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

class Cam2Image : public rclcpp::Node
{
public:
//  IMAGE_TOOLS_PUBLIC
  explicit Cam2Image(const rclcpp::NodeOptions & options)
      : Node("cam2image", options),
        is_flipped_(true),
        publish_number_(1u)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Do not execute if a --help option was provided
    if (help(options.arguments())) {
      // TODO(jacobperron): Replace with a mechanism for a node to "unload" itself
      // from a container.
      exit(0);
    }
    parse_parameters();
    initialize();
  }

private:
//  IMAGE_TOOLS_LOCAL
  void initialize()
  {
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            // The history policy determines how messages are saved until taken by
            // the reader.
            // KEEP_ALL saves all messages until they are taken.
            // KEEP_LAST enforces a limit on the number of messages that are saved,
            // specified by the "depth" parameter.
            history_policy_,
            // Depth represents how many messages to store in history when the
            // history policy is KEEP_LAST.
            depth_
        ));
    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.

    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
    qos_profile = qos_profile.keep_last(static_cast<size_t>(5));
    qos_profile.reliable();

    qos.reliability(reliability_policy_);
    pub_ = create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", qos_profile);

    // Subscribe to a message that will toggle flipping or not flipping, and manage the state in a
    // callback
    auto callback = [this](const std_msgs::msg::Bool::SharedPtr msg) -> void
    {
      this->is_flipped_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Set flip mode to: %s", this->is_flipped_ ? "on" : "off");
    };
    // Set the QoS profile for the subscription to the flip message.
    sub_ = create_subscription<std_msgs::msg::Bool>(
        "flip_image", rclcpp::SensorDataQoS(), callback);

//    if (!burger_mode_) {
      // Initialize OpenCV video capture stream.
      // Always open device 0.


      //G-streamer pipeline test
      // std::string pipeline = gstreamer_pipeline(
      //   width_, height_, width_, height_, freq_, 0
      // );
      // cap.open(cam_id_);
      // cap.open(pipeline, cv::CAP_GSTREAMER);


      std::cout<<"pre Try to load video. video_test: "<< video_test_<< std::endl;
      if (video_test_){
        std::cout<<"Try to load video."<< std::endl;
        cap.open(video_);
      }else{
        cap.open(cam_id_);
      }

      // Set the width and height based on command line arguments.
      cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
      if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        throw std::runtime_error("Could not open video stream");
      }
//    }

    // Start main timer loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
        std::bind(&Cam2Image::timerCallback, this));
  }

  /// Publish camera, or burger, image.
  void timerCallback()
  {
    cv::Mat frame;

    // Initialize a shared pointer to an Image message.
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;

    // Get the frame from the video capture.
//    if (burger_mode_) {
//      frame = burger_cap.render_burger(width_, height_);
//    } else {
//
//    }
    cap >> frame;
    // If no frame was grabbed, return early
    if (frame.empty()) {
      return;
    }

    // Conditionally flip the image
    if (is_flipped_) {
      cv::flip(frame, frame, 1);
    }

    // Convert to a ROS image
    convert_frame_to_message(frame, *msg);

    // Conditionally show image
    if (show_camera_) {
      cv::Mat cvframe = frame;
      // Show the image in a window called "cam2image".
      cv::imshow("cam2image", cvframe);
      // Draw the image to the screen and wait 1 millisecond.
      cv::waitKey(1);
    }

    // Publish the image message and increment the frame_id.
    RCLCPP_INFO(get_logger(), "Publishing image #%zd", publish_number_++);
    pub_->publish(std::move(msg));
  }

  bool help(const std::vector<std::string> args)
  {
    if (std::find(args.begin(), args.end(), "--help") != args.end() ||
        std::find(args.begin(), args.end(), "-h") != args.end())
    {
      std::stringstream ss;
      ss << "Usage: cam2image [-h] [--ros-args [-p param:=value] ...]" << std::endl;
      ss << "Publish images from a camera stream." << std::endl;
      ss << "Example: ros2 run image_tools cam2image --ros-args -p reliability:=best_effort";
      ss << std::endl << std::endl;
      ss << "Options:" << std::endl;
      ss << "  -h, --help\tDisplay this help message and exit";
      ss << std::endl << std::endl;
      ss << "Parameters:" << std::endl;
      ss << "  reliability\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
      ss << std::endl;
      ss << "  history\tHistory QoS setting. Either 'keep_last' (default) or 'keep_all'.";
      ss << std::endl;
      ss << "\t\tIf 'keep_last', then up to N samples are stored where N is the depth";
      ss << std::endl;
      ss << "  depth\t\tDepth of the publisher queue. Only honored if history QoS is 'keep_last'.";
      ss << " Default value is 10";
      ss << std::endl;
      ss << "  frequency\tPublish frequency in Hz. Default value is 30";
      ss << std::endl;
      ss << "  burger_mode\tProduce images of burgers rather than connecting to a camera";
      ss << std::endl;
      ss << "  show_camera\tShow camera stream. Either 'true' or 'false' (default)";
      ss << std::endl;
      ss << "  width\t\tWidth component of the camera stream resolution. Default value is 320";
      ss << std::endl;
      ss << "  height\tHeight component of the camera stream resolution. Default value is 240";
      ss << std::endl;
      ss << "  frame_id\t\tID of the sensor frame. Default value is 'camera_frame'";
      ss << std::endl << std::endl;
      ss << "Note: try running v4l2-ctl --list-formats-ext to obtain a list of valid values.";
      ss << std::endl;
      std::cout << ss.str();
      return true;
    }
    return false;
  }

  void parse_parameters()
  {
    // Parse 'reliability' parameter
    rcl_interfaces::msg::ParameterDescriptor reliability_desc;
    reliability_desc.description = "Reliability QoS setting for the image publisher";
    reliability_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_reliability_policy_map) {
      reliability_desc.additional_constraints += entry.first + " ";
    }
    const std::string reliability_param = this->declare_parameter(
        "reliability", "reliable", reliability_desc);
    auto reliability = name_to_reliability_policy_map.find(reliability_param);
    if (reliability == name_to_reliability_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS reliability setting '" << reliability_param << "'";
      throw std::runtime_error(oss.str());
    }
    reliability_policy_ = reliability->second;

    // Parse 'history' parameter
    rcl_interfaces::msg::ParameterDescriptor history_desc;
    history_desc.description = "History QoS setting for the image publisher";
    history_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_history_policy_map) {
      history_desc.additional_constraints += entry.first + " ";
    }
    const std::string history_param = this->declare_parameter(
        "history", name_to_history_policy_map.begin()->first, history_desc);
    auto history = name_to_history_policy_map.find(history_param);
    if (history == name_to_history_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS history setting '" << history_param << "'";
      throw std::runtime_error(oss.str());
    }
    history_policy_ = history->second;

    // Declare and get remaining parameters
    depth_ = this->declare_parameter("depth", 5);
    freq_ = this->declare_parameter("frequency", 30.0);
    show_camera_ = this->declare_parameter("show_camera", false);
    width_ = this->declare_parameter("width", 320);
    height_ = this->declare_parameter("height", 240);
//    rcl_interfaces::msg::ParameterDescriptor burger_mode_desc;
//    burger_mode_desc.description = "Produce images of burgers rather than connecting to a camera";
//    burger_mode_ = this->declare_parameter("burger_mode", false, burger_mode_desc);
    frame_id_ = this->declare_parameter("frame_id", 0);
    cam_id_ = this->declare_parameter("cam_id", 0);
    video_test_ = this->declare_parameter("video_test", false);
    video_ = this->declare_parameter("video", "empty");
  }

  /// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
  /**
   * \param[in] mat_type The OpenCV encoding type.
   * \return A string representing the encoding type.
   */
//  IMAGE_TOOLS_LOCAL
static std::string mat_type2encoding(int mat_type)
  {
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
        throw std::runtime_error("Unsupported encoding type");
    }
  }

  /// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
  /**
   * \param[in] frame The OpenCV matrix/image to convert.
   * \param[in] frame_id ID for the ROS message.
   * \param[out] Allocated shared pointer for the ROS Image message.
   */
//  IMAGE_TOOLS_LOCAL
  void convert_frame_to_message(
      const cv::Mat & frame, sensor_msgs::msg::Image & msg)
  {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    frame_id_++;
    msg.header.frame_id = std::to_string(frame_id_);
    msg.header.stamp = rclcpp::Clock().now();
  }

  cv::VideoCapture cap;
//  burger::Burger burger_cap;
  bool video_test_;
  std::string video_{""};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS parameters
  long cam_id_;
  bool show_camera_;
  size_t depth_;
  double freq_;
  rmw_qos_reliability_policy_t reliability_policy_;
  rmw_qos_history_policy_t history_policy_;
  size_t width_;
  size_t height_;
//  bool burger_mode_;
  long frame_id_;

  /// If true, will cause the incoming camera image message to flip about the y-axis.
  bool is_flipped_;
  /// The number of images published.
  size_t publish_number_;
};

int main(int argc, char **argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Cam2Image>(options));

  rclcpp::shutdown();
  return 0;
}