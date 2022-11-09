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
#include "rclcpp/time_source.hpp"
#include "rclcpp/clock.hpp"
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
    {"reliable",    RMW_QOS_POLICY_RELIABILITY_RELIABLE},
    {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

static
std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
    {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
    {"keep_all",  RMW_QOS_POLICY_HISTORY_KEEP_ALL}
};

class ShowImage : public rclcpp::Node
{
public:
//  IMAGE_TOOLS_PUBLIC
  explicit ShowImage(const rclcpp::NodeOptions &options)
      : Node("showimage", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Do not execute if a --help option was provided
//    if (help(options.arguments())) {
//      // TODO(jacobperron): Replace with a mechanism for a node to "unload" itself
//      // from a container.
//      exit(0);
//    }
    parse_parameters();
    initialize();
  }

private:
//  IMAGE_TOOLS_LOCAL
  void initialize()
  {
    if (show_image_) {
      // Initialize an OpenCV named window called "showimage".
      cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
      cv::waitKey(1);
    }
    clock_ = this->get_clock();

    // Set quality of service profile based on command line options.
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
    qos.reliability(reliability_policy_);

    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
    qos_profile = qos_profile.keep_last(static_cast<size_t>(depth_));
    // qos_profile = qos_profile.reliable();

    auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      process_image(msg, show_image_, this->get_logger());
    };

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic_.c_str());
    sub_ = create_subscription<sensor_msgs::msg::Image>(topic_, qos_profile, callback);
    start_person_add_ = create_publisher<std_msgs::msg::Bool>(topic_addPerson_, 0);

    if(save_video_){
      //the resolution has to match the image-resolution
      video_writer_ = std::make_shared<cv::VideoWriter>("video_cap.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(width_,height_));
    }
  }

//  IMAGE_TOOLS_LOCAL
//  bool help(const std::vector<std::string> args)
//  {
//    if (std::find(args.begin(), args.end(), "--help") != args.end() ||
//        std::find(args.begin(), args.end(), "-h") != args.end())
//    {
//      std::stringstream ss;
//      ss << "Usage: showimage [-h] [--ros-args [-p param:=value] ...]" << std::endl;
//      ss << "Subscribe to an image topic and show the images." << std::endl;
//      ss << "Example: ros2 run image_tools showimage --ros-args -p reliability:=best_effort";
//      ss << std::endl << std::endl;
//      ss << "Options:" << std::endl;
//      ss << "  -h, --help\tDisplay this help message and exit";
//      ss << std::endl << std::endl;
//      ss << "Parameters:" << std::endl;
//      ss << "  reliability\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
//      ss << std::endl;
//      ss << "  history\tHistory QoS setting. Either 'keep_last' (default) or 'keep_all'.";
//      ss << std::endl;
//      ss << "\t\tIf 'keep_last', then up to N samples are stored where N is the depth";
//      ss << std::endl;
//      ss << "  depth\t\tDepth of the publisher queue. Only honored if history QoS is 'keep_last'.";
//      ss << " Default value is 10";
//      ss << std::endl;
//      ss << "  show_image\tShow the image. Either 'true' (default) or 'false'";
//      ss << std::endl;
//      std::cout << ss.str();
//      return true;
//    }
//    return false;
//  }
//
//  IMAGE_TOOLS_LOCAL

  void parse_parameters()
  {
    this->declare_parameter("topic", "");
    topic_ = this->get_parameter("topic").as_string();

    this->declare_parameter("topic_addPerson", "add_person");
    topic_addPerson_ = this->get_parameter("topic_addPerson").as_string();


    // Parse 'reliability' parameter
    rcl_interfaces::msg::ParameterDescriptor reliability_desc;
    reliability_desc.description = "Reliability QoS setting for the image subscription";
    reliability_desc.additional_constraints = "Must be one of: ";
    for (auto entry: name_to_reliability_policy_map) {
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
    history_desc.description = "History QoS setting for the image subscription";
    history_desc.additional_constraints = "Must be one of: ";
    for (auto entry: name_to_history_policy_map) {
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
    depth_ = this->declare_parameter("depth", 10);
    show_image_ = this->declare_parameter("show_image", true);
    save_video_ = this->declare_parameter("save_video", false);
    width_ = this->declare_parameter("width", 640);
    height_ = this->declare_parameter("height", 480);
    video_frames_ = this->declare_parameter("frames", 300);
  }

  /// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
  /**
   * \param[in] encoding A string representing the encoding type.
   * \return The OpenCV encoding type.
   */
//  IMAGE_TOOLS_LOCAL
  int encoding2mat_type(const std::string &encoding)
  {
    if (encoding == "mono8") {
      return CV_8UC1;
    } else if (encoding == "bgr8") {
      return CV_8UC3;
    } else if (encoding == "mono16") {
      return CV_16SC1;
    } else if (encoding == "rgba8") {
      return CV_8UC4;
    } else if (encoding == "bgra8") {
      return CV_8UC4;
    } else if (encoding == "32FC1") {
      return CV_32FC1;
    } else if (encoding == "rgb8") {
      return CV_8UC3;
    } else {
      throw std::runtime_error("Unsupported encoding type");
    }
  }

  /// Convert the ROS Image message to an OpenCV matrix and display it to the user.
  // \param[in] msg The image message to show.
//  IMAGE_TOOLS_LOCAL
  void process_image(
      const sensor_msgs::msg::Image::SharedPtr msg, bool show_image,
      rclcpp::Logger logger)
  {
    RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
    //std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;
    auto duration =
        clock_->now()
        - rclcpp::Time(msg->header.stamp.sec,
                       msg->header.stamp.nanosec,
                       RCL_ROS_TIME);
    long latency = duration.to_chrono<std::chrono::milliseconds>().count(); /*1ms=1 000 000ns*/
    std::cout << "Latency in ms: " << latency << std::endl;

    if(!first_image_){
          start_time_=clock_->now();
          first_image_ = true;
      }
    if (num_test_transfers) {
      avg_fps += latency;
      num_test_transfers--;
      std::cout<< "transfers: "<< num_test_transfers<<std::endl;
    } else {
      if(!fps){
        auto time_diff = clock_->now() - start_time_;
        long lat = time_diff.to_chrono<std::chrono::seconds>().count();
        fps = 2000/lat;
      }
      auto ms = avg_fps / 2000;
      std::cout << "Average latency: " << ms << " ms." << "av fps: " << fps << std::endl;
    }

    

    if (show_image) {
      // Convert to an OpenCV matrix by assigning the data.
      cv::Mat frame(
          msg->height, msg->width, encoding2mat_type(msg->encoding),
          const_cast<unsigned char *>(msg->data.data()), msg->step);

      if (msg->encoding == "rgb8") {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
      }

      cv::Mat cvframe = frame;
//      cv::imwrite("sent_image.jpg", cvframe);
//      cv::waitKey(1);
//       Show the image in a window called "showimage".
      if(show_image_ && !save_video_){

      cv::imshow("showimage", cvframe);
//       Draw the screen and wait for 1 millisecond.
      if (cv::waitKey(1) == 'p') {
        std_msgs::msg::Bool add_person;
        add_person.set__data(true);
        start_person_add_->publish(add_person);
      }
      }
      if (save_video_){
        if(video_frames_>0){
          video_writer_->write(cvframe);
          std::cout<< "frames: "<< video_frames_<< std::endl;
          video_frames_--;
        }else{
          save_video_ = false;
          video_writer_->release();
        }
      }
    }


  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_person_add_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::shared_ptr<cv::VideoWriter> video_writer_;
  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  bool show_image_ = true;
  bool save_video_ ;
  int video_frames_;
  int width_;
  int height_;
  bool first_image_{false};
  rclcpp::Time start_time_;


  std::string topic_;
  std::string topic_addPerson_;
  std::shared_ptr<rclcpp::Clock> clock_;
  long avg_fps = 0;
  int num_test_transfers = 2000;
  long fps{0};
};

int main(int argc, char **argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ShowImage>(options));

  rclcpp::shutdown();
  return 0;
}