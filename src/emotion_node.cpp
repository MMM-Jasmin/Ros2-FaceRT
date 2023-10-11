//
// Created by dave on 27.11.21.
//

#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rtface_pkg/msg/list_id_image.hpp"
#include "rtface_pkg/msg/id_emotion.hpp"
#include "rtface_pkg/msg/list_id_emotion.hpp"
#include "EmotionPredictionRT.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "boost/program_options.hpp"


#define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif
namespace bpo = boost::program_options;


class EmotionSubPub : public rclcpp::Node {
public:
  EmotionSubPub() : Node("emotion_node")  {
    parse_parameters();
    IdEmotion_publisher_ = this->create_publisher<rtface_pkg::msg::ListIdEmotion>(topic_pub, 1);
    RCLCPP_INFO(this->get_logger(), std::string("Publishing to " + topic_pub).c_str());

    mp_ = std::make_shared<face_RT::EmotionPredictionRT>(eng_save, eng_onnx_path, eng_save_path, eng_use_dla_, eng_fp_16_);
    RCLCPP_INFO(this->get_logger(), "Emotion engine is running.");

    DEBUG_MSG("Rec init success!");

    // - - - Callback function - - -
    auto pub_id_emo_msg = [this](const rtface_pkg::msg::ListIdImage::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "Got new faces");
      faceMap.clear();
      // - - - fill faceMap - - -
      for (auto &id_image: msg->list_id_image) {
        cv::Mat face_image = cv_bridge::toCvCopy(id_image.image, "bgr8")->image;
        faceMap.insert({id_image.id, face_image});
      }

      // - - - recognize emotion - - -
      listIdEmotion.list_id_emotion.clear();
      for (auto &pair: faceMap) {
        int res{-1};
        mp_->infer(pair.second, res);
        id_emotion.set__id(pair.first);
        id_emotion.set__emotion(res);
        listIdEmotion.list_id_emotion.push_back(id_emotion);
        DEBUG_MSG( "ID_name: " << std::to_string(id_emotion.id) << ", " << id_emotion.emotion);
      }
      //DEBUG_MSG("pblish_msg init success");

      // - - - send emotions back - - -
      IdEmotion_publisher_->publish(listIdEmotion);
    };

    RCLCPP_INFO(this->get_logger(), "trying to sub");
    subscription_ = this->create_subscription<rtface_pkg::msg::ListIdImage>(topic_sub, 1, pub_id_emo_msg);
    DEBUG_MSG("sub success");
    RCLCPP_INFO(this->get_logger(), "Sub success");

  }

private:
  void parse_parameters(){
    // - - - Parameter Declarations - - -
    this->declare_parameter("topic_sub", "face_id");
    this->declare_parameter("topic_pub", "emotion_id");

    this->declare_parameter("eng_save",true);
    this->declare_parameter("eng_onnx_path", "");
    this->declare_parameter("eng_save_path", "./emotion_predict.rt");
    this->declare_parameter("eng_use_dla", -1);
    this->declare_parameter("eng_fp_16", false);

    topic_sub = this->get_parameter("topic_sub").as_string();
    topic_pub = this->get_parameter("topic_pub").as_string();

    eng_save = this->get_parameter("eng_save").as_bool();
    eng_onnx_path = this->get_parameter("eng_onnx_path").as_string();
    eng_save_path = this->get_parameter("eng_save_path").as_string();
    eng_use_dla_ =this->get_parameter("eng_use_dla").as_int();
    eng_fp_16_ = this->get_parameter("eng_fp_16").as_bool();
  }

  rclcpp::Subscription<rtface_pkg::msg::ListIdImage>::SharedPtr subscription_;
  rclcpp::Publisher<rtface_pkg::msg::ListIdEmotion>::SharedPtr IdEmotion_publisher_;
  std::shared_ptr<face_RT::EmotionPredictionRT> mp_;
  rtface_pkg::msg::ListIdEmotion listIdEmotion{};
  rtface_pkg::msg::IdEmotion id_emotion{};
  std::map<int, cv::Mat> faceMap{};

  // - - - Parameter - - -
  /* For the node */
  std::string topic_sub;
  std::string topic_pub;

  /* For the node's engine */
  bool eng_save;
  std::string eng_onnx_path;
  std::string eng_save_path;
  int eng_use_dla_;
  bool eng_fp_16_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto emotion_node = std::make_shared<EmotionSubPub>();
  rclcpp::spin(emotion_node);
  std::cout<<"shutting down the node"<<std::endl;
  rclcpp::shutdown();
  return 0;
}
