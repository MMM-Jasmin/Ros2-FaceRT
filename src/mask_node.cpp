//
// Created by dave on 27.11.21.
//
#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rtface_pkg/msg/list_id_image.hpp"
#include "rtface_pkg/msg/list_id_mask.hpp"
#include "MaskPredictionRT.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "boost/program_options.hpp"


//#define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif
namespace bpo = boost::program_options;

class MaskSubPub : public rclcpp::Node
{

public:

  MaskSubPub()
      : Node("MaskPredictionNode")
  {
    // - - - Parameter Declarations - - -
    this->declare_parameter("topic_sub", "face_id");
    this->declare_parameter("topic_pub", "mask_eval");

    this->declare_parameter("eng_save",true);
    this->declare_parameter("eng_onnx_path", "");
    this->declare_parameter("eng_save_path", "./mask_precdict.rt");
    this->declare_parameter("eng_use_dla", -1);
    this->declare_parameter("eng_fp_16", false);

    topic_sub = this->get_parameter("topic_sub").as_string();
    topic_pub = this->get_parameter("topic_pub").as_string();

    eng_save = this->get_parameter("eng_save").as_bool();
    eng_onnx_path = this->get_parameter("eng_onnx_path").as_string();
    eng_save_path = this->get_parameter("eng_save_path").as_string();
    eng_use_dla_ =this->get_parameter("eng_use_dla").as_int();
    eng_fp_16_ = this->get_parameter("eng_fp_16").as_bool();

    IdMask_publisher_ = this->create_publisher<rtface_pkg::msg::ListIdMask>(
        topic_pub, 10);
    mp_ = std::make_shared<face_RT::MaskPredictionRT>(eng_save,
                                                      eng_onnx_path,
                                                      eng_save_path,
                                                      eng_use_dla_,
                                                      eng_fp_16_);
    DEBUG_MSG("Rec init success!");

    auto publish_msg = [this](const rtface_pkg::msg::ListIdImage::UniquePtr msg) -> void {
      faceMap.clear();
      // - - - fill faceMap - - -
      for (auto &id_image: msg->list_id_image) {
        cv::Mat face_image = cv_bridge::toCvCopy(id_image.image, "bgr8")->image;
        faceMap.insert({id_image.id, face_image});
      }

      // - - - evaluate mask status - - -
      listIdMask.list_id_mask.clear();
      for (auto &&pair: faceMap) {
        bool res{false};
        mp_->infer(pair.second, res);
        id_mask.set__id(pair.first);
        id_mask.set__mask(res);
        listIdMask.list_id_mask.push_back(id_mask);
        DEBUG_MSG("ID_name: " << std::to_string(id_mask.id) << ", " << id_mask.mask);
      }
      DEBUG_MSG("pblish_msg init success");

      // - - - send mask states back - - -
      IdMask_publisher_->publish(listIdMask);
    };
    subscription_ = this->create_subscription<rtface_pkg::msg::ListIdImage>(
        topic_sub, 1, publish_msg);
    DEBUG_MSG("sub success");
  }

private:
  rclcpp::Subscription<rtface_pkg::msg::ListIdImage>::SharedPtr subscription_;
  rclcpp::Publisher<rtface_pkg::msg::ListIdMask>::SharedPtr IdMask_publisher_;
  std::shared_ptr<face_RT::MaskPredictionRT> mp_;
  rtface_pkg::msg::ListIdMask listIdMask{};
  rtface_pkg::msg::IdMask id_mask{};
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

  auto mask_node = std::make_shared<MaskSubPub>();
  rclcpp::spin(mask_node);
  rclcpp::shutdown();
  return 0;
}
