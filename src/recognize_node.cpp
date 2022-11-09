//
// Created by dave on 27.11.21.
//

#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rtface_pkg/msg/list_id_image.hpp"
#include "rtface_pkg/msg/id_name.hpp"
#include "rtface_pkg/msg/list_id_name.hpp"
#include "rtface_pkg/msg/name_list_image.hpp"
#include <FaceRecognizer.h>
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

class FaceRecognitionSubPub : public rclcpp::Node
{
public:
  FaceRecognitionSubPub()
      : Node("RecognitionNode")
  {
    // - - - Parameter Declarations - - -
    this->declare_parameter("topic_sub", "face_id");
    this->declare_parameter("topic_pub", "rec_id");
    this->declare_parameter("topic_newPerson_sub", "new_person");
    this->declare_parameter("similarity_th", .3);

    this->declare_parameter("eng_embeddings_path","");
    this->declare_parameter("eng_rebuild", false);
    this->declare_parameter("eng_save", true);

    this->declare_parameter("eng_feature_onnx_path","");
    this->declare_parameter("eng_feature_save_path", "./ffeature.rt");
    this->declare_parameter("eng_use_dla", -1);
    this->declare_parameter("eng_fp_16", false);
    this->declare_parameter("eng_attention", false);

    topic_sub = this->get_parameter("topic_sub").as_string();
    topic_pub = this->get_parameter("topic_pub").as_string();
    topic_newPerson_sub = this->get_parameter("topic_newPerson_sub").as_string();
    similarity_th_ = this->get_parameter("similarity_th").as_double();

    embeddings_path = this->get_parameter("eng_embeddings_path").as_string();
    eng_rebuild = this->get_parameter("eng_rebuild").as_bool();
    eng_save = this->get_parameter("eng_save").as_bool();

    eng_feature_onnx_path = this->get_parameter("eng_feature_onnx_path").as_string();
    eng_feature_save_path = this->get_parameter("eng_feature_save_path").as_string();
    eng_use_dla_ = this->get_parameter("eng_use_dla").as_int();
    eng_fp_16_ = this->get_parameter("eng_fp_16").as_bool();
    eng_attention_ = this->get_parameter("eng_attention").as_bool();



	rclcpp::QoS m_qos_profile = rclcpp::SensorDataQoS();
	m_qos_profile = m_qos_profile.keep_last(5);
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	
	rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();
	m_qos_profile_sysdef = m_qos_profile_sysdef.keep_last(5);
	m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);



    IdName_publisher_ = this->create_publisher<rtface_pkg::msg::ListIdName>(topic_pub, m_qos_profile);
    fr_ = std::make_shared<face_RT::FaceRecognizer>(embeddings_path, eng_rebuild, eng_save, eng_feature_onnx_path, eng_feature_save_path, eng_use_dla_, eng_fp_16_, eng_attention_);
    DEBUG_MSG("Rec init success!");

    // - - - Callback for identification - - -
    auto publish_msg = [this](const rtface_pkg::msg::ListIdImage::UniquePtr msg) -> void {
      listIdName.list_id_name.clear();
      // === fill faceMap ===
      for (auto &&id_image: msg->list_id_image) {

        cv::Mat face_image = cv_bridge::toCvCopy(id_image.image_al, "bgr8")->image;
//        DEBUG_MSG("Rec_siam_head: "+ name_sim.first +" - " + std::to_string(name_sim.second));
        auto name_sim_cos = fr_->get_recognized_person_cosine(face_image);
        DEBUG_MSG("Rec_cos: "+ name_sim_cos.first +" - " + std::to_string(name_sim_cos.second));
        if (name_sim_cos.second > similarity_th_){
          id_name.set__id(id_image.id);
          id_name.set__name(name_sim_cos.first);
          id_name.set__similarity(name_sim_cos.second);
          listIdName.list_id_name.push_back(id_name);
        }
      }
//      DEBUG_MSG("pblish_msg init success");

      // === send identifications back ===
      if(!listIdName.list_id_name.empty())

        try{
          IdName_publisher_->publish(listIdName);
        }
	      catch (...) {
		      RCLCPP_INFO(this->get_logger(), "IdName_publisher_: hmm publishing dets has failed!! ");
	      }
    };

    // - - - callback for saving a new person to the embedding db - - -
    auto save_person = [this](const rtface_pkg::msg::NameListImage::UniquePtr msg) -> void {
      auto rec_msg = msg.get();
      v_new_person_imgs.clear();
      for (auto &img: rec_msg->images){
        cv::Mat face_img = cv_bridge::toCvCopy(img, "bgr8")->image;
        cv::resize(face_img, face_img, cv::Size(112, 112), cv::INTER_LINEAR);
        v_new_person_imgs.push_back(face_img);
      }
      fr_->add_person(rec_msg->name, v_new_person_imgs);
    };

    subscription_ = this->create_subscription<rtface_pkg::msg::ListIdImage>(topic_sub, m_qos_profile_sysdef, publish_msg);
    new_person_subscription_ = this->create_subscription<rtface_pkg::msg::NameListImage>(topic_newPerson_sub, m_qos_profile_sysdef, save_person);

    DEBUG_MSG("sub success");
  }

private:
  rclcpp::Subscription<rtface_pkg::msg::NameListImage>::SharedPtr new_person_subscription_;
  rclcpp::Subscription<rtface_pkg::msg::ListIdImage>::SharedPtr subscription_;
  rclcpp::Publisher<rtface_pkg::msg::ListIdName>::SharedPtr IdName_publisher_;
  std::shared_ptr<face_RT::FaceRecognizer> fr_;

  rtface_pkg::msg::ListIdName listIdName{};
  rtface_pkg::msg::IdName id_name{};
  std::vector<std::pair<int, cv::Mat>> pair_list{};
  std::vector<cv::Mat> v_new_person_imgs;

  // - - - Parameter - - -
  /* For the node */
  std::string topic_sub;
  std::string topic_pub;
  std::string topic_newPerson_sub;
  double similarity_th_{0.3};

  /* For the node's engine */
  std::string embeddings_path;
  bool eng_rebuild;
  bool eng_save;

  std::string eng_feature_onnx_path;
  std::string eng_feature_save_path;
  long eng_use_dla_;
  bool eng_fp_16_;
  bool eng_attention_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto rec_node = std::make_shared<FaceRecognitionSubPub>();
  rclcpp::spin(rec_node);
  rclcpp::shutdown();
  return 0;
}