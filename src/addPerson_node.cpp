//
// Created by dave on 27.11.21.
//

#include <memory>
#include <map>
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rtface_pkg/msg/list_id_image.hpp"
#include "rtface_pkg/msg/id_name.hpp"
#include "rtface_pkg/msg/list_id_name.hpp"
#include "rtface_pkg/msg/name_list_image.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>



#define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif
// > --- https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

std::string exec(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}
// < --- https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po

class FaceRecAddIdSubPub : public rclcpp::Node
{
public:
  FaceRecAddIdSubPub()
      : Node("Add_person_to_rec")
  {
    // - - - Parameter Declarations - - -
    this->declare_parameter("topic_sub", "face_id");
    this->declare_parameter("topic_sub_addPerson", "add_person");
    this->declare_parameter("topic_pub", "rec_id");


    topic_sub = this->get_parameter("topic_sub").as_string();
    topic_sub_addPerson = this->get_parameter("topic_sub_addPerson").as_string();
    topic_pub = this->get_parameter("topic_pub").as_string();


    NameListImage_publisher_ = this->create_publisher<rtface_pkg::msg::NameListImage>(
        topic_pub, 1);
    DEBUG_MSG("Rec init success!");

    auto add_face = [this](const rtface_pkg::msg::ListIdImage::UniquePtr msg) -> void {
      auto last_message = msg->list_id_image;
      sensor_msgs::msg::Image image_msg{};
      DEBUG_MSG("Callback triggered!");
      auto key = cv::waitKey(4000);

      if(next_add_face){
          next_add_face = false;
          cv::namedWindow("Is this you? - press y/n");

          for (auto &id_image : last_message) {
            cv::Mat face_image = cv_bridge::toCvCopy(id_image.image_al, "bgr8")->image;
            cv::imshow("Is this you? - press y/n",face_image);
            if( cv::waitKey(0) == 'y'){
              std::string new_name = exec(R"(zenity  --title  "Enter your name" --entry --text "Enter your name here.")");
              //std::replace(new_name.begin(), new_name.end(), '\n', "");
              //std::replace(new_name.begin(), new_name.end(), ' ', "");
              new_name.erase(remove(new_name.begin(), new_name.end(),  '\n'), new_name.end());
              new_name.erase(remove(new_name.begin(), new_name.end(),  ' '), new_name.end());

              nameListImage.name = new_name;
              nameListImage.images.push_back(id_image.image);
              NameListImage_publisher_->publish(nameListImage);
              break;
            }
            //next_add_face = true;
          }
          cv::destroyWindow("Is this you? - press y/n");
      }
    };

    auto start_add_person = [this](const std_msgs::msg::Bool::UniquePtr msg) -> void {
      next_add_face = msg->data;
    };


    subscription_ = this->create_subscription<rtface_pkg::msg::ListIdImage>(
        topic_sub, 1, add_face);
    save_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_sub_addPerson, 1, start_add_person);
    DEBUG_MSG("sub success");
  }

private:
  bool next_add_face = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_subscription_;
  rclcpp::Subscription<rtface_pkg::msg::ListIdImage>::SharedPtr subscription_;
  rclcpp::Publisher<rtface_pkg::msg::NameListImage>::SharedPtr NameListImage_publisher_;
  rtface_pkg::msg::NameListImage nameListImage{};
  rtface_pkg::msg::IdName id_name{};
  std::map<int, cv::Mat> faceMap{};

  // - - - Parameter - - -
  /* For the node */
  std::string topic_sub;
  std::string topic_sub_addPerson;
  std::string topic_pub;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto rec_node = std::make_shared<FaceRecAddIdSubPub>();
  rclcpp::spin(rec_node);
  rclcpp::shutdown();
  return 0;
}