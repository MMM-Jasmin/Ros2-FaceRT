#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/clock.hpp"
#include "boost/program_options.hpp"
#include <thread>
// native ros interfaces and libs
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
// custom interfaces
#include "rtface_pkg/msg/list_id_image.hpp"
// opencv and bridge
#include "cv_bridge/cv_bridge.h"

#include "rtface_pkg/msg/list_id_name.hpp"
#include "rtface_pkg/msg/list_id_mask.hpp"
#include "rtface_pkg/msg/list_id_emotion.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <utility>



#include "RetinaFaceDetector.h"
#include "Utils.h"
#include "Timer.h"

// #define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

const double ONE_SECOND            = 1000.0; // One second in milliseconds

namespace bpo = boost::program_options;
using namespace std::chrono_literals;

class DetectFaces : public rclcpp::Node
{
    public:
    explicit DetectFaces(const rclcpp::NodeOptions &options) : Node("Detect_node", options){
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        // Do not execute if a --help option was provided
        //    if (help(options.arguments())) {
        //      exit(0);
        //    }
        parse_parameters();
        initialize();
    }

    private:
        Timer m_timer;          // Timer used to measure the time required for one iteration
	    double m_elapsedTime;   // Sum of the elapsed time, used to check if one second has passed
        float m_maxFPS = 30.0;
        uint64_t m_frameCnt = 0;
        std::string  m_last_json_msg = "";

        /**
        * Define and parse node-parameters.
        */
        void parse_parameters(){

            // - - - Parameter Declarations - - -
            this->declare_parameter("cam_topic", "camera/color/image_raw");
            this->declare_parameter("mask_topic", "mask_eval");
            this->declare_parameter("rec_topic", "rec_id");
            this->declare_parameter("emotion_topic", "emotion_id");
            this->declare_parameter("faceId_topic_name", "face_id");
            this->declare_parameter("detection_topic", "detections");
            this->declare_parameter("face_pub_size", 224);
            this->declare_parameter("fp_duration", 500);
            this->declare_parameter("eng_weights_path","");
            this->declare_parameter("eng_save_path", "./retina_detection.rt");
            this->declare_parameter("eng_use_dla", -1);
            this->declare_parameter("eng_fp_16", false);

            json_topic_ = this->declare_parameter("json_topic", "json_out");
            fps_topic_ = this->declare_parameter("fps_topic", "json_out");
            m_maxFPS = this->declare_parameter("max_fps", 30.0);
            cam_topic_ = this->get_parameter("cam_topic").as_string();
            mask_topic_ = this->get_parameter("mask_topic").as_string();
            rec_topic_ = this->get_parameter("rec_topic").as_string();
            emotion_topic_ = this->get_parameter("emotion_topic").as_string();
            faceId_topic_name_ = this->get_parameter("faceId_topic_name").as_string();
            face_publish_size_ = this->get_parameter("face_pub_size").as_int();
            fp_timer_duration_ = chrono::milliseconds(this->get_parameter("fp_duration").as_int());
            eng_weights_path_ = this->get_parameter("eng_weights_path").as_string();
            eng_save_path_ = this->get_parameter("eng_save_path").as_string();
            eng_use_dla_ = this->get_parameter("eng_use_dla").as_int();
            eng_fp_16_ = this->get_parameter("eng_fp_16").as_bool();
        }

        /**
        * initialize Node
        */
        void initialize(){
            m_elapsedTime = 0;
            m_timer.Start();

            fd_ = std::make_shared<face_RT::RetinaFaceDetector>(eng_weights_path_, eng_save_path_, eng_use_dla_, eng_fp_16_);

            RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", cam_topic_.c_str());
            // sub_ = create_subscription<sensor_msgs::msg::Image>(cam_topic_, qos, callback);

            rclcpp::QoS m_qos_profile = rclcpp::SensorDataQoS();
            m_qos_profile = m_qos_profile.keep_last(5);
            m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	
            rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();
            m_qos_profile_sysdef = m_qos_profile_sysdef.keep_last(5);
            m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

            rclcpp::CallbackGroup::SharedPtr my_callback_cam_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::CallbackGroup::SharedPtr my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions cam_options;
            rclcpp::SubscriptionOptions options;
            cam_options.callback_group = my_callback_cam_group;
            options.callback_group = my_callback_group;


            //my_subscription = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(), callback, options);

            cam_sub_ = create_subscription<sensor_msgs::msg::Image>(cam_topic_, m_qos_profile, std::bind(&DetectFaces::cam_callback, this, std::placeholders::_1), cam_options);

            // - - - init publisher - - -
            json_publisher_ = create_publisher<std_msgs::msg::String>(json_topic_, m_qos_profile_sysdef);
            m_fps_publisher = create_publisher<std_msgs::msg::String>(fps_topic_, m_qos_profile_sysdef);
            l_id_face_publisher_ = create_publisher<rtface_pkg::msg::ListIdImage>(faceId_topic_name_, m_qos_profile_sysdef);

            update_id_face_timer_ = this->create_wall_timer(fp_timer_duration_, std::bind(&DetectFaces::pub_ids_faces_callback, this));


            //ubscription_ = this->create_subscription<rtface_pkg::msg::ListIdImage>(topic_sub, m_qos_profile_sysdef, publish_msg);
            // - - - subscriber for inference nodes - - -
            std::cerr << "test: " << rec_topic_ << std::endl;
            //rec_sub_ = this->create_subscription<rtface_pkg::msg::ListIdName>(rec_topic_, m_qos_profile_sysdef, std::bind(&DetectFaces::rec_callbaasdagfck, this, std::placeholders::_1));
        
            rec_sub_   = this->create_subscription<rtface_pkg::msg::ListIdName>(rec_topic_, m_qos_profile, std::bind(&DetectFaces::rec_callback, this, std::placeholders::_1), options);
            std::cerr << "test2 " << rec_topic_ << std::endl;
            mask_sub_ = this->create_subscription<rtface_pkg::msg::ListIdMask>(mask_topic_, m_qos_profile, std::bind(&DetectFaces::mask_callback, this, std::placeholders::_1), options);
            emotion_sub_ = this->create_subscription<rtface_pkg::msg::ListIdEmotion>(emotion_topic_, m_qos_profile, std::bind(&DetectFaces::emotion_callback, this, std::placeholders::_1), options);
        }

        void cam_callback (const sensor_msgs::msg::Image::SharedPtr msg) {
            DEBUG_MSG("Trigger image Callback");
            process_image(msg, this->get_logger());
        };


        void pub_ids_faces_callback () {
            auto tracked_faces = fd_->getTrackedFaces(face_publish_size_);
            l_id_face_msg_.list_id_image.clear();

            for (auto &tf: tracked_faces) {
                auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", tf.faceCrop).toImageMsg();
                rtface_pkg::msg::IdImage id_image{};
                id_image.image = *img_msg;
                id_image.id = tf.id;

                //aligned face for arc-face-recognition
                if (!tf.faceCrop_al.empty()) {
                    auto imgcrop_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", tf.faceCrop_al).toImageMsg();
                    id_image.image_al = *imgcrop_msg;
                } else {
                    // use unaligned image, if alignment did not succeed
                    id_image.image_al = *img_msg;
                }

                l_id_face_msg_.list_id_image.push_back(id_image);
            }
            try {
                l_id_face_publisher_->publish(l_id_face_msg_);
            } catch (...) {
                DEBUG_MSG("Publish face - crop failed");
            }
        };

        void CheckFPS(uint64_t* pFrameCnt){
            m_timer.Stop();

		        double minFrameTime = 1000.0 / m_maxFPS;
		        double itrTime      = m_timer.GetElapsedTimeInMilliSec();
		        double fps;

		        //if (m_timer.GetElapsedTimeInMilliSec() < minFrameTime){
			    //      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int32_t>(std::round(minFrameTime - itrTime))));
			    //      m_elapsedTime += minFrameTime;
		        //} else
		            m_elapsedTime += itrTime;

		        fps = 1000 / (m_elapsedTime / (*pFrameCnt));

		        if (m_elapsedTime >= ONE_SECOND){
			          PrintFPS(fps, itrTime);

			          *pFrameCnt    = 0;
			          m_elapsedTime = 0;
		        }

		        m_timer.Start();
            }

        void PrintFPS(const float fps, const float itrTime){
		
	          std::stringstream str("");

	          if (fps == 0.0f)
			          str << string_format("{\"FPS\": 0.0}");
	          else
		            str << string_format("{\"FPS\": %.2f, \"lastCurrMSec\": %.2f, \"maxFPS\": %.2f }",fps, itrTime, m_maxFPS);

	          auto message = std_msgs::msg::String();
	          message.data = str.str();
            try{
	              m_fps_publisher->publish(message);
            } catch (...) {
                RCLCPP_INFO(this->get_logger(), "m_fps_publisher: hmm publishing dets has failed!! ");
            }
		
		        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        }


        /**
        * Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
        * \param[in] encoding A string representing the encoding type.
        * \return The OpenCV encoding type.
        */
        //  IMAGE_TOOLS_LOCAL
        int encoding2mat_type(const std::string &encoding) {
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

        /**
        * Process the camera - image.
        * @param msg Ponter of image-message.
        * @param show_image
        * @param logger
        */
        void process_image(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger logger) {
            // Convert to an OpenCV matrix by assigning the data.
            cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()), msg->step);

            if (msg->encoding == "rgb8") {
                cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
            }  

            fd_->detectFrame(frame);

            //publish json
            std::string tracks_json = fd_->get_all_tracks_as_json();
            if(m_last_json_msg.compare(tracks_json) != 0){
      
                std_msgs::msg::String json_msg;
                json_msg.set__data(tracks_json);
                try {
                    json_publisher_->publish(json_msg);
                } catch (...) {
                    DEBUG_MSG("Publish JSON failed");
                }
                m_last_json_msg = tracks_json;
            }
            m_frameCnt++;
            CheckFPS(&m_frameCnt);

        }

        static std::string mat_type2encoding(int mat_type) {
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

        void convert_frame_to_message(const cv::Mat & frame, sensor_msgs::msg::Image & out_msg, sensor_msgs::msg::Image & in_msg) {
            // copy cv information into ros message
            out_msg.height = frame.rows;
            out_msg.width = frame.cols;
            out_msg.encoding = mat_type2encoding(frame.type());
            out_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            size_t size = frame.step * frame.rows;
            out_msg.data.resize(size);
            memcpy(&out_msg.data[0], frame.data, size);
            //    frame_id_++;
            out_msg.header = in_msg.header;
            //    msg.header.stamp = rclcpp::Clock().now();
        }

        /**
        * Callback to process messages from mask inference and pass it to the tracker.
        * @param msg List of ids and mask states.
        */
        void mask_callback(const rtface_pkg::msg::ListIdMask::SharedPtr msg) {
            map<int, bool> id_mask_map{};
            for (auto &id_mask: msg->list_id_mask) {
                id_mask_map.insert({id_mask.id, id_mask.mask});
            }
            DEBUG_MSG("pre Aassign mask");
            fd_->assignMasks(id_mask_map);
            DEBUG_MSG("post Aassign mask");
        }

        /**
        * Callback to process messages from emotion inference and pass it to the tracker.
        * @param msg List of ids and emotion-numbers.
        */
        void emotion_callback(const rtface_pkg::msg::ListIdEmotion::SharedPtr msg) {
            map<int, int> id_emotion_map{};
            for (auto &id_emotion: msg->list_id_emotion) {
                id_emotion_map.insert({id_emotion.id, id_emotion.emotion});
                DEBUG_MSG("ID_name at detector: " << to_string(id_emotion.id) << ", " << id_emotion.emotion);
            }
            fd_->assignEmotions(id_emotion_map);
        }

        /**
        * Callback to process messages from recognition inference and pass it to the tracker.
        * @param msg List of ids and names.
        */
        void rec_callback(const rtface_pkg::msg::ListIdName::SharedPtr msg) {
            map<int, string> id_name_map{};
             for (auto &id_name_prob: msg->list_id_name) {
              id_name_map.insert({id_name_prob.id, id_name_prob.name});
             }
            fd_->assignRecognitions(id_name_map);
        }

        double fps{0};

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;

        std::string cam_topic_;
        std::shared_ptr<face_RT::RetinaFaceDetector> fd_;

        // - - - publish faces with ids for other nodes - - -
        rclcpp::Publisher<rtface_pkg::msg::ListIdImage>::SharedPtr l_id_face_publisher_{};
        rclcpp::TimerBase::SharedPtr update_id_face_timer_;
        rtface_pkg::msg::ListIdImage l_id_face_msg_{};
        int face_publish_size_;
        chrono::duration<long, ratio<1, 1000>> fp_timer_duration_;
        string faceId_topic_name_;

        // - - - json, fps, recognition, masks, emotions - - -
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_publisher_{};
        string json_topic_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_fps_publisher;
        string fps_topic_;
        rclcpp::Subscription<rtface_pkg::msg::ListIdName>::SharedPtr rec_sub_{};
        std::string rec_topic_;
        rclcpp::Subscription<rtface_pkg::msg::ListIdMask>::SharedPtr mask_sub_{};
        std::string mask_topic_;
        rclcpp::Subscription<rtface_pkg::msg::ListIdEmotion>::SharedPtr emotion_sub_{};
        std::string emotion_topic_;

        /* For the node's engine */
        bool eng_load_;
        std::string eng_weights_path_;
        std::string eng_save_path_;
        int eng_use_dla_;
        bool eng_fp_16_;
};


int main(int argc, char **argv) {
    (void) argc;
    (void) argv;
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor multi_executor(rclcpp::ExecutorOptions(), 2, false);
    rclcpp::NodeOptions options;

    auto node1 = make_shared<DetectFaces>(options);
    multi_executor.add_node(node1);

    multi_executor.spin();
    rclcpp::shutdown();
    return 0;
}
