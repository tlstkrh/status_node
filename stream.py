    #include <rclcpp/rclcpp.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <cv_bridge/cv_bridge.h>
    #include <opencv2/opencv.hpp>
    #include <string>

    class ImagePublisher : public rclcpp::Node {
    public:
        ImagePublisher() : Node("image_publisher") {
            this->declare_parameter<std::string>("rtsp_url", "normal_url");
            this->declare_parameter<int>("latency", 0);
            this->declare_parameter<std::string>("protocols", "udp");
            this->declare_parameter<std::string>("topic_name", "image_topic");
            this->declare_parameter<int>("fps", 10); 

            // Read ymal file parameters
            rtsp_url_ = this->get_parameter("rtsp_url").as_string();
            int latency = this->get_parameter("latency").as_int();
            std::string protocols = this->get_parameter("protocols").as_string();
            topic_name_ = this->get_parameter("topic_name").as_string();
            int fps = this->get_parameter("fps").as_int();

            //create  GStreamer pipeline
            std::string pipeline = "rtspsrc location=" + rtsp_url_ +
              " latency=" + std::to_string(latency) +
              " protocols=" + protocols +
              " ! queue ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGR ! videoconvert ! appsink sync=false";


            cap_.open(pipeline, cv::CAP_GSTREAMER);
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream with GStreamer pipeline: %s", pipeline.c_str());
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "Connected to RTSP stream using GStreamer pipeline");
            }

            // QoS set 
            rclcpp::QoS qos_profile(10);
            qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

            publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, qos_profile);

            // Calculate cycles use FPS (if fps == 10 then 100ms)
            int period_ms = static_cast<int>(1000 / fps);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&ImagePublisher::timer_callback, this)
            );
        }

    private:
        void timer_callback() {
            cv::Mat frame;
            if (cap_.read(frame)) {
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                publisher_->publish(*msg);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to read frame from RTSP stream.");
            }
        }

        cv::VideoCapture cap_;
        std::string rtsp_url_;
        std::string topic_name_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

    int main(int argc, char** argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<ImagePublisher>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
