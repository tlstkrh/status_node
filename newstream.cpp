image_publisher:
  ros__parameters:
    rtsp_url: "rtsp://192.168.144.25:8554/main.264"
    latency: 100
    protocols: "tcp"
    topic_name: "image_topic"
    fps: 10
    queue_size: 20
    leaky_downstream: true


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher()
      : Node("image_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.144.25:8554/main.264");
        this->declare_parameter<int>("latency", 100);
        this->declare_parameter<std::string>("protocols", "tcp");  // or "udp"
        this->declare_parameter<std::string>("topic_name", "image_topic");
        this->declare_parameter<int>("fps", 10);
        this->declare_parameter<int>("queue_size", 20);
        this->declare_parameter<bool>("leaky_downstream", true);

        // Read parameters
        rtsp_url_ = this->get_parameter("rtsp_url").as_string();
        int latency = this->get_parameter("latency").as_int();
        std::string protocols = this->get_parameter("protocols").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        int fps = this->get_parameter("fps").as_int();
        int queue_size = this->get_parameter("queue_size").as_int();
        bool leaky = this->get_parameter("leaky_downstream").as_bool();

        // GStreamer leaky 옵션 문자열
        // leaky_downstream=true 이면 "leaky=downstream", 아니면 "leaky=no"로 설정
        std::string leaky_option = leaky ? "downstream" : "no";

        // Compose GStreamer pipeline
        //  - 첫 번째 queue: rtspsrc ~ 디코딩 파트 사이
        //  - 두 번째 queue: videoconvert ~ appsink 사이
        //  - appsink: OpenCV(cv::VideoCapture)가 프레임을 가져오는 싱크
        pipeline_ =
            "rtspsrc location=" + rtsp_url_ +
            " latency=" + std::to_string(latency) +
            " protocols=" + protocols +
            " ! queue max-size-buffers=" + std::to_string(queue_size) +
            " leaky=" + leaky_option +
            " ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "queue max-size-buffers=" + std::to_string(queue_size) +
            " leaky=" + leaky_option +
            " ! appsink sync=false drop=true max-buffers=1";

        RCLCPP_INFO(this->get_logger(), "Using pipeline: %s", pipeline_.c_str());

        cap_.open(pipeline_, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream with GStreamer pipeline: %s", pipeline_.c_str());
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to RTSP stream using GStreamer pipeline");
        }

        // QoS 설정
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, qos_profile);

        // FPS에 따른 timer 주기 계산 (fps=10 → 100ms)
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
            // OpenCV Mat → sensor_msgs::Image 변환
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from RTSP stream.");
        }
    }

    cv::VideoCapture cap_;
    std::string pipeline_;
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




#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher()
      : Node("image_publisher"),
        fail_count_(0)
    {
        // Declare parameters
        this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.144.25:8554/main.264");
        this->declare_parameter<int>("latency", 100);
        this->declare_parameter<std::string>("protocols", "tcp");  // or "udp"
        this->declare_parameter<std::string>("topic_name", "image_topic");
        this->declare_parameter<int>("fps", 10);
        this->declare_parameter<int>("queue_size", 20);
        this->declare_parameter<bool>("leaky_downstream", true);

        // Read parameters
        rtsp_url_ = this->get_parameter("rtsp_url").as_string();
        int latency = this->get_parameter("latency").as_int();
        std::string protocols = this->get_parameter("protocols").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        int fps = this->get_parameter("fps").as_int();
        int queue_size = this->get_parameter("queue_size").as_int();
        bool leaky = this->get_parameter("leaky_downstream").as_bool();

        std::string leaky_option = leaky ? "downstream" : "no";

        // Compose GStreamer pipeline
        pipeline_ =
            "rtspsrc location=" + rtsp_url_ +
            " latency=" + std::to_string(latency) +
            " protocols=" + protocols +
            " ! queue max-size-buffers=" + std::to_string(queue_size) +
            " leaky=" + leaky_option +
            " ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "queue max-size-buffers=" + std::to_string(queue_size) +
            " leaky=" + leaky_option +
            " ! appsink sync=false drop=true max-buffers=1";

        RCLCPP_INFO(this->get_logger(), "Using pipeline: %s", pipeline_.c_str());

        // 첫 연결 시도
        if (!openPipeline()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream at startup.");
            return;
        }

        // QoS 설정
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, qos_profile);

        // FPS에 따른 타이머 주기 설정
        int period_ms = static_cast<int>(1000 / fps);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ImagePublisher::timer_callback, this)
        );
    }

private:
    bool openPipeline() {
        if (cap_.isOpened()) {
            cap_.release();
        }
        cap_.open(pipeline_, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Connected to RTSP stream using GStreamer pipeline");
        fail_count_ = 0;
        return true;
    }

    void timer_callback() {
        if (!cap_.isOpened()) {
            // 열려있지 않다면 재시도
            RCLCPP_WARN(this->get_logger(), "Capture not opened. Trying to re-open pipeline...");
            if (!openPipeline()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to re-open pipeline.");
            }
            return;
        }

        cv::Mat frame;
        if (cap_.read(frame)) {
            // 성공적으로 프레임을 읽었다면, fail_count_ 리셋
            fail_count_ = 0;

            // Mat -> sensor_msgs::Image 변환
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        } else {
            // 프레임 읽기 실패 시
            fail_count_++;
            RCLCPP_WARN(this->get_logger(),
                        "Failed to read frame from RTSP stream. fail_count=%d",
                        fail_count_);

            // fail_count_ 임계값 도달 시 재접속 시도
            if (fail_count_ >= 10) {  // 예: 10번 연속 실패하면 재접속
                RCLCPP_WARN(this->get_logger(), "Trying to re-open pipeline...");
                if (!openPipeline()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to re-open pipeline after repeated failures.");
                }
            }
        }
    }

    // 멤버 변수들
    cv::VideoCapture cap_;
    std::string pipeline_;
    std::string rtsp_url_;
    std::string topic_name_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 재연결 로직을 위한 카운터
    int fail_count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
