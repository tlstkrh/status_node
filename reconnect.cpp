#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher()
      : Node("image_publisher"),
        fail_count_(0),
        reconnect_requested_(false)
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
            " ! appsink sync=false";


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

        // 비동기 재연결 타이머 (1초 간격으로 재접속 요청이 있으면 시도)
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ImagePublisher::reconnect_callback, this)
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
        // 메인 타이머 콜백은 빠르게 반환해야 함.
        if (!cap_.isOpened()) {
            // 열려있지 않다면 프레임을 읽지 않고 반환
            return;
        }

        cv::Mat frame;
        if (cap_.read(frame)) {
            // 성공적으로 프레임을 읽었다면
            fail_count_ = 0;
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        } else {
            fail_count_++;
            RCLCPP_WARN(this->get_logger(),
                        "Failed to read frame from RTSP stream. fail_count=%d",
                        fail_count_);

            // 일정 횟수 이상 실패하면 재접속 요청
            if (fail_count_ >= 10) {
                fail_count_ = 0;  // 카운트 리셋
                RCLCPP_WARN(this->get_logger(), "Requesting to re-open pipeline...");
                reconnect_requested_ = true;  
            }
        }
    }

    void reconnect_callback() {
        // 1초 간격으로 돌아가는 타이머
        // reconnect_requested_가 true이고, 아직 노드가 살아있는 경우에만 재접속 시도
        if (reconnect_requested_ && rclcpp::ok()) {
            reconnect_requested_ = false;  // 플래그 리셋
            RCLCPP_WARN(this->get_logger(), "Re-opening pipeline now...");
            if (!openPipeline()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to re-open pipeline (reconnect_callback).");
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
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    int fail_count_;
    std::atomic<bool> reconnect_requested_; // 재연결 요청 플래그
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    // 여기까지 왔다는 것은 ^C 신호 등으로 종료를 시작했다는 의미

    rclcpp::shutdown();
    return 0;
}

