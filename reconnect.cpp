#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher()
    : Node("image_publisher"),
      fail_count_(0),
      reconnect_requested_(false),
      stop_reading_(false)
  {
    // 1. 파라미터 선언 및 읽기
    this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.144.25:8554/main.264");
    this->declare_parameter<int>("latency", 100);
    this->declare_parameter<std::string>("protocols", "tcp");  // or "udp"
    this->declare_parameter<std::string>("topic_name", "image_topic");
    this->declare_parameter<int>("fps", 10);              // Publish할 FPS
    this->declare_parameter<int>("queue_size", 20);
    this->declare_parameter<bool>("leaky_downstream", true);

    rtsp_url_ = this->get_parameter("rtsp_url").as_string();
    int latency = this->get_parameter("latency").as_int();
    std::string protocols = this->get_parameter("protocols").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    int publish_fps = this->get_parameter("fps").as_int();
    int queue_size = this->get_parameter("queue_size").as_int();
    bool leaky = this->get_parameter("leaky_downstream").as_bool();
    std::string leaky_option = leaky ? "downstream" : "no";

    // 2. GStreamer 파이프라인 구성
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

    // 3. 파이프라인 열기
    if (!openPipeline()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream at startup.");
      return;
    }

    // 4. ROS Publisher 생성 (QoS: Best Effort)
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, qos_profile);

    // 5. 30fps로 읽는 별도 쓰레드 시작 (스트림 계속 읽기)
    reading_thread_ = std::thread(&ImagePublisher::readLoop, this);

    // 6. Publish 타이머: publish_fps(예: 10fps) 주기로 실행
    int period_ms = static_cast<int>(1000 / publish_fps);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ImagePublisher::timer_callback, this)
    );

    // 7. 재연결 타이머 (1초 간격, 필요시 파이프라인 재시작)
    reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ImagePublisher::reconnect_callback, this)
    );
  }

  ~ImagePublisher() {
    stop_reading_ = true;
    if (reading_thread_.joinable()) {
      reading_thread_.join();
    }
  }

private:
  // RTSP 파이프라인을 여는 함수
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

  // 30fps로 스트림을 계속 읽는 별도 쓰레드 루프
  void readLoop() {
    while (rclcpp::ok() && !stop_reading_) {
      if (!cap_.isOpened()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      cv::Mat frame;
      if (cap_.read(frame)) {
        // 최신 프레임을 mutex로 보호하며 업데이트
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = frame.clone();
        fail_count_ = 0;  // 정상 읽었으므로 리셋
      } else {
        fail_count_++;
        RCLCPP_WARN(this->get_logger(), "Failed to read frame in readLoop, fail_count=%d", fail_count_);
        if (fail_count_ >= 10) {
          RCLCPP_WARN(this->get_logger(), "ReadLoop: Requesting to re-open pipeline...");
          reconnect_requested_ = true;
          fail_count_ = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 잠깐 대기
      }
    }
  }

  // 타이머 콜백: 10fps 주기로 최신 프레임을 publish
  void timer_callback() {
    cv::Mat frame_to_publish;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_.empty()) {
        return;
      }
      // 가장 최신 프레임 복사 (이전 프레임은 덮어쓰여짐)
      frame_to_publish = latest_frame_.clone();
    }
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_to_publish).toImageMsg();
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Published frame");
  }

  // 재연결 타이머 콜백: 1초마다 재연결 요청 있으면 파이프라인 재시작
  void reconnect_callback() {
    if (reconnect_requested_ && rclcpp::ok()) {
      reconnect_requested_ = false;
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

  std::thread reading_thread_;           // 30fps로 읽는 쓰레드
  std::mutex frame_mutex_;               // 최신 프레임 보호용 mutex
  cv::Mat latest_frame_;                 // 최신 읽은 프레임 저장

  int fail_count_;
  std::atomic<bool> reconnect_requested_; // 재연결 요청 플래그
  std::atomic<bool> stop_reading_;       // 읽기 쓰레드 종료용 플래그
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


