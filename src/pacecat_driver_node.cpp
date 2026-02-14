#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstdint>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>

class PacecatScanNode : public rclcpp::Node {
public:
  PacecatScanNode()
    : rclcpp::Node("pacecat_scan_node"),
      collecting_(false),
      port_("/dev/ttyAMA0"),
      baudrate_(230400),
      READ_SIZE_(1024),
      BUFFER_THRESHOLD_(static_cast<size_t>(98 * 20 * 1.5))
  {
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    this->declare_parameter<std::string>("port", "/dev/ttyAMA0");
    this->declare_parameter<int>("baudrate", 230400);
    this->declare_parameter<int>("read_size", 1024);
    this->declare_parameter<double>("timer_frequency_hz", 10.0);
    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    READ_SIZE_ = static_cast<size_t>(this->get_parameter("read_size").as_int());
    BUFFER_THRESHOLD_ = static_cast<size_t>(98 * 20 * 1.5);
    double freq = this->get_parameter("timer_frequency_hz").as_double();
    if (freq <= 0.0) freq = 10.0;
    auto period_ms = static_cast<int>(1000.0 / freq);
    open_serial();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&PacecatScanNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Timer frequency: %.2f Hz (%d ms)", freq, period_ms);
  }
  ~PacecatScanNode() {
    if (serial_fd_ >= 0) ::close(serial_fd_);
  }
private:
  void accumulate_and_publish_points() {
    const size_t PACKET_SIZE = 98;
    const size_t POINTS_PER_PACKET = 30;
    for (size_t i = 0; i + PACKET_SIZE <= buffer_.size(); ++i) {
      if (buffer_[i] == 0xCF && buffer_[i + 1] == 0xFA) {
        uint16_t points = buffer_[i + 2] | (buffer_[i + 3] << 8);
        uint16_t start_angle = buffer_[i + 4] | (buffer_[i + 5] << 8);
        if (points != POINTS_PER_PACKET) continue;
        // 0度スタート検出でバッファリセット
        if (start_angle == 0) {
          intensity_buffer_.clear();
          distance_buffer_.clear();
          collecting_ = true;
        }
        if (!collecting_) continue;
        // 30点分のデータを格納
        for (size_t p = 0; p < POINTS_PER_PACKET; ++p) {
          size_t base = i + 8 + p * 3;
          uint8_t intensity = buffer_[base];
          uint16_t distance = buffer_[base + 1] | (buffer_[base + 2] << 8);
          intensity_buffer_.push_back(intensity);
          distance_buffer_.push_back(distance);
        }
        // 600点たまったらLaserScanとしてpublish
        if (intensity_buffer_.size() >= 600 && distance_buffer_.size() >= 600) {
          auto msg = sensor_msgs::msg::LaserScan();
          msg.header.stamp = this->now();
          msg.header.frame_id = "laser_frame";
          msg.angle_min = 0.0f;
          msg.angle_max = 2.0f * M_PI;
          msg.angle_increment = (msg.angle_max - msg.angle_min) / 600.0f;
          msg.range_min = 0.05f;
          msg.range_max = 30.0f;
          msg.ranges.resize(600);
          msg.intensities.resize(600);
          for (size_t k = 0; k < 600; ++k) {
            msg.ranges[k] = distance_buffer_[k] * 0.001f; // mm→m
            msg.intensities[k] = static_cast<float>(intensity_buffer_[k]);
          }
          pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "[360deg] LaserScan published (%zu points)", intensity_buffer_.size());
          intensity_buffer_.clear();
          distance_buffer_.clear();
          collecting_ = false;
        }
        i += PACKET_SIZE - 1; // 次のパケット探索へ
      }
    }
  }
  void open_serial() {
    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
      throw std::runtime_error("serial open failed");
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes: %s", strerror(errno));
      ::close(serial_fd_);
      throw std::runtime_error("tcgetattr failed");
    }
    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes: %s", strerror(errno));
      ::close(serial_fd_);
      throw std::runtime_error("tcsetattr failed");
    }
  }
  void timer_callback() {
    std::vector<uint8_t> temp(READ_SIZE_);
    ssize_t bytes_read = ::read(serial_fd_, temp.data(), READ_SIZE_);
    if (bytes_read > 0) {
      buffer_.insert(buffer_.end(), temp.begin(), temp.begin() + bytes_read);
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
      return;
    }
    if (buffer_.size() >= BUFFER_THRESHOLD_) {
      accumulate_and_publish_points();
      buffer_.clear();
    }
  }
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string port_;
  int baudrate_;
  size_t READ_SIZE_;
  size_t BUFFER_THRESHOLD_;
  int serial_fd_ = -1;
  std::vector<uint8_t> buffer_;
  std::vector<uint8_t> intensity_buffer_;
  std::vector<uint16_t> distance_buffer_;
  bool collecting_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PacecatScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
