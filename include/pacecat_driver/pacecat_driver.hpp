#ifndef PACECAT_DRIVER_HPP_
#define PACECAT_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <vector>
#include <mutex>
#include <atomic>

namespace pacecat_driver
{

// プロトコル定義
struct PacecatPacketHeader
{
  uint16_t header;        // 0xFACF (CF FA)
  uint16_t points;        // ポイント数 (30点 = 0x001E)
  uint16_t start_angle;   // 開始角度 (0.1度単位)
  uint16_t angle_step;    // 角度ステップ (180 = 18度 = 0x00B4)
} __attribute__((packed));

struct PacecatPointData
{
  uint8_t data[3];  // 距離・反射強度データ（3バイト）
} __attribute__((packed));

struct PacecatPacket
{
  PacecatPacketHeader header;
  PacecatPointData points[30];  // 30点分のデータ
} __attribute__((packed));

struct LidarPoint {
  float angle;
  float distance;
  float intensity;
};

class PacecatDriver
{
public:
  PacecatDriver();
  virtual ~PacecatDriver();

  bool open(const std::string & port, int baudrate = 230400);
  void close();
  void start();
  void stop();
  bool readData(std::vector<uint8_t> & buffer);
  bool processData(std::vector<uint8_t> & buffer, std::vector<float> & ranges, std::vector<float> & intensities);

private:
  bool configureSerial(int baudrate);

  // メンバー変数
  int serial_fd_ = -1;
  std::string port_;
  int baudrate_ = 230400;
  std::atomic<bool> running_;
  std::mutex open_mutex_;
  bool is_open_ = false;
};

} // namespace pacecat_driver

#endif // PACECAT_DRIVER_HPP_