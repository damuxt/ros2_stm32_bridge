#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <mutex>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace ros2_stm32 {
namespace driver {

enum ParseStatus { HEADER, TYPE, DATA, CHECKSUM, END };

enum MessageType { VOLTAGE = 1, WHEEL_ENCODE = 2, ANGULAR_VELOCITY = 3, ACCELERATION = 4, ORIENTATION = 5, SPEED = 6, PID = 7, SERVO = 8 };

class MiniDriver : public rclcpp::Node {
 public:
  MiniDriver();
  ~MiniDriver();

  void Run();

 private:
  void Pid();
  bool Init();
  void ParseMessage();
  void DistributeMessage(MessageType type, uint8_t* payload);
  void CheckAndPublishImu();
  int CalculateDelta(int &current_encode, int receive_encode);
  void HandleEncodeMessage(short left_encode, short right_encode);
  void TwistHandleCallback(const geometry_msgs::msg::Twist & msg);
  void SendTimerCallback();
  void SpeedCommand(short left,short right);

  std::string PrintHex(uint8_t* data, int length) {
    std::stringstream ss;
    for (int i = 0; i < length; i++) {
      ss << std::setw(2) << std::setfill('0') << std::hex << (int)data[i] << " ";
    }
    return ss.str();
  }

 private:
  bool parse_flag_;

  boost::system::error_code ec_;
  boost::asio::io_service io_service_;
  serial_port_ptr port_;

  std::mutex mutex_;
  std::mutex twist_mutex_;

  std::string port_name_;
  int baud_rate_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr voltage_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr wheel_encode_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscribe_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  

  sensor_msgs::msg::Imu imu_msg_;
  bool imu_msg_flag_[3]{false, false, false};  //  [ANGULAR_VELOCITY, ACCELERATION, ORIENTATION]

  int control_rate_;
  std::string odom_frame_, base_frame_;

  double maximum_encoding_;
  double pulse_per_cycle_, encoder_resolution_, reduction_ratio_, pid_rate_;
  double model_param_cw_, model_param_acw_, wheel_diameter_;

  bool start_flag_;
  double delta_time_;
  double accumulation_x_, accumulation_y_, accumulation_th_;
  int current_left_encode_, current_right_encode_;

  rclcpp::Time now_, last_time_, last_twist_time_;
  geometry_msgs::msg::Twist current_twist_;
  
};

}  // namespace driver
}  // namespace ros2_stm32
