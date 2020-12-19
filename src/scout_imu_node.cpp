#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "imu.hpp"
#include <Eigen/Geometry>
#include <serial/serial.h> //这是一个跨平台的串口库
using namespace std;
IMU imuraw;
sensor_msgs::Imu imuros;
std::vector<uint8_t> buffer;      //用于转储raw date
std::vector<uint8_t> temp_buffer; //由于serial只支持直接覆盖，因此需要一个temp量来存储

void printall(std::vector<uint8_t> buffer);
void signal_handler_IO(int status);
void parseSerial(serial::Serial &sp);
int main(int argc, char **argv)
{
  ros::init(argc, argv, "CartIMU");
  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("CartImu", 10000);
  serial::Serial serial_port;
  // FIXMETimeout规定了两个字符之间的最大间隔(ms)，理论上可以设置为1/（100Hz），来辅助分包，这里暂时取更大值
  serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
  serial_port.setPort("/dev/ttyUSB0");
  serial_port.setBaudrate(115200);
  serial_port.setTimeout(timeout);
  ros::Rate loop_rate(100);
  try
  {
    serial_port.open();
  }
  catch (const serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port.");
    return -1;
  }
  if (serial_port.isOpen())
  {
    ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
  }
  else
  {
    return -1;
  }
  Eigen::Matrix3f rotation;
  Eigen::Quaternionf q;
  //不变的消息内容事先设置，降低运算量
  imuros.header.frame_id = "cart_imu";
  imuros.angular_velocity_covariance[0] = 0.013;
  imuros.angular_velocity_covariance[1] = 0;
  imuros.angular_velocity_covariance[2] = 0;
  imuros.angular_velocity_covariance[3] = 0;
  imuros.angular_velocity_covariance[4] = 0.013;
  imuros.angular_velocity_covariance[5] = 0;
  imuros.angular_velocity_covariance[6] = 0;
  imuros.angular_velocity_covariance[7] = 0;
  imuros.angular_velocity_covariance[8] = 0.013;

  imuros.linear_acceleration_covariance[0] = 0.083;
  imuros.linear_acceleration_covariance[1] = 0;
  imuros.linear_acceleration_covariance[2] = 0;
  imuros.linear_acceleration_covariance[3] = 0;
  imuros.linear_acceleration_covariance[4] = 0.083;
  imuros.linear_acceleration_covariance[5] = 0;
  imuros.linear_acceleration_covariance[6] = 0;
  imuros.linear_acceleration_covariance[7] = 0;
  imuros.linear_acceleration_covariance[8] = 0.083;

  int count = 0;
  int count_su =0 ;
  while (ros::ok())
  {
    size_t n = serial_port.available();
    if (n >= 32)
    {
      n = serial_port.read(buffer, 32);
      if (buffer.size() >= 32)
      {
        auto it = buffer.begin();
        if (*it == 0x68 && *(it + 1) == 0x1F && *(it + 2) == 0x00 && *(it + 3) == 0x84)
        {
          uint8_t crc_value = imuraw.CRC_cal(buffer, 30);
          //循环不变量：传给校验的必然是从0x68开始的长于32字节的vector
          if (crc_value == *(it + 31))
          {
            imuros.header.stamp = ros::Time::now();

            imuraw.roll = imuraw.ToAngle(buffer, 4);
            imuraw.pitch = imuraw.ToAngle(buffer, 7);
            imuraw.yaw = imuraw.ToAngle(buffer, 10);
            imuraw.x_acc = imuraw.ToAcc(buffer, 13);
            imuraw.y_acc = imuraw.ToAcc(buffer, 16);
            imuraw.z_acc = imuraw.ToAcc(buffer, 19);
            imuraw.roll_vel = imuraw.ToAngle(buffer, 22);
            imuraw.pitch_vel = imuraw.ToAngle(buffer, 25);
            imuraw.yaw_vel = imuraw.ToAngle(buffer, 28);
            rotation = Eigen::AngleAxisf(imuraw.roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(imuraw.pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(imuraw.yaw, Eigen::Vector3f::UnitZ());
            q = rotation;
            q.normalize();
            imuros.orientation.w = q.w();
            imuros.orientation.x = q.x();
            imuros.orientation.y = q.y();
            imuros.orientation.z = q.z();
            imuros.angular_velocity.x = imuraw.roll_vel;
            imuros.angular_velocity.y = imuraw.pitch_vel;
            imuros.angular_velocity.z = imuraw.yaw_vel;
            imuros.linear_acceleration.x = imuraw.x_acc;
            imuros.linear_acceleration.y = imuraw.y_acc;
            imuros.linear_acceleration.z = imuraw.z_acc;
            buffer.erase(buffer.begin(), buffer.begin() + 32);
            imu_pub.publish(imuros);
          }
          else
          {
            std::cout << count++ << std::endl;
          }
        }
        else
        {
          std::cout << count++ << std::endl;
        }
      }
      else
      {
        std::cout << count++ << std::endl;
      }
    }
    else{
      std::cout << count_su << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  serial_port.close();
  std::cout << "program died " << std::endl;
  return 0;
}
// 用于输出buffer，主要用于调试
void printall(const std::vector<uint8_t> buffer)
{
  for (auto data : buffer)
  {
    printf("%x ", data);
  }
  printf("\n");
}
