/**
 * @file imu_node.cpp
 * @author Yuwei
 * @brief 真正版本的IMU驱动
 * @version 0.1
 * @date 2020-12-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "imu.hpp"
#include <Eigen/Geometry>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace std;
using namespace boost::asio;
IMU imuraw;
sensor_msgs::Imu imuros;

uint8_t buf[32];

void printall(std::vector<uint8_t> buffer);
int main(int argc, char **argv)
{
  ros::init(argc, argv, "CartIMU");
  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("CartImu", 1000);
  io_service iosev;
  serial_port sp(iosev);
  try
  {
    sp.open("/dev/ttyUSB0");
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM(e.what() << ". please plugin in the USB of IMU.");
  }
  sp.set_option(serial_port::baud_rate(115200));
  sp.set_option(serial_port::flow_control(serial_port::flow_control::none)); //无流控制
  sp.set_option(serial_port::parity(serial_port::parity::none));             //无校验
  sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));        //1位停止位
  sp.set_option(serial_port::character_size(8));
  ros::Rate loop_rate(100);
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
  ROS_INFO_STREAM("Open /dev/ttyUSB0 successfully!");
  while (ros::ok())
  {
    if (read(sp, buffer(buf)) == 32)
    {
      imuros.header.stamp = ros::Time::now();
      imuraw.parse_readbuff(buf, 32);
      // imuraw.printall();
      if (imuraw.CRC_cal(&buf[1], 30) != buf[31])
        std::cout << count++ << std::endl;
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
      imu_pub.publish(imuros);
      ros::spinOnce();
      loop_rate.sleep();
    }
    else
    {
      std::cout << "no enough data" << std::endl;
    }
  }
  iosev.run();
  return 0;
}
