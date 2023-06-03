#ifndef XTARK_DRIVER_HPP
#define XTARK_DRIVER_HPP
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define G     9.8
#define head1 0xAA
#define head2 0x55
#define sendType_velocity    0x11
#define sendType_pid         0x12
#define sendType_params      0x13
#define sendType_wheelspeed  0x14

#define foundType_Packages    0x06

//#define MAX_STEERING_ANGLE    0.87
//#define M_PI 3.1415926535
enum packetFinderState
{
    waitingForHead1,
    waitingForHead2,
    waitingForPayloadSize,
    waitingForPayloadType,
    waitingForPayload,
    waitingForCheckSum,
    handlePayload
};

struct pid_param
{
    int kp;
    int ki;
    int kd;
};

struct imu_data
{
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float q0;
    float q1;
    float q2;
    float q3;
};

typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;

namespace xtark_driver
{
  class XtarkDriver : public rclcpp::Node
  {
    public:
        explicit XtarkDriver(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
        virtual ~XtarkDriver();
    private:
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
        const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
        const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lvel_pub_;
        const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rvel_pub_;
        const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lset_pub_;
        const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rset_pub_;
        const rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        const rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void check_sum(uint8_t* data, long unsigned int len, uint8_t& dest);
        void SetParams(double linear_correction,int servo_bias);
        void SetPID(int p,int i, int d);
        void SetVelocity(double x, double y, double yaw);
        void distribute_data(uint8_t msg_type, uint8_t* buffer_data);
        void handle_base_data(const uint8_t* buffer_data);
        bool initRobot();
        void send_speed_callback();
        void recv_msg();

        bool recv_flag_;
        bool publish_odom_transform_;
        bool start_flag_;
        bool first_init_;
        uint8_t msg_seq_;

        
        std::string port_name_;
        std::string odom_frame_;
        std::string imu_frame_;
        std::string base_frame_;
        std::string code_version_;
        int control_rate_;
        int baud_rate_;
        double linear_correction_factor_;
        int servo_bias_;
        int kp_;   
        int ki_;
        int kd_;

        std::vector<double> imu_list_;
        std::vector<double> odom_list_;    
        std::vector<int>    wheelspeedSet_list_;
        std::vector<int>    wheelspeedGet_list_;

        serialp_ptr sp_;
        boost::asio::io_service io_service_;
        boost::system::error_code ec_;        

        packetFinderState state_;

        rclcpp::Time last_twist_time_;
        rclcpp::Time now_;
        rclcpp::Time last_time_;

        geometry_msgs::msg::Twist current_twist_;
        geometry_msgs::msg::Twist joy_twist_;
        nav_msgs::msg::Odometry odom_;
        geometry_msgs::msg::TransformStamped transformStamped_;
        
        std_msgs::msg::Int32 lvel_pub_data_;
        std_msgs::msg::Int32 rvel_pub_data_;
        std_msgs::msg::Int32 rset_pub_data_;
        std_msgs::msg::Int32 lset_pub_data_;
        sensor_msgs::msg::Imu imu_pub_data_;
        std_msgs::msg::Float32  battery_pub_data_;


        rclcpp::TimerBase::SharedPtr send_speed_timer;
        std::thread recv_thread;

        tf2_ros::TransformBroadcaster tf_broadcaster_;

  };
}


#endif





