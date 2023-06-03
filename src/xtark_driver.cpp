#include "xtark_driver.hpp"
namespace xtark_driver
{
    XtarkDriver::XtarkDriver(rclcpp::NodeOptions options)
    : Node("xtark_driver", options),
    odometry_pub_(create_publisher<nav_msgs::msg::Odometry>("odom", 10)),
    battery_pub_(create_publisher<std_msgs::msg::Float32>("voltage", 1)),
    imu_pub_(create_publisher<sensor_msgs::msg::Imu>("imu", 10)),
    lvel_pub_(create_publisher<std_msgs::msg::Int32>("xtark/lvel", 10)),
    rvel_pub_(create_publisher<std_msgs::msg::Int32>("xtark/rvel", 10)),
    lset_pub_(create_publisher<std_msgs::msg::Int32>("xtark/lset", 10)),
    rset_pub_(create_publisher<std_msgs::msg::Int32>("xtark/rset", 10)),
    cmd_sub_(create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::QoS(10),
            std::bind(&XtarkDriver::CmdVelCallback, this, std::placeholders::_1))),
    joy_sub_(create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&XtarkDriver::JoyCallback, this, std::placeholders::_1))),
    tf_broadcaster_(this)
    {
        this->declare_parameter<std::string>("port_name", "/dev/ttyTHS0");
        this->get_parameter("port_name", port_name_);
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_);
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->get_parameter("base_frame", base_frame_);
        this->declare_parameter<std::string>("imu_frame", "base_imu_link");
        this->get_parameter("imu_frame", imu_frame_);

        this->declare_parameter<int>("baud_rate", 115200);
        this->get_parameter("baud_rate", baud_rate_);
        this->declare_parameter<int>("control_rate", 50);
        this->get_parameter("control_rate", control_rate_);

        this->declare_parameter<double>("linear_correction_factor", 1.0);
        this->get_parameter("linear_correction_factor", linear_correction_factor_);

        this->declare_parameter<int>("servo_bias", 0);
        this->get_parameter("servo_bias", servo_bias_);

        this->declare_parameter<bool>("publish_odom_transform", true);
        this->get_parameter("publish_odom_transform", publish_odom_transform_);

        this->declare_parameter<int>("Kp", 300);
        this->get_parameter("Kp", kp_);
        this->declare_parameter<int>("Ki", 0);
        this->get_parameter("Ki", ki_);
        this->declare_parameter<int>("Kd", 200);
        this->get_parameter("Kd", kd_);

        odom_list_.resize(6,0.0);
        imu_list_.resize(9,0.0);
        wheelspeedSet_list_.resize(2,0);
        wheelspeedGet_list_.resize(2,0);

        last_twist_time_ = this->now();
        now_ = this->now();
        last_time_= this->now();
        if(initRobot())
        {
            RCLCPP_INFO(this->get_logger(), "robot initialized");
            SetParams(linear_correction_factor_,servo_bias_);
            sleep(0.1);
            SetPID(kp_,ki_,kd_);
            send_speed_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000/control_rate_)), std::bind(&XtarkDriver::send_speed_callback, this));
            recv_thread = std::thread(std::bind(&XtarkDriver::recv_msg, this));
        }
        SetVelocity(0,0,0);
        RCLCPP_INFO(this->get_logger(), "XtarkDriver Node has been initialized.");
    }
    XtarkDriver::~XtarkDriver()
    {
        if (recv_thread.joinable()) {
            recv_thread.join();
        }
        RCLCPP_INFO(this->get_logger(), "XtarkDriver Node has been destroyed.");
    }

    void XtarkDriver::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "cmd vel data is null.");
            return;
        }

        last_twist_time_ = this->now();
        current_twist_ = *msg;
    }
    void XtarkDriver::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "cmd Joy data is null.");
            return;
        }

        if(msg->buttons[7] == 1)
        {
            joy_twist_.linear.x = msg->axes[1];
            joy_twist_.angular.z = msg->axes[2];
        }
        else
        {
            joy_twist_.linear.x = 0;
            joy_twist_.angular.z = 0;
        }

    }
    /*与OpenCRP串口通信接收数据线程*/
    void XtarkDriver::recv_msg()
    {
        uint8_t payload_size, check_num, buffer_data[255],payload_type;
        state_ = waitingForHead1;
        recv_flag_ = true;
        while(recv_flag_)
        {
            switch (state_)
            {
                case waitingForHead1:
                    check_num = 0x00;
                    boost::asio::read(*sp_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
                    state_ = buffer_data[0] == head1 ? waitingForHead2 : waitingForHead1;
                    if(state_ == waitingForHead1)
                    {
                        RCLCPP_DEBUG(this->get_logger(), "recv head1 error : ->%d", static_cast<int>(buffer_data[0]));
                    }
                    break;
                case waitingForHead2:
                    boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[1],1),ec_);
                    state_ = buffer_data[1] == head2 ? waitingForPayloadSize : waitingForHead1;
                    if(state_ == waitingForHead1)
                    {
                        RCLCPP_DEBUG(this->get_logger(), "recv head1 error : ->%d", static_cast<int>(buffer_data[1]));
                    }
                    break;
                case waitingForPayloadSize:
                    boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[2],1),ec_);
                    payload_size = buffer_data[2] - 4;
                    state_ = waitingForPayload;
                    break;
                case waitingForPayload:
                    boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3],payload_size),ec_);
                    payload_type = buffer_data[3];
                    state_ = waitingForCheckSum;
                    break;
                case waitingForCheckSum:
                    boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3+payload_size],1),ec_);
                    check_sum(buffer_data,3+payload_size,check_num);
                    state_ = buffer_data[3+payload_size] == check_num ? handlePayload : waitingForHead1;
                    if(state_ == waitingForHead1)
                    {
                        RCLCPP_DEBUG(this->get_logger(), "check sum error! recv is : ->%d, calc is %d", static_cast<int>(buffer_data[3+payload_size]), check_num);
                    }
                    break;
                case handlePayload:
                    distribute_data(payload_type, buffer_data);
                    state_ = waitingForHead1;
                    break;
                default:
                    state_ = waitingForHead1;
                    break;
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

    }
    /*数据校验分发*/
    void XtarkDriver::distribute_data(uint8_t msg_type, uint8_t* buffer_data)
    {
        if(msg_type == foundType_Packages)
            handle_base_data(buffer_data);
    }
    /*收到串口数据包解析函数*/
    void XtarkDriver::handle_base_data(const uint8_t* buffer_data)
    {
        now_ = this->now();
        //gyro
        imu_list_[0]=((double)((int16_t)(buffer_data[4]*256+buffer_data[5]))/32768*2000/180*3.1415);
        imu_list_[1]=((double)((int16_t)(buffer_data[6]*256+buffer_data[7]))/32768*2000/180*3.1415);
        imu_list_[2]=((double)((int16_t)(buffer_data[8]*256+buffer_data[9]))/32768*2000/180*3.1415);
        //Acc 
        imu_list_[3]=((double)((int16_t)(buffer_data[10]*256+buffer_data[11]))/32768*2*9.8);
        imu_list_[4]=((double)((int16_t)(buffer_data[12]*256+buffer_data[13]))/32768*2*9.8);
        imu_list_[5]=((double)((int16_t)(buffer_data[14]*256+buffer_data[15]))/32768*2*9.8);
        //Angle 
        imu_list_[6]=((double)((int16_t)(buffer_data[16]*256+buffer_data[17]))/100);
        imu_list_[7]=((double)((int16_t)(buffer_data[18]*256+buffer_data[19]))/100);
        imu_list_[8]=((double)((int16_t)(buffer_data[20]*256+buffer_data[21]))/100);
        imu_pub_data_.header.stamp = now_;
        imu_pub_data_.header.frame_id = imu_frame_;
        imu_pub_data_.angular_velocity.x = imu_list_[0];
        imu_pub_data_.angular_velocity.y = imu_list_[1];
        imu_pub_data_.angular_velocity.z = imu_list_[2];
        imu_pub_data_.linear_acceleration.x = imu_list_[3];
        imu_pub_data_.linear_acceleration.y = imu_list_[4];
        imu_pub_data_.linear_acceleration.z = imu_list_[5];
        imu_pub_data_.orientation_covariance = {1e6, 0, 0,
                                                0, 1e6, 0,
                                                0, 0, 0.02};
        imu_pub_data_.angular_velocity_covariance = {1e6, 0, 0,
                                                    0, 1e6, 0,
                                                    0, 0, 1e6};
        imu_pub_data_.linear_acceleration_covariance = {1e-2, 0, 0,
                                                        0, 0, 0,
                                                        0, 0, 0};
        imu_pub_->publish(imu_pub_data_);

        //odom
        odom_list_[0]=((double)((int16_t)(buffer_data[22]*256+buffer_data[23]))/1000);
        odom_list_[1]=((double)((int16_t)(buffer_data[24]*256+buffer_data[25]))/1000);
        odom_list_[2]=((double)((int16_t)(buffer_data[26]*256+buffer_data[27]))/1000);
        //dx dy dyaw base_frame
        odom_list_[3]=((double)((int16_t)(buffer_data[28]*256+buffer_data[29]))/1000);
        odom_list_[4]=((double)((int16_t)(buffer_data[30]*256+buffer_data[31]))/1000);
        odom_list_[5]=((double)((int16_t)(buffer_data[32]*256+buffer_data[33]))/1000);

        transformStamped_.header.stamp    = now_;
        transformStamped_.header.frame_id = odom_frame_;
        transformStamped_.child_frame_id  = base_frame_;
        transformStamped_.transform.translation.x = odom_list_[0];
        transformStamped_.transform.translation.y = odom_list_[1];
        transformStamped_.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0,0,odom_list_[2]);
        transformStamped_.transform.rotation = tf2::toMsg(q);
        if(publish_odom_transform_)
            tf_broadcaster_.sendTransform(transformStamped_);

        odom_.header.frame_id = odom_frame_;
        odom_.child_frame_id  = base_frame_;
        odom_.header.stamp    = now_;
        odom_.pose.pose.position.x = odom_list_[0];
        odom_.pose.pose.position.y = odom_list_[1];
        odom_.pose.pose.position.z = 0;
        odom_.pose.pose.orientation = tf2::toMsg(q);
        odom_.twist.twist.linear.x = odom_list_[3]/((now_-last_time_).seconds());
        odom_.twist.twist.linear.y = odom_list_[4]/((now_-last_time_).seconds());
        odom_.twist.twist.angular.z = odom_list_[5]/((now_-last_time_).seconds());
        // [Remaining covariance setting code is same]

        odometry_pub_->publish(odom_);
        last_time_ = now_;

        wheelspeedGet_list_[0]=((int16_t)(buffer_data[34]*256+buffer_data[35]));
        wheelspeedGet_list_[1]=((int16_t)(buffer_data[36]*256+buffer_data[37]));
        wheelspeedSet_list_[0]=((int16_t)(buffer_data[38]*256+buffer_data[39]));
        wheelspeedSet_list_[1]=((int16_t)(buffer_data[40]*256+buffer_data[41]));
        lvel_pub_data_.data = wheelspeedGet_list_[0];
        rvel_pub_data_.data = wheelspeedGet_list_[1];
        lset_pub_data_.data = wheelspeedSet_list_[0];
        rset_pub_data_.data = wheelspeedSet_list_[1];
        lvel_pub_->publish(lvel_pub_data_);
        rvel_pub_->publish(rvel_pub_data_);
        lset_pub_->publish(lset_pub_data_);
        rset_pub_->publish(rset_pub_data_);
        battery_pub_data_.data = (double)(((buffer_data[42]<<8)+buffer_data[43]))/100;
        battery_pub_->publish(battery_pub_data_);
    }
    /*将收到的/cmd_vel上的速度话题消息通过串口发送给机器人*/
    void XtarkDriver::send_speed_callback()
    {
        double linear_speed, angular_speed;
        rclcpp::Time now = this->now();
        if((now - last_twist_time_).seconds() <= 1.0)
        {
            linear_speed = current_twist_.linear.x;
            angular_speed = current_twist_.angular.z;
        }
        else
        {
            linear_speed  = joy_twist_.linear.x;
            angular_speed = joy_twist_.angular.z;
        }

        SetVelocity(linear_speed,0,angular_speed);
    }

    /*初始化硬件端口*/
    bool XtarkDriver::initRobot()
    {
        if(sp_)
        {
            RCLCPP_ERROR(this->get_logger(), "The SerialPort is already opened!");
            return false;
        }
        sp_ = serialp_ptr(new boost::asio::serial_port(io_service_));
        sp_->open(port_name_,ec_);
        if(ec_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "error : port_->open() failed...port_name=%s, e=%s", port_name_.c_str(), ec_.message().c_str());
            return false;
        }
        sp_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        sp_->set_option(boost::asio::serial_port_base::character_size(8));
        sp_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        sp_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        sp_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        
        return true;
    }
    /*串口通信校验位计算辅助函数*/
    void XtarkDriver::check_sum(uint8_t* data, long unsigned int len, uint8_t& dest)
    {
        dest = 0x00;
        for(long unsigned int i=0;i<len;i++)
        {
            dest += *(data + i);
        }
    }
    /*底盘参数发送函数*/
    void XtarkDriver::SetParams(double linear_correction,int servo_bias)
    {
        static uint8_t param_data[20];
        param_data[0]  = head1;
        param_data[1]  = head2;
        param_data[2]  = 0x09;
        param_data[3]  = sendType_params;
        param_data[4]  = (int16_t)((int16_t)(linear_correction*1000)>>8)   &0xff;
        param_data[5]  = (int16_t)(linear_correction*1000)        &0xff;
        param_data[6]  = ((int16_t)((int16_t)servo_bias)>>8)      &0xff;
        param_data[7]  = ((int16_t)servo_bias)                     &0xff;
        check_sum(param_data,8,param_data[8]);
        boost::asio::write(*sp_.get(),boost::asio::buffer(param_data,9),ec_);
    }
    /*PID参数发送函数*/
    void XtarkDriver::SetPID(int p,int i, int d)
    {
        static uint8_t pid_data[11];
        pid_data[0] = head1;
        pid_data[1] = head2;
        pid_data[2] = 0x0b;
        pid_data[3] = sendType_pid;
        pid_data[4] = (p>>8)&0xff;
        pid_data[5] = p&0xff;
        pid_data[6] = (i>>8)&0xff;
        pid_data[7] = i&0xff;
        pid_data[8] = (d>>8)&0xff;
        pid_data[9] = d&0xff;
        check_sum(pid_data,10,pid_data[10]);
        boost::asio::write(*sp_.get(),boost::asio::buffer(pid_data,11),ec_);
    }
    /*底盘速度发送函数*/
    void XtarkDriver::SetVelocity(double x, double y, double yaw)
    {
        static uint8_t vel_data[11];
        vel_data[0] = head1;
        vel_data[1] = head2;
        vel_data[2] = 0x0b;
        vel_data[3] = sendType_velocity;
        vel_data[4] = ((int16_t)(x*1000)>>8) & 0xff;
        vel_data[5] = ((int16_t)(x*1000)) & 0xff;
        vel_data[6] = ((int16_t)(y*1000)>>8) & 0xff;
        vel_data[7] = ((int16_t)(y*1000)) & 0xff;
        vel_data[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
        vel_data[9] = ((int16_t)(yaw*1000)) & 0xff;
        check_sum(vel_data,10,vel_data[10]);
        boost::asio::write(*sp_.get(),boost::asio::buffer(vel_data,11),ec_);
    }
}