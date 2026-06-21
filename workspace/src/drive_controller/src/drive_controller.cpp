#include <chrono>
#include <iostream>
#include <memory>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robot_msgs/msg/int64_vector.hpp"
#include "robot_msgs/msg/float32_vector.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class DriveController : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ticks_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ticks_right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_left_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_right_sub_;


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cmv_vel_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cmv_vel_right_pub_;

#if false
    rclcpp::Subscription<robot_msgs::msg::Float32Vector>::SharedPtr pid_sub_;
#endif
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    float kp_, ki_, kd_;

    double r_, l_, kl_, ka_, ticks_;
    std::string frame_id_, child_frame_id_;
    bool publish_odom_;
    bool publish_tf_;
    bool limit_vels_;
    float max_x_vel_, max_yaw_vel_;

    std::vector<float> reset_X_, reset_P_, prev_X_, prev_P_;
    int64_t ticks_l_, ticks_r_;
    float wl_, wr_;

public:
    DriveController();

private:
    void delay(uint64_t ms);
    
    std::vector<float> ticks2rads(const std::vector<int64_t>& T);
    std::vector<int64_t> rads2ticks(const std::vector<float>& P);

    std::vector<float> calcGlobalPose(const std::vector<int64_t>& ticks);
    
    void ticksLeftCallback(const std_msgs::msg::Int64& msg);
    void ticksRightCallback(const std_msgs::msg::Int64& msg);
    void velLeftCallback(const std_msgs::msg::Float32& msg);
    void velRightCallback(const std_msgs::msg::Float32& msg);

    std::vector<float> calcGlobalVels(float wl, float wr);

    void odomCallback(const robot_msgs::msg::Int64Vector& msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg);
#if false
    void pidCallback(const robot_msgs::msg::Float32Vector& msg);
#endif
    void resetOdomCallback(const std_msgs::msg::Bool& msg);
};


DriveController::DriveController() : Node("drive_controller"), ticks_l_{0}, ticks_r_{0} {
    this->declare_parameter("publish_odom", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("publish_tf", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("odom_pub", rclcpp::PARAMETER_STRING);
    this->declare_parameter("cmd_vel_sub", rclcpp::PARAMETER_STRING);
    this->declare_parameter("cmd_vel_pub_left_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("cmd_vel_pub_right_topic", rclcpp::PARAMETER_STRING);
#if false
    this->declare_parameter("pid_sub_topic", rclcpp::PARAMETER_STRING);
#endif
    this->declare_parameter("reset_sub_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ticks_left_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ticks_right_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("vel_left_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("vel_right_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("r", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("l", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("kl", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ka", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ticks", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
    this->declare_parameter("child_frame_id", rclcpp::PARAMETER_STRING);
    this->declare_parameter<std::string>("frame_prefix", "");  // namespacing: <vehicle_id>/ к фреймам
    this->declare_parameter("kp", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ki", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("kd", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("limit_vels", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("max_x_vel", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_yaw_vel", rclcpp::PARAMETER_DOUBLE);

    // namespacing alpha-схемы: топики ОТНОСИТЕЛЬНЫЕ (push-ros-namespace из tvp_core_vehicle
    // подставит /<vehicle_id>/), фреймы префиксуются frame_prefix. join_ns_topic больше не нужен.
    std::string frame_prefix = this->get_parameter("frame_prefix").as_string();

    publish_odom_ = this->get_parameter("publish_odom").as_bool();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    std::string odom_pub_topic = this->get_parameter("odom_pub").as_string();
    std::string cmd_vel_sub_topic = this->get_parameter("cmd_vel_sub").as_string();
    std::string cmd_vel_pub_left_topic = this->get_parameter("cmd_vel_pub_left_topic").as_string();
    std::string cmd_vel_pub_right_topic = this->get_parameter("cmd_vel_pub_right_topic").as_string();
#if false
    std::string pid_sub_topic = this->get_parameter("pid_sub_topic").as_string();
#endif
    std::string reset_sub_topic = this->get_parameter("reset_sub_topic").as_string();
    std::string ticks_left_topic = this->get_parameter("ticks_left_topic").as_string();
    std::string ticks_right_topic = this->get_parameter("ticks_right_topic").as_string();
    std::string vel_left_topic = this->get_parameter("vel_left_topic").as_string();
    std::string vel_right_topic = this->get_parameter("vel_right_topic").as_string();


    r_ = this->get_parameter("r").as_double();
    l_ = this->get_parameter("l").as_double();
    kl_ = this->get_parameter("kl").as_double();
    ka_ = this->get_parameter("ka").as_double();
    ticks_ = this->get_parameter("ticks").as_double();
    frame_id_ = frame_prefix + this->get_parameter("frame_id").as_string();
    child_frame_id_ = frame_prefix + this->get_parameter("child_frame_id").as_string();
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    limit_vels_ = this->get_parameter("limit_vels").as_bool();
    max_x_vel_ = this->get_parameter("max_x_vel").as_double();
    max_yaw_vel_ = this->get_parameter("max_yaw_vel").as_double();

    RCLCPP_INFO(this->get_logger(), "publish_odom: '%s'", publish_odom_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "publish_tf: '%s'", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "odom_pub_topic: '%s'", odom_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_pub_topic: '%s'", odom_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "vel_sub_topic: '%s'", cmd_vel_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel_pub_left_topic: '%s'", cmd_vel_pub_left_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel_pub_right_topic: '%s'", cmd_vel_pub_right_topic.c_str());
#if false
    RCLCPP_INFO(this->get_logger(), "pid_sub_topic: '%s'", pid_sub_topic.c_str());
#endif
    RCLCPP_INFO(this->get_logger(), "reset_sub_topic: '%s'", reset_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "ticks_left_topic: '%s'", ticks_left_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "ticks_right_topic: '%s'", ticks_right_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "vel_left_topic: '%s'", vel_left_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "vel_right_topic: '%s'", vel_right_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "r: %f", r_);
    RCLCPP_INFO(this->get_logger(), "l: %f", l_);
    RCLCPP_INFO(this->get_logger(), "kl: %f", kl_);
    RCLCPP_INFO(this->get_logger(), "ka: %f", ka_);
    RCLCPP_INFO(this->get_logger(), "ticks: %f", ticks_);
    RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "child_frame_id: '%s'", child_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "kp: %f", kp_);
    RCLCPP_INFO(this->get_logger(), "ki: %f", ki_);
    RCLCPP_INFO(this->get_logger(), "kd: %f", kd_);
    RCLCPP_INFO(this->get_logger(), "limit_vels: %s", (limit_vels_ ? "true" : "false"));
    RCLCPP_INFO(this->get_logger(), "max_x_vel: %f", max_x_vel_);
    RCLCPP_INFO(this->get_logger(), "max_yaw_vel: %f", max_yaw_vel_);

    ticks_left_sub_ = this->create_subscription<std_msgs::msg::Int64>(ticks_left_topic, 10, std::bind(&DriveController::ticksLeftCallback, this, _1));
    ticks_right_sub_ = this->create_subscription<std_msgs::msg::Int64>(ticks_right_topic, 10, std::bind(&DriveController::ticksRightCallback, this, _1));

    vel_left_sub_ = this->create_subscription<std_msgs::msg::Float32>(vel_left_topic, 10, std::bind(&DriveController::velLeftCallback, this, _1));
    vel_right_sub_ = this->create_subscription<std_msgs::msg::Float32>(vel_right_topic, 10, std::bind(&DriveController::velRightCallback, this, _1));


    if (publish_odom_) {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic, 10);
    }

    if (publish_tf_) {
        tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_sub_topic, 10, std::bind(&DriveController::cmdVelCallback, this, _1));

    rclcpp::QoS qos(1);
    qos.reliable();

    cmv_vel_left_pub_ = this->create_publisher<std_msgs::msg::Float32>(cmd_vel_pub_left_topic, qos);
    cmv_vel_right_pub_ = this->create_publisher<std_msgs::msg::Float32>(cmd_vel_pub_right_topic, qos);

#if false
    pid_sub_ = this->create_subscription<robot_msgs::msg::Float32Vector>(pid_sub_topic, 10, std::bind(&DriveController::pidCallback, this, _1));
#endif
    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(reset_sub_topic, 10, std::bind(&DriveController::resetOdomCallback, this, _1));

    reset_X_ = {0., 0., 0.};
    reset_P_ = {0., 0.};

    prev_X_ = reset_X_;
    prev_P_ = reset_P_;
}


void DriveController::delay(uint64_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


std::vector<float> DriveController::ticks2rads(const std::vector<int64_t>& T) {
    std::vector<float> P;
    for (size_t i = 0; i < T.size(); i++) {
        P.push_back(2. * std::numbers::pi * T[i] / ticks_);
    }

    return P;
}


std::vector<int64_t> DriveController::rads2ticks(const std::vector<float>& P) {
    std::vector<int64_t> T;
    for (size_t i = 0; i < P.size(); i++) {
        T.push_back(P[i] * ticks_ / 2. / std::numbers::pi);
    }

    return T;
}


std::vector<float> DriveController::calcGlobalPose(const std::vector<int64_t>& ticks) {
    std::vector<float> P = ticks2rads(ticks);

    std::vector<float> X;
    X.resize(3);

    float dpl = P[0] - prev_P_[0];
    float dpr = P[1] - prev_P_[1];

    float ds = kl_ * (dpr + dpl) * r_ / 2.;
    float dth = ka_ * (dpr - dpl) * r_ / l_;

    float dx = ds * std::cos(dth);
    float dy = ds * std::sin(dth);
    float dY = dth;

    X[0] = prev_X_[0] + dx * std::cos(prev_X_[2]) - dy * std::sin(prev_X_[2]);
    X[1] = prev_X_[1] + dx * std::sin(prev_X_[2]) + dy * std::cos(prev_X_[2]);
    X[2] = prev_X_[2] + dY;

    prev_X_ = X;
    prev_P_ = P;

    return X;
}


void DriveController::ticksLeftCallback(const std_msgs::msg::Int64& msg) {
    ticks_l_ = msg.data;
}


void DriveController::ticksRightCallback(const std_msgs::msg::Int64& msg) {
    ticks_r_ = msg.data;
    robot_msgs::msg::Int64Vector v;
    v.data.push_back(ticks_l_);
    v.data.push_back(ticks_r_);
    odomCallback(v);
}


void DriveController::velLeftCallback(const std_msgs::msg::Float32& msg) {
    wl_ = msg.data;
}


void DriveController::velRightCallback(const std_msgs::msg::Float32& msg) {
    wr_ = msg.data;
}


std::vector<float> DriveController::calcGlobalVels(float wl, float wr) {
    float vl = r_ * wl;
    float vr = r_ * wr;

    float v = 1/2. * (vl + vr);
    float w = (vr - vl) / l_;

    std::vector<float> global_vels;
    
    global_vels.push_back(v);
    global_vels.push_back(w);

    return global_vels;
}


void DriveController::odomCallback(const robot_msgs::msg::Int64Vector& msg) {
    auto odom = nav_msgs::msg::Odometry();

    odom.header.frame_id = frame_id_;
    odom.header.stamp = this->get_clock()->now();
    odom.child_frame_id = child_frame_id_;

    std::vector<float> X = calcGlobalPose(msg.data);

    odom.pose.pose.position.x = X[0];
    odom.pose.pose.position.y = X[1];
    odom.pose.pose.position.z = 0.;

    tf2::Quaternion q;
    q.setRPY(0., 0., X[2]);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    auto V = calcGlobalVels(wl_, wr_);

    odom.twist.twist.linear.x = V[0];
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = V[1];

    if (publish_odom_) {
        odom_pub_->publish(odom);
    }

    auto transform = geometry_msgs::msg::TransformStamped();

    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;

    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;

    transform.transform.rotation.x = odom.pose.pose.orientation.x;
    transform.transform.rotation.y = odom.pose.pose.orientation.y;
    transform.transform.rotation.z = odom.pose.pose.orientation.z;
    transform.transform.rotation.w = odom.pose.pose.orientation.w;
    
    if (publish_tf_) {
        tf2_broadcaster_->sendTransform(transform);
    }
}


void DriveController::cmdVelCallback(const geometry_msgs::msg::Twist& msg) {
    RCLCPP_INFO(this->get_logger(), "CMD_CALLBACK");
	
    auto vel = msg;

    if (limit_vels_) {
        vel.linear.x = vel.linear.x > max_x_vel_ ? max_x_vel_ : vel.linear.x;
        vel.linear.x = vel.linear.x < -max_x_vel_ ? -max_x_vel_ : vel.linear.x;

        vel.angular.z = vel.angular.z > max_yaw_vel_ ? max_yaw_vel_ : vel.angular.z;
        vel.angular.z = vel.angular.z < -max_yaw_vel_ ? -max_yaw_vel_ : vel.angular.z;
    }

    auto& vx = vel.linear.x;
    auto& wz = vel.angular.z;

    float wl = (vx - wz * l_ / 2.) / r_;
    float wr = (vx + wz * l_ / 2.) / r_;

    std_msgs::msg::Float32 msg_l, msg_r;

    msg_l.data = wl;
    cmv_vel_left_pub_->publish(msg_l);

    msg_r.data = wr;
    cmv_vel_right_pub_->publish(msg_r);
}


#if false
void DriveController::pidCallback(const robot_msgs::msg::Float32Vector& msg) {
    kp_ = msg.data[0];
    ki_ = msg.data[1];
    kd_ = msg.data[2];

    cmdVelCallback(geometry_msgs::msg::Twist());
}
#endif


void DriveController::resetOdomCallback(const std_msgs::msg::Bool& msg) {
    if (msg.data == false) {
        return;
    }

    delay(500);

    prev_X_ = reset_X_;
    prev_P_ = reset_P_;

    RCLCPP_INFO(this->get_logger(), "ODOMETRY RESET");
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveController>()); 
    rclcpp::shutdown();

    return 0;
}
