#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono;

class StatePublisher : public rclcpp::Node {
public:
    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()): Node("state_publisher", options) {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(), "Starting state publisher");

        loop_rate_ = std::make_shared<rclcpp::Rate>(33ms);

        timer_ = this->create_wall_timer(33ms, std::bind(&StatePublisher::publish, this));
    }

    void publish();
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Rate::SharedPtr loop_rate_;
    rclcpp::TimerBase::SharedPtr timer_;

    const double degree=M_PI/180.0;
    double tilt = 0.;
    double tinc = degree;
    double swivel = 0.;
    double angle = 0.;
    double height = 0.;
    double hinc = 0.005;
};

void StatePublisher::publish() {
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;

    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"swivel", "tilt", "periscope"};    
    joint_state.position = {swivel, tilt, height};

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "axis";

    t.transform.translation.x = cos(angle)*2;
    t.transform.translation.y = sin(angle)*2;
    t.transform.translation.z = 0.7;
    tf2::Quaternion q;

    q.setRPY(0, 0, angle+M_PI/2);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();


    tilt+=tinc;
    if (tilt<-0.5 || tilt>0.0) {
        tinc*=-1;
    }
    height += hinc;
    if (height>0.2 || height<0.0) {
        hinc*=-1;
    }

    swivel+=degree;
    angle+=degree;

    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);

    RCLCPP_INFO(this->get_logger(), "Publishing joint state");

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}