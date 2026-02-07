#include "teleop_ack_joy/TeleopAckJoyNode_node.hpp"

// For _1
using namespace std::placeholders;

TeleopAckJoyNode::TeleopAckJoyNode(const rclcpp::NodeOptions& options) : Node("TeleopAckjoyNode", options) {
    min_axis_input = this->declare_parameter<float>("min_axis_input", -1.0);
    max_axis_input = this->declare_parameter<float>("max_axis_input", 1.0);
    min_steering_angle = this->declare_parameter<float>("min_steering_angle", -0.2733);
    max_steering_angle = this->declare_parameter<float>("max_steering_angle", 0.2733);
    max_speed = this->declare_parameter<float>("max_speed", 6.7056);
    throttle_axis = this->declare_parameter<int>("throttle_axis", 5);
    reverse_axis = this->declare_parameter<int>("reverse_axis", 2);
    steering_axis = this->declare_parameter<int>("steering_axis", 0);

    joy_sub =
        this->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, std::bind(&TeleopAckJoyNode::joy_cb, this, _1));

    ackermann_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ack_vel", 1);
}

void TeleopAckJoyNode::joy_cb(sensor_msgs::msg::Joy::SharedPtr outputs) {
    ackermann_msgs::msg::AckermannDrive command;

    // axes[i] == 1 if fully released, -1 if fully pressed
    // neutral == 0, fully backward = left fully pressed == -1, fully forward = right fully pressed == 1
    float throttle = (-outputs->axes.at(throttle_axis) + outputs->axes.at(reverse_axis)) / 2;
    float steering = outputs->axes.at(steering_axis);

    command.steering_angle =
        map_input(steering, min_axis_input, max_axis_input, min_steering_angle, max_steering_angle);
    command.speed = map_input(throttle, min_axis_input, max_axis_input, -max_speed, max_speed);

    ackermann_pub->publish(command);
}

float TeleopAckJoyNode::map_input(float in, float inMin, float inMax, float outMin, float outMax) {
    float slope = (outMax - outMin) / (inMax - inMin);
    float b = outMax - slope * inMax;

    return slope * in + b;
}
