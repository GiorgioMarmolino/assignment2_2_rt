#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class ROS2Robot : public rclcpp::Node
{
public:
    ROS2Robot()
    : Node("Robot_control")
    {
        motion_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ROS2Robot::getOdomClbk, this, _1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motion_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    void getOdomClbk(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        auto pos = odom->pose.pose.position;
        auto orient = odom->pose.pose.orientation;
        geometry_msgs::msg::Twist speed;

        RCLCPP_INFO(this->get_logger(), "Position: x=%.1f, y=%.1f, z=%.1f", pos.x, pos.y, pos.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: x=%.1f, y=%.1f, z=%.1f, w=%.1f", orient.x, orient.y, orient.z, orient.w);

        if (pos.x > 9.0){speed.linear.x = 1.0; speed.angular.z = 1.0;}
        else if (pos.x < 1.0){speed.linear.x = 1.0; speed.angular.z = -1.0;}
        else{speed.linear.x = 1.0; speed.angular.z = 0.0;}
        motion_pub->publish(speed);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.1f, angular.z=%.1f", speed.linear.x, speed.angular.z);
    }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto RobotNode = std::make_shared<ROS2Robot>();
    rclcpp::spin(RobotNode);
    rclcpp::shutdown();
    return 0;
}
