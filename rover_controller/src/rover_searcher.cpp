#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

class RoverSearcher : public rclcpp::Node
{
public:
    RoverSearcher() 
        : Node("rover_searcher"),
          dist_from_center_(0.0), 
          phase_(0),
          rotation_sequence_step_(0),
          distance_moved_(0.0), 
          angle_rotated_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RoverSearcher::odom_callback, this, std::placeholders::_1));
        aruco_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_position", 10, std::bind(&RoverSearcher::aruco_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RoverSearcher::control_loop, this));

        path_msg_.header.frame_id = "odom";
    }

private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        if (phase_ == 0) // Phase 0 - Initialize position
        {
            initial_x_ = msg->pose.pose.position.x;
            initial_y_ = msg->pose.pose.position.y;
            initial_yaw_ = get_yaw_from_quaternion(msg->pose.pose.orientation);
            last_known_coord_x_ = initial_x_;
            last_known_coord_y_ = initial_y_;
            phase_ = 1;
            return;
        }

        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_yaw_ = get_yaw_from_quaternion(msg->pose.pose.orientation);

        dist_from_center_ = get_euclidean_distance(current_x_, current_y_, last_known_coord_x_, last_known_coord_y_);
        distance_moved_ = get_euclidean_distance(current_x_, current_y_, initial_x_, initial_y_);

        angle_rotated_ = std::abs(current_yaw_ - initial_yaw_);
        if (angle_rotated_ > M_PI) // wrap yaw
        {
            angle_rotated_ = 2 * M_PI - angle_rotated_;
        }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose = msg->pose.pose;
        path_msg_.poses.push_back(pose_stamped);

        path_msg_.header.stamp = this->now();
        path_publisher_->publish(path_msg_);
    }

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        detected_aruco_ = true;
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        RCLCPP_INFO(this->get_logger(), "Search completed. ArUco found at (%f, %f, %f)", goal_x_, goal_y_, msg->pose.position.z);
    }

    void control_loop()
    {
        auto message = geometry_msgs::msg::Twist();

        if(detected_aruco_ == true && phase_ != 5)
        {
            phase_ = 5;
            desired_yaw = std::atan2(goal_x_ - current_y_, goal_y_ - current_x_);
            initial_yaw_ = current_yaw_;
        }

        if (phase_ == 5) // going to aruco
        {

            if (std::abs(current_yaw_ - desired_yaw) >= 0.1)
            {
                message.linear.x = 0.0;
                message.angular.z = turning_speed;
            }
            else if (get_euclidean_distance(current_x_, current_y_, goal_x_, goal_y_) >= 0.1)
            {
                message.linear.x = forw_speed;
                message.angular.z = 0.0;
            }
            else
            {
                message.linear.x = 0.0;
                message.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Reached Aruco.");
            }
        }   
        else if (dist_from_center_ >= dist_thresh && phase_ != 4)
        {
            desired_yaw = std::atan2(last_known_coord_y_ - current_y_, last_known_coord_x_ - current_x_);
            initial_yaw_ = current_yaw_;
            RCLCPP_INFO(this->get_logger(), "Out of Bounds, Resetting Search...");
            side_length_ = orig_side_len;
            phase_ = 4;
        }

        if (phase_ == 1) // Phase 1 - Move forward
        {
            message.linear.x = forw_speed;
            message.angular.z = 0.0;
            if (distance_moved_ >= side_length_)
            {
                phase_ = 2;
                rotation_sequence_step_ = 0;
                initial_yaw_ = current_yaw_;
            }
        }
        else if (phase_ == 2) // Phase 2 - Perform rotation sequence
        {
            message.linear.x = 0.0;

            if (rotation_sequence_step_ < rotation_angles_.size())
            {
                double target_angle = rotation_angles_[rotation_sequence_step_];
                desired_yaw = initial_yaw_ + target_angle;
                if (desired_yaw > M_PI) desired_yaw -= 2 * M_PI;
                if (desired_yaw < -M_PI) desired_yaw += 2 * M_PI;

                double yaw_diff = std::abs(current_yaw_ - desired_yaw);
                if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;

                if (yaw_diff >= 0.1)
                {
                    message.angular.z = (target_angle > 0) ? turning_speed : -turning_speed;
                }
                else
                {
                    rotation_sequence_step_++;
                    initial_yaw_ = current_yaw_;
                }
            }
            else
            {
                phase_ = 3;
                rotation_sequence_step_ = 0;
            }
        }
        else if (phase_ == 3) // Phase 3 - Rotate to hexagon angle and move forward
        {
            message.linear.x = 0.0;
            message.angular.z = turning_speed;

            if (angle_rotated_ >= angle_to_turn_)
            {
                phase_ = 1;
                initial_x_ = current_x_;
                initial_y_ = current_y_;
                side_length_ += exp_step;
            }
        }
        else if (phase_ == 4) //Out of Bounds
        {
            if (std::abs(current_yaw_ - desired_yaw) >= 0.1)
            {
                message.linear.x = 0.0;
                message.angular.z = turning_speed;
            }
            else if (get_euclidean_distance(current_x_, current_y_, last_known_coord_x_, last_known_coord_y_) >= 0.1)
            {
                message.linear.x = forw_speed;
                message.angular.z = 0.0;
            }
            else
            {
                phase_ = 0;
            }
        }

        publisher_->publish(message);
    }

    double get_euclidean_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    const double exp_step = 0.3;
    const double orig_side_len = 0.6;
    const double dist_thresh = 2.0;
    const double angle_to_turn_ = M_PI / 3;
    double dist_from_center_, last_known_coord_x_ = 0.0, last_known_coord_y_ = 0.0;
    int phase_;
    double side_length_ = 0.6;
    size_t rotation_sequence_step_;
    std::vector<double> rotation_angles_ = {-M_PI, M_PI, M_PI, -M_PI};
    double initial_x_, initial_y_, initial_yaw_;
    double current_x_, current_y_, current_yaw_;
    double distance_moved_, angle_rotated_;
    double desired_yaw;
    const double forw_speed = 0.2;
    const double turning_speed = 0.5;
    double goal_x_, goal_y_;

    bool detected_aruco_ = false;

    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverSearcher>());
    rclcpp::shutdown();
    return 0;
}
