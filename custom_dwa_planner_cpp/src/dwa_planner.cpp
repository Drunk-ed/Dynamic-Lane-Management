#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <memory>
#include <limits>

namespace angles {

double shortest_angular_distance(double from, double to)
{
    double diff = to - from;

    while (diff > M_PI) {
        diff -= 2.0 * M_PI;
    }

    while (diff < -M_PI) {
        diff += 2.0 * M_PI;
    }

    return diff;
}

}  // namespace angles


class DWALocalPlanner : public rclcpp::Node
{
public:

    struct Trajectory
    {
        std::vector<std::pair<double, double>> path;
        double speed;
        double turn_rate;
        double score;
    };

    DWALocalPlanner()
        : Node("dwa_planner")
    {
        // ============================================================
        // Parameters
        // ============================================================

        this->declare_parameter("max_speed", 0.15);
        this->declare_parameter("min_speed", 0.0);
        this->declare_parameter("max_turn_rate", 1.0);
        this->declare_parameter("step_time", 0.2);
        this->declare_parameter("safety_margin", 0.15);
        this->declare_parameter("goal_tolerance", 0.20);
        this->declare_parameter("sim_samples", 15);
        this->declare_parameter("eval_samples", 80);
        this->declare_parameter("robot_radius", 0.12);

        this->declare_parameter("goal_x", 2.0);
        this->declare_parameter("goal_y", 0.0);

        this->declare_parameter("slow_down_distance", 0.75);
        this->declare_parameter("max_acceleration", 0.1);
        this->declare_parameter("max_deceleration", 0.2);
        this->declare_parameter("max_turn_acceleration", 1.0);

        this->declare_parameter("goal_weight", 3.0);
        this->declare_parameter("heading_weight", 1.5);
        this->declare_parameter("obstacle_weight", 3.0);
        this->declare_parameter("stopping_buffer", 0.1);

        // ============================================================
        // Load parameters
        // ============================================================

        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_turn_ = this->get_parameter("max_turn_rate").as_double();
        step_time_ = this->get_parameter("step_time").as_double();

        safety_margin_ =
            this->get_parameter("safety_margin").as_double();

        goal_tolerance_ =
            this->get_parameter("goal_tolerance").as_double();

        sim_samples_ =
            this->get_parameter("sim_samples").as_int();

        eval_samples_ =
            this->get_parameter("eval_samples").as_int();

        robot_radius_ =
            this->get_parameter("robot_radius").as_double();

        slow_down_distance_ =
            this->get_parameter("slow_down_distance").as_double();

        max_acceleration_ =
            this->get_parameter("max_acceleration").as_double();

        max_deceleration_ =
            this->get_parameter("max_deceleration").as_double();

        max_turn_acceleration_ =
            this->get_parameter("max_turn_acceleration").as_double();

        goal_weight_ =
            this->get_parameter("goal_weight").as_double();

        heading_weight_ =
            this->get_parameter("heading_weight").as_double();

        obstacle_weight_ =
            this->get_parameter("obstacle_weight").as_double();

        stopping_buffer_ =
            this->get_parameter("stopping_buffer").as_double();

        // ============================================================
        // Subscribers
        // ============================================================

        // Odometry
        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "odom",
                10,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    odom_data_ = msg;
                });

        // LiDAR
        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                10,
                [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
                {
                    scan_data_ = msg;
                });

        // ============================================================
        // Dynamic goal subscriber
        //
        // Relative topic name means:
        //
        // /divider_01/goal_pose
        // /divider_02/goal_pose
        // /divider_03/goal_pose
        //
        // when the DWA node is launched inside each namespace.
        // ============================================================

        goal_sub_ =
            this->create_subscription<geometry_msgs::msg::Pose2D>(
                "goal_pose",
                10,
                [this](const geometry_msgs::msg::Pose2D::SharedPtr msg)
                {
                    set_goal(msg->x, msg->y);
                });

        // ============================================================
        // Publishers
        // ============================================================

        cmd_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10);

        marker_pub_ =
            this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                "visualization_marker_array",
                10);

        // ============================================================
        // Initial goal
        // ============================================================

        goal_x_ =
            this->get_parameter("goal_x").as_double();

        goal_y_ =
            this->get_parameter("goal_y").as_double();

        RCLCPP_INFO(
            this->get_logger(),
            "Initial goal set to (%.2f, %.2f)",
            goal_x_,
            goal_y_);

        RCLCPP_INFO(
            this->get_logger(),
            "Dynamic goal topic enabled: goal_pose");

        // ============================================================
        // Random number generator
        // ============================================================

        gen_.seed(rd_());

        // ============================================================
        // Initial velocity
        // ============================================================

        current_speed_ = 0.0;
        current_turn_rate_ = 0.0;

        // ============================================================
        // Control timer
        // ============================================================

        timer_ =
            this->create_wall_timer(
                std::chrono::duration<double>(step_time_),
                [this]()
                {
                    control_loop();
                });
    }


private:

    // ================================================================
    // ROS interfaces
    // ================================================================

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;


    // ================================================================
    // Data storage
    // ================================================================

    nav_msgs::msg::Odometry::SharedPtr odom_data_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_data_;


    // ================================================================
    // Configuration
    // ================================================================

    double max_speed_{0.15};

    double min_speed_{0.0};

    double max_turn_{1.0};

    double step_time_{0.2};

    double safety_margin_{0.15};

    double goal_tolerance_{0.20};

    double robot_radius_{0.12};

    double slow_down_distance_{0.75};

    double max_acceleration_{0.1};

    double max_deceleration_{0.2};

    double max_turn_acceleration_{1.0};

    double goal_weight_{3.0};

    double heading_weight_{1.5};

    double obstacle_weight_{3.0};

    double stopping_buffer_{0.1};

    int sim_samples_{15};

    int eval_samples_{80};


    // ================================================================
    // Current velocity state
    // ================================================================

    double current_speed_{0.0};

    double current_turn_rate_{0.0};


    // ================================================================
    // Goal
    // ================================================================

    double goal_x_{2.0};

    double goal_y_{0.0};

    bool goal_reached_{false};


    // ================================================================
    // Random generator
    // ================================================================

    std::random_device rd_;

    std::mt19937 gen_;


    // ================================================================
    // Get yaw from odometry
    // ================================================================

    double get_yaw_from_odom()
    {
        if (!odom_data_) {
            return 0.0;
        }

        tf2::Quaternion q(
            odom_data_->pose.pose.orientation.x,
            odom_data_->pose.pose.orientation.y,
            odom_data_->pose.pose.orientation.z,
            odom_data_->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll;
        double pitch;
        double yaw;

        m.getRPY(roll, pitch, yaw);

        return yaw;
    }


    // ================================================================
    // Simulate trajectory
    // ================================================================

    Trajectory simulate_trajectory(
        double speed,
        double turn_rate)
    {
        Trajectory traj;

        traj.speed = speed;
        traj.turn_rate = turn_rate;
        traj.score = -std::numeric_limits<double>::infinity();

        if (!odom_data_) {
            return traj;
        }

        double x =
            odom_data_->pose.pose.position.x;

        double y =
            odom_data_->pose.pose.position.y;

        double yaw =
            get_yaw_from_odom();


        for (int i = 0; i < sim_samples_; ++i) {

            yaw += turn_rate * step_time_;

            x +=
                speed *
                std::cos(yaw) *
                step_time_;

            y +=
                speed *
                std::sin(yaw) *
                step_time_;

            traj.path.emplace_back(x, y);
        }

        return traj;
    }


    // ================================================================
    // Collision checking
    // ================================================================

    double check_collision(
        const std::vector<std::pair<double, double>>& path)
    {
        if (!scan_data_ || !odom_data_) {
            return -std::numeric_limits<double>::infinity();
        }

        if (scan_data_->ranges.empty()) {
            return 0.0;
        }

        double collision_penalty = 0.0;

        double min_distance =
            std::numeric_limits<double>::infinity();

        const double total_margin =
            safety_margin_ + robot_radius_;


        for (const auto& point : path) {

            double dx =
                point.first -
                odom_data_->pose.pose.position.x;

            double dy =
                point.second -
                odom_data_->pose.pose.position.y;

            double dist =
                std::hypot(dx, dy);

            double angle =
                std::atan2(dy, dx) -
                get_yaw_from_odom();

            angle =
                angles::shortest_angular_distance(
                    0.0,
                    angle);


            if (std::abs(angle) > scan_data_->angle_max) {
                continue;
            }


            if (scan_data_->angle_increment == 0.0) {
                continue;
            }


            int raw_idx =
                static_cast<int>(
                    (angle - scan_data_->angle_min) /
                    scan_data_->angle_increment);

            raw_idx =
                std::clamp(
                    raw_idx,
                    0,
                    static_cast<int>(scan_data_->ranges.size()) - 1);


            size_t idx =
                static_cast<size_t>(raw_idx);


            double range =
                static_cast<double>(
                    scan_data_->ranges[idx]);


            if (std::isfinite(range) &&
                range < dist + total_margin)
            {
                double penetration =
                    (dist + total_margin) - range;

                collision_penalty +=
                    -obstacle_weight_ *
                    1000.0 *
                    (1.0 + penetration);

                min_distance =
                    std::min(min_distance, range);
            }
        }


        if (min_distance <
            total_margin * 1.5)
        {
            collision_penalty +=
                -obstacle_weight_ *
                5000.0 *
                (1.5 -
                 min_distance / total_margin);
        }

        return collision_penalty;
    }


    // ================================================================
    // Evaluate trajectory
    // ================================================================

    double evaluate_trajectory(
        const Trajectory& traj)
    {
        if (!odom_data_ ||
            !scan_data_ ||
            traj.path.empty())
        {
            return -std::numeric_limits<double>::infinity();
        }

        double current_x =
            odom_data_->pose.pose.position.x;

        double current_y =
            odom_data_->pose.pose.position.y;

        double current_yaw =
            get_yaw_from_odom();


        // ------------------------------------------------------------
        // Goal distance at the end of the simulated trajectory
        // ------------------------------------------------------------

        double final_x =
            traj.path.back().first;

        double final_y =
            traj.path.back().second;


        double goal_dist =
            std::hypot(
                final_x - goal_x_,
                final_y - goal_y_);


        double goal_score =
            -goal_weight_ *
            20.0 *
            goal_dist;


        // ------------------------------------------------------------
        // Future heading
        //
        // IMPORTANT:
        // Evaluate the heading AFTER the simulated turn, rather
        // than repeatedly using the robot's current heading.
        // ------------------------------------------------------------

        double final_yaw =
            current_yaw +
            traj.turn_rate *
            step_time_ *
            static_cast<double>(sim_samples_);


        double target_angle =
            std::atan2(
                goal_y_ - final_y,
                goal_x_ - final_x);


        double angle_diff =
            angles::shortest_angular_distance(
                final_yaw,
                target_angle);


        double heading_score =
            -heading_weight_ *
            8.0 *
            std::abs(angle_diff);


        // ------------------------------------------------------------
        // Obstacle collision score
        // ------------------------------------------------------------

        double collision_score =
            check_collision(traj.path);


        // ------------------------------------------------------------
        // Smoothness
        // ------------------------------------------------------------

        double speed_diff =
            std::abs(
                traj.speed -
                current_speed_);

        double turn_diff =
            std::abs(
                traj.turn_rate -
                current_turn_rate_);


        double smoothness_score =
            -0.5 * speed_diff -
            0.8 * turn_diff;


        // ------------------------------------------------------------
        // Speed preference
        // ------------------------------------------------------------

        double speed_score = 0.0;

        if (traj.speed > max_speed_ * 0.6) {

            speed_score = 1.0;

        }
        else if (traj.speed > max_speed_ * 0.3) {

            speed_score = 0.5;
        }


        // ------------------------------------------------------------
        // Slow down near goal
        // ------------------------------------------------------------

        double slow_down_factor = 1.0;

        if (goal_dist < slow_down_distance_) {

            slow_down_factor =
                goal_dist /
                slow_down_distance_;

            speed_score *=
                slow_down_factor;
        }


        return
            goal_score +
            heading_score +
            collision_score +
            smoothness_score +
            speed_score;
    }


    // ================================================================
    // Visualize trajectories
    // ================================================================

    void visualize_trajectories(
        const std::vector<Trajectory>& trajectories,
        const Trajectory& best_traj)
    {
        visualization_msgs::msg::MarkerArray marker_array;


        // ------------------------------------------------------------
        // Clear old markers
        // ------------------------------------------------------------

        visualization_msgs::msg::Marker clear_marker;

        clear_marker.action =
            visualization_msgs::msg::Marker::DELETEALL;

        marker_array.markers.push_back(
            clear_marker);


        if (odom_data_) {

            // ========================================================
            // Safety margin
            // ========================================================

            visualization_msgs::msg::Marker safety_marker;

            safety_marker.header.frame_id =
                odom_data_->header.frame_id;

            safety_marker.header.stamp = now();

            safety_marker.ns =
                "safety_margin";

            safety_marker.id = 0;

            safety_marker.type =
                visualization_msgs::msg::Marker::SPHERE;

            safety_marker.action =
                visualization_msgs::msg::Marker::ADD;

            safety_marker.scale.x =
                (safety_margin_ + robot_radius_) * 2.0;

            safety_marker.scale.y =
                (safety_margin_ + robot_radius_) * 2.0;

            safety_marker.scale.z = 0.1;

            safety_marker.color.a = 0.2;

            safety_marker.color.r = 1.0;

            safety_marker.color.g = 0.0;

            safety_marker.color.b = 0.0;

            safety_marker.pose.position.x =
                odom_data_->pose.pose.position.x;

            safety_marker.pose.position.y =
                odom_data_->pose.pose.position.y;

            marker_array.markers.push_back(
                safety_marker);


            // ========================================================
            // Goal marker
            // ========================================================

            visualization_msgs::msg::Marker goal_marker;

            goal_marker.header.frame_id =
                odom_data_->header.frame_id;

            goal_marker.header.stamp = now();

            goal_marker.ns = "goal";

            goal_marker.id = 0;

            goal_marker.type =
                visualization_msgs::msg::Marker::SPHERE;

            goal_marker.action =
                visualization_msgs::msg::Marker::ADD;

            goal_marker.scale.x =
                goal_tolerance_ * 2.2;

            goal_marker.scale.y =
                goal_tolerance_ * 2.2;

            goal_marker.scale.z = 0.1;

            goal_marker.color.a = 0.5;

            goal_marker.color.r = 0.0;

            goal_marker.color.g = 1.0;

            goal_marker.color.b = 0.0;

            goal_marker.pose.position.x =
                goal_x_;

            goal_marker.pose.position.y =
                goal_y_;

            marker_array.markers.push_back(
                goal_marker);


            // ========================================================
            // Stopping buffer
            // ========================================================

            visualization_msgs::msg::Marker buffer_marker;

            buffer_marker.header.frame_id =
                odom_data_->header.frame_id;

            buffer_marker.header.stamp = now();

            buffer_marker.ns =
                "stopping_buffer";

            buffer_marker.id = 0;

            buffer_marker.type =
                visualization_msgs::msg::Marker::SPHERE;

            buffer_marker.action =
                visualization_msgs::msg::Marker::ADD;

            buffer_marker.scale.x =
                stopping_buffer_ * 2.0;

            buffer_marker.scale.y =
                stopping_buffer_ * 2.0;

            buffer_marker.scale.z = 0.1;

            buffer_marker.color.a = 0.3;

            buffer_marker.color.r = 1.0;

            buffer_marker.color.g = 1.0;

            buffer_marker.color.b = 0.0;

            buffer_marker.pose.position.x =
                goal_x_;

            buffer_marker.pose.position.y =
                goal_y_;

            marker_array.markers.push_back(
                buffer_marker);
        }


        // ============================================================
        // All trajectories
        // ============================================================

        int id = 0;

        for (const auto& traj : trajectories) {

            visualization_msgs::msg::Marker marker;

            marker.header.frame_id =
                odom_data_
                ? odom_data_->header.frame_id
                : "odom";

            marker.header.stamp = now();

            marker.ns =
                "trajectories";

            marker.id = id++;

            marker.type =
                visualization_msgs::msg::Marker::LINE_STRIP;

            marker.action =
                visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 0.02;

            marker.color.a = 0.3;

            marker.color.r = 0.0;

            marker.color.g = 0.5;

            marker.color.b = 1.0;


            for (const auto& point : traj.path) {

                geometry_msgs::msg::Point p;

                p.x = point.first;

                p.y = point.second;

                p.z = 0.0;

                marker.points.push_back(p);
            }

            marker_array.markers.push_back(
                marker);
        }


        // ============================================================
        // Best trajectory
        // ============================================================

        if (!best_traj.path.empty()) {

            visualization_msgs::msg::Marker best_marker;

            best_marker.header.frame_id =
                odom_data_
                ? odom_data_->header.frame_id
                : "odom";

            best_marker.header.stamp = now();

            best_marker.ns =
                "best_trajectory";

            best_marker.id = 0;

            best_marker.type =
                visualization_msgs::msg::Marker::LINE_STRIP;

            best_marker.action =
                visualization_msgs::msg::Marker::ADD;

            best_marker.scale.x = 0.05;

            best_marker.color.a = 1.0;

            best_marker.color.r = 0.0;

            best_marker.color.g = 1.0;

            best_marker.color.b = 0.0;


            for (const auto& point :
                 best_traj.path)
            {
                geometry_msgs::msg::Point p;

                p.x = point.first;

                p.y = point.second;

                p.z = 0.0;

                best_marker.points.push_back(p);
            }

            marker_array.markers.push_back(
                best_marker);
        }


        marker_pub_->publish(
            marker_array);
    }


    // ================================================================
    // Main DWA control loop
    // ================================================================

    void control_loop()
    {
        if (!odom_data_ ||
            !scan_data_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Waiting for odometry and laser scan data...");

            return;
        }


        // ------------------------------------------------------------
        // Distance to current goal
        // ------------------------------------------------------------

        double dist_to_goal =
            std::hypot(
                goal_x_ -
                    odom_data_->pose.pose.position.x,

                goal_y_ -
                    odom_data_->pose.pose.position.y);


        // ------------------------------------------------------------
        // Stopping zones
        // ------------------------------------------------------------

        const double stopping_distance =
            stopping_buffer_;

        const double slow_down_zone =
            goal_tolerance_;


        // ============================================================
        // Goal reached
        //
        // Stop as soon as the robot enters the goal tolerance.
        // This prevents the robot from crossing the goal and then
        // suddenly seeing the target behind it at approximately 180°.
        // ============================================================

        if (dist_to_goal <= goal_tolerance_) {

            if (!goal_reached_) {

                goal_reached_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Goal reached at distance %.3f m. Stopping.",
                    dist_to_goal);
            }


            geometry_msgs::msg::Twist stop_cmd;

            stop_cmd.linear.x = 0.0;

            stop_cmd.angular.z = 0.0;

            cmd_pub_->publish(stop_cmd);


            current_speed_ = 0.0;

            current_turn_rate_ = 0.0;

            return;
        }


        // ============================================================
        // Slow down before reaching the goal
        //
        // The slowdown zone is now larger than the stopping zone.
        // ============================================================

        else if (dist_to_goal <
                 slow_down_distance_)
        {
            double slow_down_factor =
                std::clamp(
                    dist_to_goal /
                    slow_down_distance_,
                    0.0,
                    1.0);


            geometry_msgs::msg::Twist cmd;

            // Very slow approach near the target.
            cmd.linear.x =
                std::min(
                    current_speed_,
                    max_speed_ *
                    0.20 *
                    slow_down_factor);


            cmd.angular.z =
                current_turn_rate_ *
                0.3;


            cmd_pub_->publish(
                cmd);


            current_speed_ =
                cmd.linear.x;

            current_turn_rate_ =
                cmd.angular.z;

            return;
        }


        // Goal moved / new goal received
        goal_reached_ = false;


        // ============================================================
        // HEADING CONTROL
        //
        // Prevent the robot from driving while the target direction
        // is significantly different from its current heading.
        //
        // This avoids the circular/spiral behaviour seen when DWA
        // keeps selecting curved forward trajectories.
        // ============================================================

        double current_x =
            odom_data_->pose.pose.position.x;

        double current_y =
            odom_data_->pose.pose.position.y;

        double current_yaw =
            get_yaw_from_odom();

        double target_angle =
            std::atan2(
                goal_y_ - current_y,
                goal_x_ - current_x);

        double angle_error =
            angles::shortest_angular_distance(
                current_yaw,
                target_angle);

        const double rotate_threshold =
            15.0 * M_PI / 180.0;

        const double drive_threshold =
            8.0 * M_PI / 180.0;


        // ------------------------------------------------------------
        // Large heading error:
        // rotate in place.
        // ------------------------------------------------------------

        if (std::abs(angle_error) >
            rotate_threshold)
        {
            geometry_msgs::msg::Twist align_cmd;

            double angular_cmd =
                2.0 * angle_error;

            angular_cmd =
                std::max(
                    -max_turn_,
                    std::min(
                        angular_cmd,
                        max_turn_));

            align_cmd.linear.x = 0.0;
            align_cmd.angular.z = angular_cmd;

            cmd_pub_->publish(
                align_cmd);

            current_speed_ = 0.0;
            current_turn_rate_ = angular_cmd;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "ALIGNING: error = %.1f deg",
                angle_error * 180.0 / M_PI);

            return;
        }


        // ============================================================
        // Generate trajectories
        // ============================================================

        std::vector<Trajectory> trajectories;

        Trajectory best_traj;

        double best_score =
            -std::numeric_limits<double>::infinity();


        // ============================================================
        // Dynamic Window
        // ============================================================

        double min_speed_window =
            std::max(
                0.0,
                current_speed_ -
                    max_acceleration_ *
                    step_time_);


        double max_speed_window =
            std::min(
                max_speed_,
                current_speed_ +
                    max_acceleration_ *
                    step_time_);


        double min_turn_window =
            std::max(
                -max_turn_,
                current_turn_rate_ -
                    max_turn_acceleration_ *
                    step_time_);


        double max_turn_window =
            std::min(
                max_turn_,
                current_turn_rate_ +
                    max_turn_acceleration_ *
                    step_time_);


        // ------------------------------------------------------------
        // Deterministic DWA sampling
        //
        // This avoids random trajectory changes from one cycle
        // to the next.
        // ------------------------------------------------------------

        const int speed_samples = 6;
        const int turn_samples = 21;


        for (int i = 0;
             i < speed_samples;
             ++i)
        {
            double speed = 0.0;

            if (speed_samples == 1)
            {
                speed = min_speed_window;
            }
            else
            {
                speed =
                    min_speed_window +
                    (max_speed_window -
                     min_speed_window) *
                    static_cast<double>(i) /
                    static_cast<double>(speed_samples - 1);
            }


            for (int j = 0;
                 j < turn_samples;
                 ++j)
            {
                double turn = 0.0;

                if (turn_samples == 1)
                {
                    turn = min_turn_window;
                }
                else
                {
                    turn =
                        min_turn_window +
                        (max_turn_window -
                         min_turn_window) *
                        static_cast<double>(j) /
                        static_cast<double>(turn_samples - 1);
                }


                Trajectory traj =
                    simulate_trajectory(
                        speed,
                        turn);


                traj.score =
                    evaluate_trajectory(
                        traj);


                trajectories.push_back(
                    traj);


                if (traj.score >
                    best_score)
                {
                    best_score =
                        traj.score;

                    best_traj =
                        traj;
                }
            }
        }


        // ============================================================
        // Visualization
        // ============================================================

        visualize_trajectories(
            trajectories,
            best_traj);


        // ============================================================
        // Execute best trajectory
        // ============================================================

        if (std::isfinite(best_score)) {

            // --------------------------------------------------------
            // Linear acceleration limit
            // --------------------------------------------------------

            double speed_diff =
                best_traj.speed -
                current_speed_;


            double max_speed_change =
                (speed_diff > 0.0)
                ? max_acceleration_ *
                    step_time_
                : max_deceleration_ *
                    step_time_;


            double new_speed;

            if (std::abs(speed_diff) <
                max_speed_change)
            {
                new_speed =
                    best_traj.speed;
            }
            else
            {
                new_speed =
                    current_speed_ +
                    std::copysign(
                        max_speed_change,
                        speed_diff);
            }


            // --------------------------------------------------------
            // Angular acceleration limit
            // --------------------------------------------------------

            double turn_diff =
                best_traj.turn_rate -
                current_turn_rate_;


            double max_turn_change =
                max_turn_acceleration_ *
                step_time_;


            double new_turn;

            if (std::abs(turn_diff) <
                max_turn_change)
            {
                new_turn =
                    best_traj.turn_rate;
            }
            else
            {
                new_turn =
                    current_turn_rate_ +
                    std::copysign(
                        max_turn_change,
                        turn_diff);
            }


            // --------------------------------------------------------
            // Slow down near goal
            // --------------------------------------------------------

            if (dist_to_goal <
                slow_down_distance_)
            {
                double slow_down_factor =
                    dist_to_goal /
                    slow_down_distance_;


                new_speed =
                    std::min(
                        new_speed,
                        max_speed_ *
                        slow_down_factor);


                if (dist_to_goal <
                    slow_down_distance_ /
                    2.0)
                {
                    new_speed =
                        std::min(
                            new_speed,
                            max_speed_ *
                            0.3 *
                            slow_down_factor);
                }
            }


            // Never command negative velocity
            new_speed =
                std::max(
                    0.0,
                    std::min(
                        new_speed,
                        max_speed_));


            // Limit angular velocity
            new_turn =
                std::max(
                    -max_turn_,
                    std::min(
                        new_turn,
                        max_turn_));


            // --------------------------------------------------------
            // Publish command
            // --------------------------------------------------------

            // ========================================================
            // Goal-directed forward control
            //
            // Once reasonably aligned, do not allow the random/DWA
            // angular sample to pull the robot into a large circle.
            //
            // DWA still determines the forward velocity and trajectory
            // evaluation, while the final angular command is coupled
            // to the actual goal direction.
            // ========================================================

            double final_heading_error =
                angles::shortest_angular_distance(
                    get_yaw_from_odom(),
                    std::atan2(
                        goal_y_ -
                            odom_data_->pose.pose.position.y,
                        goal_x_ -
                            odom_data_->pose.pose.position.x));


            if (std::abs(final_heading_error) >
                drive_threshold)
            {
                new_speed =
                    std::min(
                        new_speed,
                        max_speed_ * 0.25);
            }


            // Proportional goal-heading correction.
            double goal_turn =
                1.8 * final_heading_error;


            goal_turn =
                std::max(
                    -max_turn_,
                    std::min(
                        goal_turn,
                        max_turn_));


            // Near the goal, prioritize pointing at the goal.
            if (dist_to_goal <
                slow_down_distance_)
            {
                new_turn =
                    goal_turn;
            }
            else
            {
                // Blend DWA steering with goal steering.
                new_turn =
                    0.35 * new_turn +
                    0.65 * goal_turn;
            }


            // If the heading error becomes large again, stop
            // forward motion rather than allowing another circle.
            if (std::abs(final_heading_error) >
                rotate_threshold)
            {
                new_speed = 0.0;
                new_turn = goal_turn;
            }


            geometry_msgs::msg::Twist cmd;

            cmd.linear.x =
                std::max(
                    0.0,
                    std::min(
                        new_speed,
                        max_speed_));

            cmd.angular.z =
                std::max(
                    -max_turn_,
                    std::min(
                        new_turn,
                        max_turn_));

            cmd_pub_->publish(
                cmd);


            // --------------------------------------------------------
            // Update internal velocity state
            // --------------------------------------------------------

            current_speed_ =
                new_speed;

            current_turn_rate_ =
                new_turn;
        }

        // ============================================================
        // No valid trajectory
        // ============================================================

        else {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "No valid trajectory found! Stopping.");


            geometry_msgs::msg::Twist stop_cmd;

            stop_cmd.linear.x = 0.0;

            stop_cmd.angular.z = 0.0;

            cmd_pub_->publish(
                stop_cmd);


            current_speed_ = 0.0;

            current_turn_rate_ = 0.0;
        }
    }


    // ================================================================
    // Set new goal
    // ================================================================

    void set_goal(
        double x,
        double y)
    {
        goal_x_ = x;

        goal_y_ = y;

        goal_reached_ = false;


        // Reset velocity state when assigning a completely
        // new navigation target.
        current_speed_ = 0.0;

        current_turn_rate_ = 0.0;


        RCLCPP_INFO(
            this->get_logger(),
            "New goal set to (%.2f, %.2f)",
            goal_x_,
            goal_y_);
    }
};


// ====================================================================
// Main
// ====================================================================

int main(
    int argc,
    char** argv)
{
    rclcpp::init(
        argc,
        argv);


    auto node =
        std::make_shared<DWALocalPlanner>();


    rclcpp::spin(node);


    rclcpp::shutdown();

    return 0;
}
