#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <cmath>
#include <thread>

class TrackingGenerator : public rclcpp::Node {
public:
    TrackingGenerator() : Node("motion_generator_node"), 
                         trajectory_in_progress_(false), 
                         joint_states_ready_(false),
                         base_movement_in_progress_(false),
                         base_forward_(true),
                         tcp_error_threshold_(0.002),
                         base_target_x_(0.0),
                         base_start_x_(0.0) {
        
        // Declare parameters
        this->declare_parameter("target_position.x", 0.5);
        this->declare_parameter("target_position.y", 0.5);
        this->declare_parameter("target_position.z", 0.4);
        
        this->declare_parameter("arm_motion.velocity", 0.5);
        this->declare_parameter("arm_motion.acceleration", 0.5);
        
        this->declare_parameter("base_motion.velocity", 0.5);
        this->declare_parameter("base_motion.acceleration", 0.1);
        this->declare_parameter("base_motion.distance", 0.8);
        
        // Load parameters
        load_parameters();
        
        // Publishers
        arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/motion_library/arm_trajectory", 10);
        
        base_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/motion_library/base_trajectory", 10);

        // Visualization marker publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker", 10);

        this->declare_parameter<std::string>("robot_description", "");
        std::string robot_description;
        this->get_parameter("robot_description", robot_description);

        if (!kdl_parser::treeFromString(robot_description, tree_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree.");
            return;
        }

        // Create kinematic chain from world to tool0 (includes base + arm)
        std::string base_link = "world";
        std::string tip_link = "tool0";
        if (!tree_.getChain(base_link, tip_link, full_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from %s to %s", base_link.c_str(), tip_link.c_str());
            return;
        }

        // Also create arm-only chain for initial movement
        std::string arm_base = "base_link";
        if (!tree_.getChain(arm_base, tip_link, arm_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract arm chain from %s to %s", arm_base.c_str(), tip_link.c_str());
            return;
        }

        // Create solvers
        full_ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(full_chain_);
        full_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(full_chain_);
        arm_ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(arm_chain_);
        arm_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(arm_chain_);

        // Initialize joint positions arrays
        full_joint_positions_ = KDL::JntArray(full_chain_.getNrOfJoints());
        arm_joint_positions_ = KDL::JntArray(arm_chain_.getNrOfJoints());

        RCLCPP_INFO(this->get_logger(), "Full chain joints: %d, Arm chain joints: %d", 
                    full_chain_.getNrOfJoints(), arm_chain_.getNrOfJoints());

        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                update_joint_states(msg);
            });

        // Timer to check trajectory completion
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrackingGenerator::check_trajectory_status, this)
        );

        // Timer for real-time coordinated control (50Hz)
        coordinated_control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TrackingGenerator::execute_coordinated_movement, this)
        );

        // Timer for visualization marker publishing
        viz_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrackingGenerator::publish_target_marker, this)
        );

        // Timer for debugging TCP position
        debug_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TrackingGenerator::debug_tcp_position, this)
        );
    }

private:
    struct VelocityProfile {
        double t_accel;
        double t_const;
        double t_decel;
        double total_time;
        double max_velocity;
    };

    struct MotionParams {
        KDL::Vector target_position;
        double arm_velocity;
        double arm_acceleration;
        double base_velocity;
        double base_acceleration;
        double base_distance;
    };

    // Joint state management
    KDL::JntArray full_joint_positions_;
    KDL::JntArray arm_joint_positions_;
    bool joint_states_ready_ = false;

    // Trajectory state management
    bool trajectory_in_progress_;
    bool base_movement_in_progress_;
    bool base_forward_;
    double current_trajectory_duration_;
    rclcpp::Time trajectory_start_time_;
    
    // Store target world position
    KDL::Vector target_world_position_;
    KDL::JntArray final_arm_config_;
    bool target_position_set_ = false;
    double base_target_x_;
    double base_start_x_;
    rclcpp::Time base_movement_start_time_;

    // Motion parameters
    MotionParams params_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // KDL components
    KDL::Tree tree_;
    KDL::Chain full_chain_;
    KDL::Chain arm_chain_;
    
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> full_ik_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> full_fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> arm_ik_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> arm_fk_solver_;
    
    // Publishers
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr base_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr coordinated_control_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::TimerBase::SharedPtr debug_timer_;
    double tcp_error_threshold_;

    void load_parameters() {
        // Load target position
        params_.target_position.x(this->get_parameter("target_position.x").as_double());
        params_.target_position.y(this->get_parameter("target_position.y").as_double());
        params_.target_position.z(this->get_parameter("target_position.z").as_double());
        
        // Load arm motion parameters
        params_.arm_velocity = this->get_parameter("arm_motion.velocity").as_double();
        params_.arm_acceleration = this->get_parameter("arm_motion.acceleration").as_double();
        
        // Load base motion parameters
        params_.base_velocity = this->get_parameter("base_motion.velocity").as_double();
        params_.base_acceleration = this->get_parameter("base_motion.acceleration").as_double();
        params_.base_distance = this->get_parameter("base_motion.distance").as_double();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Loaded parameters:\n"
                   "  Target position: (%.3f, %.3f, %.3f)\n"
                   "  Arm velocity: %.3f m/s, Arm acceleration: %.3f m/s²\n"
                   "  Base velocity: %.3f m/s, Base acceleration: %.3f m/s²\n"
                   "  Base distance: %.3f m",
                   params_.target_position.x(), params_.target_position.y(), params_.target_position.z(),
                   params_.arm_velocity, params_.arm_acceleration,
                   params_.base_velocity, params_.base_acceleration,
                   params_.base_distance);
    }

    void debug_tcp_position() {
        if (!joint_states_ready_ || !target_position_set_) return;
        
        // Compute TCP position using full chain (world -> tool0)
        KDL::Frame tcp_world_full;
        int result = full_fk_solver_->JntToCart(full_joint_positions_, tcp_world_full);
        
        if (result >= 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "TCP World Position (full chain): (%.3f, %.3f, %.3f), Target: (%.3f, %.3f, %.3f)",
                       tcp_world_full.p.x(), tcp_world_full.p.y(), tcp_world_full.p.z(),
                       target_world_position_.x(), target_world_position_.y(), target_world_position_.z());
            
            // Also compute using base + arm separately for debugging
            KDL::Frame base_pose;
            base_pose.p = KDL::Vector(full_joint_positions_(0), full_joint_positions_(1), 0.0);
            base_pose.M = KDL::Rotation::RotZ(full_joint_positions_(2));
            
            KDL::Frame tcp_in_base;
            arm_fk_solver_->JntToCart(arm_joint_positions_, tcp_in_base);
            
            KDL::Frame tcp_world_separate = base_pose * tcp_in_base;
            
            RCLCPP_INFO(this->get_logger(), 
                       "TCP World Position (base+arm): (%.3f, %.3f, %.3f), Base: (%.3f, %.3f, %.3f)",
                       tcp_world_separate.p.x(), tcp_world_separate.p.y(), tcp_world_separate.p.z(),
                       base_pose.p.x(), base_pose.p.y(), base_pose.p.z());
        }
    }

    void publish_target_marker() {
        if (!target_position_set_) {
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "target_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = target_world_position_.x();
        marker.pose.position.y = target_world_position_.y();
        marker.pose.position.z = target_world_position_.z() + 0.4;
        
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }

    void execute_coordinated_movement() {
        if (!base_movement_in_progress_ || !joint_states_ready_ || !target_position_set_) {
            return;
        }

        // Get current positions
        double current_base_x = full_joint_positions_(0);
        double current_base_y = full_joint_positions_(1);
        
        // Check if we've reached the target
        if (std::abs(current_base_x - base_target_x_) < 0.001) {
            RCLCPP_INFO(this->get_logger(), "Base movement completed. Target reached.");
            base_movement_in_progress_ = false;
            
            // Verify final TCP position
            KDL::Frame final_tcp;
            full_fk_solver_->JntToCart(full_joint_positions_, final_tcp);
            double final_error = (final_tcp.p - target_world_position_).Norm();
            RCLCPP_INFO(this->get_logger(), "Final TCP error: %.4f m", final_error);
            
            // Start next movement after a delay
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            start_coordinated_movement();
            return;
        }

        // Calculate smooth velocity profile
        double elapsed = (this->get_clock()->now() - base_movement_start_time_).seconds();
        double total_distance = std::abs(base_target_x_ - base_start_x_);
        double max_velocity = params_.base_velocity;
        double acceleration = params_.base_acceleration;
        
        // Simple trapezoidal velocity profile
        double current_velocity = 0.0;
        double distance_traveled = std::abs(current_base_x - base_start_x_);
        double distance_remaining = std::abs(base_target_x_ - current_base_x);
        
        // Acceleration phase
        if (distance_traveled < total_distance * 0.3) {
            current_velocity = std::min(max_velocity, acceleration * elapsed);
        }
        // Deceleration phase
        else if (distance_remaining < total_distance * 0.3) {
            double decel_time = std::sqrt(2 * distance_remaining / acceleration);
            current_velocity = std::min(max_velocity, acceleration * decel_time);
        }
        // Constant velocity phase
        else {
            current_velocity = max_velocity;
        }
        
        // Apply direction
        if (base_target_x_ < current_base_x) {
            current_velocity = -current_velocity;
        }
        
        // Calculate next position (small step)
        double dt = 0.02; // 20ms
        double next_base_x = current_base_x + current_velocity * dt;
        
        // Clamp to target
        if ((base_target_x_ > base_start_x_ && next_base_x > base_target_x_) ||
            (base_target_x_ < base_start_x_ && next_base_x < base_target_x_)) {
            next_base_x = base_target_x_;
        }

        // First, verify current TCP position
        KDL::Frame current_tcp_world;
        full_fk_solver_->JntToCart(full_joint_positions_, current_tcp_world);
        double current_error = (current_tcp_world.p - target_world_position_).Norm();
        
        if (current_error > 0.005) { // 5mm threshold
            RCLCPP_WARN(this->get_logger(), 
                       "Large TCP error before update: %.4f m. Current TCP: (%.3f, %.3f, %.3f)",
                       current_error, current_tcp_world.p.x(), current_tcp_world.p.y(), current_tcp_world.p.z());
        }

        // Create base pose for IK
        KDL::Frame next_base_pose;
        next_base_pose.p = KDL::Vector(next_base_x, current_base_y, 0.0);
        next_base_pose.M = KDL::Rotation::RotZ(0.0);
        
        // Transform target from world to base frame
        KDL::Frame target_in_base = next_base_pose.Inverse() * 
                                   KDL::Frame(KDL::Rotation::Identity(), target_world_position_);
        
        // Solve IK for arm
        KDL::JntArray arm_solution(6);
        int result = arm_ik_solver_->CartToJnt(arm_joint_positions_, target_in_base, arm_solution);
        
        if (result >= 0) {
            // Verify solution by computing where TCP would be
            KDL::Frame tcp_in_base;
            arm_fk_solver_->JntToCart(arm_solution, tcp_in_base);
            KDL::Frame tcp_in_world = next_base_pose * tcp_in_base;
            double error = (tcp_in_world.p - target_world_position_).Norm();
            
            if (error < 0.01) { // 1cm tolerance
                publish_coordinated_trajectory_segment(next_base_x, current_base_y, arm_solution, 0.1);
                
                // Update arm joint positions for next iteration
                arm_joint_positions_ = arm_solution;
                
                if (static_cast<int>(elapsed * 50) % 25 == 0) { // Log every 0.5s
                    RCLCPP_INFO(this->get_logger(), 
                               "Movement: base=%.3f->%.3f, TCP error=%.4f m, TCP at (%.3f,%.3f,%.3f)",
                               current_base_x, base_target_x_, error,
                               tcp_in_world.p.x(), tcp_in_world.p.y(), tcp_in_world.p.z());
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                            "IK solution verification failed! Error: %.4f m. Target: (%.3f,%.3f,%.3f), Got: (%.3f,%.3f,%.3f)",
                            error, 
                            target_world_position_.x(), target_world_position_.y(), target_world_position_.z(),
                            tcp_in_world.p.x(), tcp_in_world.p.y(), tcp_in_world.p.z());
                
                // Try to maintain current arm configuration
                publish_coordinated_trajectory_segment(next_base_x, current_base_y, arm_joint_positions_, 0.1);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "IK failed for coordinated movement");
            // Maintain current arm configuration
            publish_coordinated_trajectory_segment(next_base_x, current_base_y, arm_joint_positions_, 0.1);
        }
    }

    void publish_coordinated_trajectory_segment(double base_x, double base_y, 
                                               const KDL::JntArray& arm_config, 
                                               double duration) {
        // Create base trajectory
        trajectory_msgs::msg::JointTrajectory base_traj;
        base_traj.header.stamp = this->get_clock()->now();
        base_traj.joint_names = {
            "world_to_base_translate_x",
            "world_to_base_translate_y", 
            "world_to_base_rotate_z"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint base_point;
        base_point.positions = {base_x, base_y, 0.0};
        base_point.time_from_start = rclcpp::Duration::from_seconds(duration);
        base_traj.points.push_back(base_point);
        
        // Create arm trajectory
        trajectory_msgs::msg::JointTrajectory arm_traj;
        arm_traj.header.stamp = this->get_clock()->now();
        arm_traj.joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint arm_point;
        arm_point.positions.resize(6);
        for (int i = 0; i < 6; ++i) {
            arm_point.positions[i] = arm_config(i);
        }
        arm_point.time_from_start = rclcpp::Duration::from_seconds(duration);
        arm_traj.points.push_back(arm_point);
        
        // Publish both trajectories with minimal delay
        arm_pub_->publish(arm_traj);
        base_pub_->publish(base_traj);
    }

    void update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Initialize with zeros
        for (unsigned int i = 0; i < full_joint_positions_.rows(); ++i) {
            full_joint_positions_(i) = 0.0;
        }
        for (unsigned int i = 0; i < arm_joint_positions_.rows(); ++i) {
            arm_joint_positions_(i) = 0.0;
        }

        // Map received joint states
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint_name = msg->name[i];
            double position = msg->position[i];

            // Base joints
            if (joint_name == "world_to_base_translate_x") {
                full_joint_positions_(0) = position;
            } else if (joint_name == "world_to_base_translate_y") {
                full_joint_positions_(1) = position;
            } else if (joint_name == "world_to_base_rotate_z") {
                full_joint_positions_(2) = position;
            }
            // Arm joints
            else if (joint_name == "shoulder_pan_joint") {
                full_joint_positions_(3) = position;
                arm_joint_positions_(0) = position;
            } else if (joint_name == "shoulder_lift_joint") {
                full_joint_positions_(4) = position;
                arm_joint_positions_(1) = position;
            } else if (joint_name == "elbow_joint") {
                full_joint_positions_(5) = position;
                arm_joint_positions_(2) = position;
            } else if (joint_name == "wrist_1_joint") {
                full_joint_positions_(6) = position;
                arm_joint_positions_(3) = position;
            } else if (joint_name == "wrist_2_joint") {
                full_joint_positions_(7) = position;
                arm_joint_positions_(4) = position;
            } else if (joint_name == "wrist_3_joint") {
                full_joint_positions_(8) = position;
                arm_joint_positions_(5) = position;
            }
        }

        if (!joint_states_ready_) {
            joint_states_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "Joint states received. Starting initial arm movement.");
            generate_and_publish_arm_trajectory();
        }
    }

    VelocityProfile calculate_velocity_profile(double distance, double desired_velocity, double acceleration) {
        VelocityProfile profile;
        
        double t_to_max_vel = desired_velocity / acceleration;
        double accel_distance = 0.5 * acceleration * t_to_max_vel * t_to_max_vel;
        double total_accel_decel_distance = 2.0 * accel_distance;
        
        if (total_accel_decel_distance >= distance) {
            profile.t_accel = std::sqrt(distance / acceleration);
            profile.t_const = 0.0;
            profile.t_decel = profile.t_accel;
            profile.max_velocity = acceleration * profile.t_accel;
        } else {
            profile.t_accel = t_to_max_vel;
            profile.t_decel = t_to_max_vel;
            profile.max_velocity = desired_velocity;
            
            double const_vel_distance = distance - total_accel_decel_distance;
            profile.t_const = const_vel_distance / desired_velocity;
        }
        
        profile.total_time = profile.t_accel + profile.t_const + profile.t_decel;
        return profile;
    }

    double get_position_at_time(double t, double distance, const VelocityProfile& profile) {
        if (t <= 0.0) return 0.0;
        if (t >= profile.total_time) return distance;
        
        double acceleration = profile.max_velocity / profile.t_accel;
        
        if (t <= profile.t_accel) {
            return 0.5 * acceleration * t * t;
        } else if (t <= profile.t_accel + profile.t_const) {
            double accel_distance = 0.5 * acceleration * profile.t_accel * profile.t_accel;
            double const_time = t - profile.t_accel;
            return accel_distance + profile.max_velocity * const_time;
        } else {
            double accel_distance = 0.5 * acceleration * profile.t_accel * profile.t_accel;
            double const_distance = profile.max_velocity * profile.t_const;
            double decel_time = t - profile.t_accel - profile.t_const;
            double decel_distance = profile.max_velocity * decel_time - 0.5 * acceleration * decel_time * decel_time;
            return accel_distance + const_distance + decel_distance;
        }
    }

    trajectory_msgs::msg::JointTrajectory generate_linear_cartesian_trajectory(
        const KDL::Frame& start,
        const KDL::Frame& end,
        double desired_velocity,
        double acceleration
    ) {
        double distance = (end.p - start.p).Norm();
        VelocityProfile profile = calculate_velocity_profile(distance, desired_velocity, acceleration);
        
        double dt = 0.02;
        int num_steps = static_cast<int>(std::ceil(profile.total_time / dt));
        
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };

        KDL::JntArray q_init = arm_joint_positions_;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Initial motion: Start=(%.3f,%.3f,%.3f), End=(%.3f,%.3f,%.3f), distance=%.3fm, time=%.2fs",
                   start.p.x(), start.p.y(), start.p.z(),
                   end.p.x(), end.p.y(), end.p.z(), 
                   distance, profile.total_time);

        for (int i = 0; i <= num_steps; ++i) {
            double t = i * dt;
            if (t > profile.total_time) t = profile.total_time;
            
            double s = get_position_at_time(t, distance, profile);
            double alpha = (distance > 0.0) ? s / distance : 0.0;
            
            KDL::Vector interp_pos = start.p * (1.0 - alpha) + end.p * alpha;
            KDL::Frame interp_pose(KDL::Rotation::Identity(), interp_pos);

            KDL::JntArray q_out(arm_chain_.getNrOfJoints());
            int result = arm_ik_solver_->CartToJnt(q_init, interp_pose, q_out);

            if (result >= 0) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions.resize(q_out.rows());
                for (unsigned j = 0; j < q_out.rows(); ++j)
                    point.positions[j] = q_out(j);

                point.time_from_start = rclcpp::Duration::from_seconds(t);
                traj.points.push_back(point);
                q_init = q_out;
                
                if (i == num_steps) {
                    final_arm_config_ = q_out;
                }
            }
        }

        return traj;
    }

    void generate_and_publish_arm_trajectory() {
        if (!joint_states_ready_) {
            RCLCPP_WARN(this->get_logger(), "Joint states not received yet. Skipping trajectory generation.");
            return;
        }

        // Get current TCP position in base frame
        KDL::Frame current_pose;
        int fk_result = arm_fk_solver_->JntToCart(arm_joint_positions_, current_pose);
        if (fk_result < 0) {
            RCLCPP_ERROR(this->get_logger(), "Arm forward kinematics failed.");
            return;
        }

        // Get current base position to compute world position
        KDL::Frame base_pose;
        base_pose.p = KDL::Vector(full_joint_positions_(0), full_joint_positions_(1), 0.0);
        base_pose.M = KDL::Rotation::RotZ(full_joint_positions_(2));
        
        // Current TCP in world frame
        KDL::Frame current_world_pose = base_pose * current_pose;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Current TCP world position: (%.3f, %.3f, %.3f), Base at: (%.3f, %.3f)",
                   current_world_pose.p.x(), current_world_pose.p.y(), current_world_pose.p.z(),
                   base_pose.p.x(), base_pose.p.y());

        // Target is in base frame for initial movement
        KDL::Frame start = current_pose;
        KDL::Frame end(KDL::Rotation::Identity(), params_.target_position);
        
        // Store target in world coordinates (transform from base to world)
        target_world_position_ = base_pose * end.p;
        
        target_position_set_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Target TCP world position: (%.3f, %.3f, %.3f)",
                   target_world_position_.x(), target_world_position_.y(), target_world_position_.z());

        double desired_velocity = params_.arm_velocity;
        double acceleration = params_.arm_acceleration;

        auto traj = generate_linear_cartesian_trajectory(start, end, desired_velocity, acceleration);
        
        if (!traj.points.empty()) {
            auto last_point = traj.points.back();
            current_trajectory_duration_ = last_point.time_from_start.sec + 
                                         last_point.time_from_start.nanosec * 1e-9;
            trajectory_start_time_ = this->get_clock()->now();
            trajectory_in_progress_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Published initial arm trajectory, duration: %.2fs", current_trajectory_duration_);
        }
        
        arm_pub_->publish(traj);
    }

    void start_coordinated_movement() {
        if (!target_position_set_ || base_movement_in_progress_) {
            return;
        }

        base_start_x_ = full_joint_positions_(0);
        base_target_x_ = base_start_x_ + (base_forward_ ? params_.base_distance : -params_.base_distance);
        base_movement_start_time_ = this->get_clock()->now();
        base_movement_in_progress_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Starting coordinated movement: base %.3f -> %.3f", 
                   base_start_x_, base_target_x_);
        
        base_forward_ = !base_forward_;
    }

    void check_trajectory_status() {
        if (trajectory_in_progress_ && !base_movement_in_progress_) {
            auto current_time = this->get_clock()->now();
            double elapsed_time = (current_time - trajectory_start_time_).seconds();
            
            if (elapsed_time >= current_trajectory_duration_) {
                RCLCPP_INFO(this->get_logger(), "Initial arm trajectory completed. Starting coordinated movement.");
                trajectory_in_progress_ = false;
                
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                start_coordinated_movement();
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingGenerator>());
    rclcpp::shutdown();
    return 0;
}