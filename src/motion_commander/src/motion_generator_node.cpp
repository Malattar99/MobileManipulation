#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <cmath>

class MotionGenerator : public rclcpp::Node {
public:
    MotionGenerator() : Node("motion_generator_node"), 
                       trajectory_in_progress_(false),
                       joint_states_ready_(false) {
        
        // Declare parameters with default values
        this->declare_parameter("pose1.x", 0.7);
        this->declare_parameter("pose1.y", 0.0);
        this->declare_parameter("pose1.z", 0.4);
        this->declare_parameter("pose1.roll", 0.0);
        this->declare_parameter("pose1.pitch", 0.0);
        this->declare_parameter("pose1.yaw", 0.0);
        
        this->declare_parameter("pose2.x", 0.7);
        this->declare_parameter("pose2.y", 0.5);
        this->declare_parameter("pose2.z", 0.4);
        this->declare_parameter("pose2.roll", 0.0);
        this->declare_parameter("pose2.pitch", 0.0);
        this->declare_parameter("pose2.yaw", 0.0);
        
        this->declare_parameter("linear_velocity", 0.5);
        this->declare_parameter("linear_acceleration", 0.2);
        
        // Option to continuously loop between poses
        this->declare_parameter("continuous_motion", true);
        
        // Publisher for arm trajectory
        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/motion_library/arm_trajectory", 10);

        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                update_joint_states(msg);
            });

        // Service to trigger motion execution
        motion_service_ = this->create_service<std_srvs::srv::Trigger>(
            "execute_linear_motion",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                if (trajectory_in_progress_) {
                    response->success = false;
                    response->message = "Trajectory already in progress";
                } else if (!joint_states_ready_) {
                    response->success = false;
                    response->message = "Joint states not ready";
                } else {
                    execute_single_motion();
                    response->success = true;
                    response->message = "Linear motion started";
                }
            });

        // Initialize KDL
        this->declare_parameter<std::string>("robot_description", "");
        std::string robot_description;
        this->get_parameter("robot_description", robot_description);

        if (!kdl_parser::treeFromString(robot_description, tree_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree.");
            return;
        }

        std::string base_link = "base_link";
        std::string tip_link = "tool0";
        if (!tree_.getChain(base_link, tip_link, chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from %s to %s", 
                        base_link.c_str(), tip_link.c_str());
            return;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

        joint_positions_ = KDL::JntArray(chain_.getNrOfJoints());

        RCLCPP_INFO(this->get_logger(), "Motion Generator initialized with %d joints", 
                   chain_.getNrOfJoints());

        // Timer for continuous motion if enabled
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotionGenerator::timer_callback, this)
        );
        
        // Load and display current parameters
        load_parameters();
    }

private:
    struct VelocityProfile {
        double t_accel;
        double t_const;
        double t_decel;
        double total_time;
        double max_velocity;
    };

    struct MotionParameters {
        KDL::Frame pose1;
        KDL::Frame pose2;
        double linear_velocity;
        double linear_acceleration;
        bool continuous_motion;
    };

    void load_parameters() {
        // Load pose1
        double x1 = this->get_parameter("pose1.x").as_double();
        double y1 = this->get_parameter("pose1.y").as_double();
        double z1 = this->get_parameter("pose1.z").as_double();
        double roll1 = this->get_parameter("pose1.roll").as_double();
        double pitch1 = this->get_parameter("pose1.pitch").as_double();
        double yaw1 = this->get_parameter("pose1.yaw").as_double();
        
        params_.pose1 = KDL::Frame(
            KDL::Rotation::RPY(roll1, pitch1, yaw1),
            KDL::Vector(x1, y1, z1)
        );
        
        // Load pose2
        double x2 = this->get_parameter("pose2.x").as_double();
        double y2 = this->get_parameter("pose2.y").as_double();
        double z2 = this->get_parameter("pose2.z").as_double();
        double roll2 = this->get_parameter("pose2.roll").as_double();
        double pitch2 = this->get_parameter("pose2.pitch").as_double();
        double yaw2 = this->get_parameter("pose2.yaw").as_double();
        
        params_.pose2 = KDL::Frame(
            KDL::Rotation::RPY(roll2, pitch2, yaw2),
            KDL::Vector(x2, y2, z2)
        );
        
        // Load velocity and acceleration
        params_.linear_velocity = this->get_parameter("linear_velocity").as_double();
        params_.linear_acceleration = this->get_parameter("linear_acceleration").as_double();
        params_.continuous_motion = this->get_parameter("continuous_motion").as_bool();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Loaded parameters:\n"
                   "  Pose1: (%.3f, %.3f, %.3f) RPY(%.3f, %.3f, %.3f)\n"
                   "  Pose2: (%.3f, %.3f, %.3f) RPY(%.3f, %.3f, %.3f)\n"
                   "  Velocity: %.3f m/s, Acceleration: %.3f m/s²\n"
                   "  Continuous motion: %s",
                   x1, y1, z1, roll1, pitch1, yaw1,
                   x2, y2, z2, roll2, pitch2, yaw2,
                   params_.linear_velocity, params_.linear_acceleration,
                   params_.continuous_motion ? "enabled" : "disabled");
    }

    void update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint_name = msg->name[i];
            double position = msg->position[i];

            if (joint_name == "shoulder_pan_joint") {
                joint_positions_(0) = position;
            } else if (joint_name == "shoulder_lift_joint") {
                joint_positions_(1) = position;
            } else if (joint_name == "elbow_joint") {
                joint_positions_(2) = position;
            } else if (joint_name == "wrist_1_joint") {
                joint_positions_(3) = position;
            } else if (joint_name == "wrist_2_joint") {
                joint_positions_(4) = position;
            } else if (joint_name == "wrist_3_joint") {
                joint_positions_(5) = position;
            }
        }
        
        if (!joint_states_ready_) {
            joint_states_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "Joint states ready");
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

        KDL::JntArray q_init = joint_positions_;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Generating trajectory: distance=%.3fm, max_vel=%.3fm/s, total_time=%.2fs", 
                   distance, profile.max_velocity, profile.total_time);

        for (int i = 0; i <= num_steps; ++i) {
            double t = i * dt;
            if (t > profile.total_time) t = profile.total_time;
            
            double s = get_position_at_time(t, distance, profile);
            double alpha = (distance > 0.0) ? s / distance : 0.0;
            
            // Interpolate position and orientation
            KDL::Vector interp_pos = start.p * (1.0 - alpha) + end.p * alpha;
            
            // For orientation, we could use quaternion SLERP, but for now keeping it simple
            // with just the end orientation
            KDL::Frame interp_pose(end.M, interp_pos);

            KDL::JntArray q_out(chain_.getNrOfJoints());
            int result = ik_solver_->CartToJnt(q_init, interp_pose, q_out);

            if (result >= 0) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions.resize(q_out.rows());
                for (unsigned j = 0; j < q_out.rows(); ++j)
                    point.positions[j] = q_out(j);

                point.time_from_start = rclcpp::Duration::from_seconds(t);
                traj.points.push_back(point);

                q_init = q_out;
            }
        }

        return traj;
    }

    void execute_single_motion() {
        // Reload parameters in case they changed
        load_parameters();
        
        KDL::Frame start = at_pose1_ ? params_.pose1 : params_.pose2;
        KDL::Frame end = at_pose1_ ? params_.pose2 : params_.pose1;
        at_pose1_ = !at_pose1_;

        auto traj = generate_linear_cartesian_trajectory(
            start, end, 
            params_.linear_velocity, 
            params_.linear_acceleration
        );
        
        if (!traj.points.empty()) {
            auto last_point = traj.points.back();
            current_trajectory_duration_ = last_point.time_from_start.sec + 
                                         last_point.time_from_start.nanosec * 1e-9;
            trajectory_start_time_ = this->get_clock()->now();
            trajectory_in_progress_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                       "Executing linear motion from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                       start.p.x(), start.p.y(), start.p.z(), 
                       end.p.x(), end.p.y(), end.p.z());
            
            pub_->publish(traj);
        }
    }

    void timer_callback() {
        if (!joint_states_ready_ || !params_.continuous_motion) {
            return;
        }

        if (!trajectory_in_progress_) {
            execute_single_motion();
        } else {
            auto now = this->get_clock()->now();
            auto elapsed = (now - trajectory_start_time_).seconds();
            
            if (elapsed >= current_trajectory_duration_ + 0.5) {
                trajectory_in_progress_ = false;
            }
        }
    }

    // State variables
    bool trajectory_in_progress_;
    bool joint_states_ready_;
    bool at_pose1_ = true;
    double current_trajectory_duration_;
    rclcpp::Time trajectory_start_time_;
    
    // Motion parameters
    MotionParameters params_;
    
    // KDL components
    KDL::Tree tree_;
    KDL::Chain chain_;
    KDL::JntArray joint_positions_;
    
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    
    // ROS interfaces
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motion_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionGenerator>());
    rclcpp::shutdown();
    return 0;
}