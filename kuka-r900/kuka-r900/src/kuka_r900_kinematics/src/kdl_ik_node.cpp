#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <mutex>

class KDLInverseKinematicsNode : public rclcpp::Node {
public:
    KDLInverseKinematicsNode()
        : Node("kdl_ik_node") {

        std::string urdf_string;
        this->declare_parameter("robot_description", "");
        this->get_parameter("robot_description", urdf_string);

        urdf::Model urdf_model;
        if (!urdf_model.initString(urdf_string)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF string");
            return;
        }

        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
            return;
        }

        std::string base_link = "base_link";
        std::string tip_link = "tool0";

        if (!tree.getChain(base_link, tip_link, chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from %s to %s", base_link.c_str(), tip_link.c_str());
            return;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
        q_current_ = KDL::JntArray(chain_.getNrOfJoints());

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10,
            std::bind(&KDLInverseKinematicsNode::pose_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&KDLInverseKinematicsNode::joint_states_callback, this, std::placeholders::_1));

        setting_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/ik_settings", 10,
            std::bind(&KDLInverseKinematicsNode::ik_settings_callback, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/ik_control_mode", 10,
            std::bind(&KDLInverseKinematicsNode::ik_mode_callback, this, std::placeholders::_1));

        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_trajectory_controller/joint_trajectory", 10);

        ik_status_pub_ = this->create_publisher<std_msgs::msg::String>("/ik_status", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&KDLInverseKinematicsNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "KDL IK node started.");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        last_target_ = *msg;
        has_target_ = true;
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(q_mutex_);
        if (msg->position.size() >= chain_.getNrOfJoints()) {
            for (size_t i = 0; i < chain_.getNrOfJoints(); ++i) {
                q_current_(i) = msg->position[i];
            }
        }
    }

    void ik_settings_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string data = msg->data;
        size_t delim = data.find(',');
        if (delim != std::string::npos) {
            elbow_config_ = data.substr(0, delim);
            wrist_config_ = data.substr(delim + 1);
            RCLCPP_INFO(this->get_logger(), "IK config updated: elbow=%s, wrist=%s", elbow_config_.c_str(), wrist_config_.c_str());
        }
    }

    void ik_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        ik_enabled_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "IK mode set to: %s", ik_enabled_ ? "ENABLED" : "DISABLED");
    }

    void timer_callback() {
        if (!ik_enabled_ || !has_target_) return;

        geometry_msgs::msg::PoseStamped target_copy;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_copy = last_target_;
        }

        KDL::Frame target;
        target.p = KDL::Vector(target_copy.pose.position.x, target_copy.pose.position.y, target_copy.pose.position.z);
        target.M = KDL::Rotation::Quaternion(
            target_copy.pose.orientation.x,
            target_copy.pose.orientation.y,
            target_copy.pose.orientation.z,
            target_copy.pose.orientation.w);

        KDL::JntArray q_init(chain_.getNrOfJoints());
        {
            std::lock_guard<std::mutex> lock(q_mutex_);
            q_init = q_current_;
        }

        if (elbow_config_ == "up") q_init(1) = -1.0;
        else if (elbow_config_ == "down") q_init(1) = 1.0;

        if (wrist_config_ == "flipped") q_init(4) = 1.5;
        else if (wrist_config_ == "normal") q_init(4) = 0.0;

        KDL::JntArray q_out(chain_.getNrOfJoints());
        std_msgs::msg::String status_msg;

        if (ik_solver_->CartToJnt(q_init, target, q_out) >= 0) {
            trajectory_msgs::msg::JointTrajectory traj;
            traj.header.stamp = this->now();
            traj.joint_names = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            for (unsigned int i = 0; i < q_out.rows(); ++i) {
                point.positions.push_back(q_out(i));
            }

            point.velocities.resize(chain_.getNrOfJoints(), 0.0);
            point.time_from_start = rclcpp::Duration::from_seconds(1.0);

            traj.points.push_back(point);
            traj_pub_->publish(traj);

            status_msg.data = "ok";
        } else {
            RCLCPP_WARN(this->get_logger(), "IK failed for given target pose.");
            status_msg.data = "fail";
        }

        ik_status_pub_->publish(status_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr setting_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ik_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::Chain chain_;
    KDL::JntArray q_current_;
    std::mutex q_mutex_;

    geometry_msgs::msg::PoseStamped last_target_;
    std::mutex target_mutex_;
    bool has_target_ = false;
    bool ik_enabled_ = true;

    std::string elbow_config_ = "up";
    std::string wrist_config_ = "normal";
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KDLInverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
