#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>

class ForwardKinematicsNode : public rclcpp::Node {
public:
    ForwardKinematicsNode() : Node("kdl_fk_node") {
        this->declare_parameter("robot_description", "");
        std::string urdf_string;
        this->get_parameter("robot_description", urdf_string);

        urdf::Model urdf_model;
        if (!urdf_model.initString(urdf_string)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
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
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain");
            return;
        }

        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ForwardKinematicsNode::joint_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Forward Kinematics Node started.");
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < chain_.getNrOfJoints()) return;

        KDL::JntArray q(chain_.getNrOfJoints());
        for (size_t i = 0; i < chain_.getNrOfJoints(); ++i) {
            q(i) = msg->position[i];
        }

        KDL::Frame result;
        if (fk_solver_->JntToCart(q, result) >= 0) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "base_link";

            pose.pose.position.x = result.p.x();
            pose.pose.position.y = result.p.y();
            pose.pose.position.z = result.p.z();

            double x, y, z, w;
            result.M.GetQuaternion(x, y, z, w);
            pose.pose.orientation.x = x;
            pose.pose.orientation.y = y;
            pose.pose.orientation.z = z;
            pose.pose.orientation.w = w;

            pose_pub_->publish(pose);
        }
    }

    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
