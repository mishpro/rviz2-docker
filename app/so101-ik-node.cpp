#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <iostream>
#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

class SO101IKNode : public rclcpp::Node {
public:
    SO101IKNode(const std::string &urdf_path)
        : Node("so101_ik_node")
    {
        // Загрузка модели
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        ee_id_ = model_.getFrameId("gripper_frame_link");
        q_current_ = pinocchio::neutral(model_);
        q_target_ = q_current_;
        q_start_interp_ = q_current_;

        for (int i = 1; i < (int)model_.njoints; ++i) {
            if (model_.nqs[i] > 0)
                joint_names_.push_back(model_.names[i]);
        }

        // Параметры
        declare_parameter("publish_rate", 50.0);
        declare_parameter("interp_steps", 100);
        declare_parameter("ik_max_iter", 100);
        declare_parameter("ik_eps", 1e-4);
        declare_parameter("ik_dt", 1e-1);
        declare_parameter("ik_damp", 1e-6);

        // Publishers
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);

        // Subscriber
        target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_pose", 10,
            std::bind(&SO101IKNode::targetCallback, this, std::placeholders::_1));

        // Таймер публикации joint states
        double rate = get_parameter("publish_rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / rate)),
            std::bind(&SO101IKNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "SO-101 IK node ready. nq=%d, nv=%d",
                    model_.nq, model_.nv);
        RCLCPP_INFO(get_logger(), "Send target via: ros2 topic pub --once /target_pose "
                "geometry_msgs/msg/PointStamped \"{header: {frame_id: base_link}, "
                    "point: {x: 0.2, y: 0.0, z: 0.15}}\"");
    }

private:
    void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        Eigen::Vector3d target(msg->point.x, msg->point.y, msg->point.z);
        RCLCPP_INFO(get_logger(), "New target: [%.3f, %.3f, %.3f]",
                    target.x(), target.y(), target.z());

        // Решаем IK от текущей позы
        Eigen::VectorXd q = q_current_;
        int max_iter = get_parameter("ik_max_iter").as_int();
        double eps = get_parameter("ik_eps").as_double();
        double dt = get_parameter("ik_dt").as_double();
        double damp = get_parameter("ik_damp").as_double();

        for (int i = 0; i < max_iter; ++i) {
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacement(model_, data_, ee_id_);

            Eigen::Vector3d err = target - data_.oMf[ee_id_].translation();
            if (err.norm() < eps) {
                RCLCPP_INFO(get_logger(), "IK converged at iter %d, error=%.6f",
                            i, err.norm());
                break;
            }

            Eigen::MatrixXd J(6, model_.nv);
            J.setZero();
            pinocchio::computeJointJacobians(model_, data_, q);
            pinocchio::getFrameJacobian(model_, data_, ee_id_,
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);

            Eigen::MatrixXd J_pos = J.topRows(3);
            Eigen::MatrixXd JJt = J_pos * J_pos.transpose();
            JJt.diagonal().array() += damp;
            Eigen::VectorXd v = J_pos.transpose() * JJt.ldlt().solve(err);
            q = pinocchio::integrate(model_, q, dt * v);
            q = q.cwiseMax(model_.lowerPositionLimit)
                .cwiseMin(model_.upperPositionLimit);
        }

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, ee_id_);
        RCLCPP_INFO(get_logger(), "Final EE pos: [%.3f, %.3f, %.3f]",
                    data_.oMf[ee_id_].translation().x(),
                    data_.oMf[ee_id_].translation().y(),
                    data_.oMf[ee_id_].translation().z());

        q_target_ = q;
        interp_step_ = 0;
        int n = get_parameter("interp_steps").as_int();
        interp_total_ = (n > 0) ? n : 1;
        q_start_interp_ = q_current_;

        publishTargetMarker(target);
    }

    void timerCallback() {
        // Интерполяция к целевой позе
        if (interp_step_ < interp_total_) {
            double alpha = (double)interp_step_ / interp_total_;
            // Сглаживание (cosine interpolation)
            double smooth = 0.5 * (1.0 - std::cos(M_PI * alpha));
            q_current_ = q_start_interp_ + smooth * (q_target_ - q_start_interp_);
            q_current_ = q_current_.cwiseMax(model_.lowerPositionLimit)
                .cwiseMin(model_.upperPositionLimit);
            interp_step_++;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = joint_names_;
        msg.position.resize(q_current_.size());
        for (int i = 0; i < q_current_.size(); ++i)
            msg.position[i] = q_current_[i];
        joint_pub_->publish(msg);
    }

    void publishTargetMarker(const Eigen::Vector3d &pos) {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now();
        m.header.frame_id = "base_link";
        m.ns = "ik_target";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = pos.x();
        m.pose.position.y = pos.y();
        m.pose.position.z = pos.z();
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.03;
        m.color.r = 1.0;
        m.color.g = 0.2;
        m.color.b = 0.2;
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration(0, 0);
        marker_pub_->publish(m);
    }

    // Pinocchio
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_id_;
    std::vector<std::string> joint_names_;
    Eigen::VectorXd q_current_;
    Eigen::VectorXd q_target_;
    Eigen::VectorXd q_start_interp_;
    int interp_step_ = 0;
    int interp_total_ = 1;

    // ROS 2
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // URDF путь: аргумент или параметр
    std::string urdf_path;
    if (argc >= 2) {
        urdf_path = argv[1];
    } else {
        // Пытаемся получить из ROS-параметра
        rclcpp::Node tmp("tmp");
        tmp.declare_parameter("urdf_path", "");
        urdf_path = tmp.get_parameter("urdf_path").as_string();
    }

    if (urdf_path.empty()) {
        std::cerr << "Usage: " << argv[0] << " <path/to/so101.urdf>" << std::endl;
        std::cerr << "  or pass -p urdf_path:=<path> as ROS param" << std::endl;
        return 1;
    }

    rclcpp::spin(std::make_shared<SO101IKNode>(urdf_path));
    rclcpp::shutdown();
    return 0;
}
