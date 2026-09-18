#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

enum class IKStatus { Converged, Approximate, Projected, Failed };

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
        declare_parameter("ik_max_iter", 500);
        declare_parameter("perturb_scales", std::vector<double>{0.05, 0.15, 0.30, 0.50, 0.80});
        declare_parameter("auto_flip_restart", true);
        declare_parameter("ik_eps", 1e-4);
        declare_parameter("ik_eps_visual", 0.01);
        declare_parameter("ik_dt", 1e-1);
        declare_parameter("ik_dt_min", 5e-2);
        declare_parameter("ik_dt_max", 5e-1);
        declare_parameter("ik_damp", 1e-6);
        declare_parameter("ik_man_k", 1e-4);
        declare_parameter("ik_man_thresh", 1e-4);
        declare_parameter("weight_jc", 0.5);
        declare_parameter("weight_man", 0.2);
        declare_parameter("weight_prev", 0.3);
        declare_parameter("weight_pref", 0.5);
        declare_parameter("weight_drift", 0.1);
        declare_parameter("preferred_q", std::vector<double>{0.0, -0.3, 1.0, -0.7, 0.0, 0.0});
        declare_parameter("ik_stuck_patience", 40);
        declare_parameter("reach_padding", 1e-2);
        declare_parameter("init_target_x", 0.20);
        declare_parameter("init_target_y", 0.0);
        declare_parameter("init_target_z", 0.15);
        declare_parameter("vel_max", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        declare_parameter("mu_boundary", 5.0);
        declare_parameter("weight_collision", 0.5);
        declare_parameter("collision_d_min", 0.005);
        declare_parameter("collision_margin", 0.010);

        // Рабочая область: r_max через FK в нескольких позах
        computeWorkspaceRadius();

        // Улучшенная начальная поза: IK к (init_target_x, init_target_y, init_target_z)
        {
            Eigen::Vector3d init_target(
                get_parameter("init_target_x").as_double(),
                get_parameter("init_target_y").as_double(),
                get_parameter("init_target_z").as_double());
            q_current_ = solveIK(init_target, q_current_, /*track_status=*/nullptr);
            q_target_ = q_current_;
            q_start_interp_ = q_current_;
        }

        // Publishers
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);
        ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ee_pose", 10);
        collision_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/collision_marker", 10);

        // Subscriber
        target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_pose", 10,
            std::bind(&SO101IKNode::targetCallback, this, std::placeholders::_1));

        // Таймер публикации joint states
        double rate = get_parameter("publish_rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / rate)),
            std::bind(&SO101IKNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "SO-101 IK node ready. nq=%d, nv=%d, r_max=%.3f m",
                    model_.nq, model_.nv, r_max_);
        RCLCPP_INFO(get_logger(), "Send target via: ros2 topic pub --once /target_pose "
                "geometry_msgs/msg/PointStamped \"{header: {frame_id: base_link}, "
                    "point: {x: 0.2, y: 0.0, z: 0.15}}\"");
    }

private:
    void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        Eigen::Vector3d target(msg->point.x, msg->point.y, msg->point.z);
        RCLCPP_INFO(get_logger(), "New target: [%.3f, %.3f, %.3f]",
                    target.x(), target.y(), target.z());

        // Проекция на рабочую область
        IKStatus status = IKStatus::Converged;
        target = projectToWorkspace(target, status);

        // Решаем IK от текущей позы
        Eigen::VectorXd q = solveIK(target, q_current_, &status);

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, ee_id_);
        Eigen::Vector3d ee_pos = data_.oMf[ee_id_].translation();
        const char* st = (status == IKStatus::Converged)   ? "converged"
                       : (status == IKStatus::Approximate) ? "approximate"
                       : (status == IKStatus::Projected)   ? "projected" : "failed";
        RCLCPP_INFO(get_logger(), "IK %s, final EE=[%.3f, %.3f, %.3f] err=%.4f",
                    st, ee_pos.x(), ee_pos.y(), ee_pos.z(),
                    (target - ee_pos).norm());

        q_target_ = q;
        interp_step_ = 0;
        int n = get_parameter("interp_steps").as_int();
        interp_total_ = (n > 0) ? n : 1;
        q_start_interp_ = q_current_;

        publishTargetMarker(target, status);
        publishEEPose(ee_pos, status);
        publishCollisionMarker();
    }

    Eigen::Vector3d projectToWorkspace(Eigen::Vector3d target, IKStatus& status) {
        double pad = get_parameter("reach_padding").as_double();
        Eigen::Vector3d d = target - base_pos_;
        double r = d.norm();
        if (r > r_max_ - pad) {
            target = base_pos_ + d * (r_max_ - pad) / r;
            status = IKStatus::Projected;
            RCLCPP_WARN(get_logger(),
                "Target outside workspace (r=%.3f > r_max=%.3f), projected to [%.3f, %.3f, %.3f]",
                r, r_max_, target.x(), target.y(), target.z());
        }
        return target;
    }

    Eigen::VectorXd solveIK(const Eigen::Vector3d& target,
                            const Eigen::VectorXd& q_start,
                            IKStatus* status)
    {
        int max_iter = get_parameter("ik_max_iter").as_int();
        auto perturb_scales = get_parameter("perturb_scales").as_double_array();
        double eps    = get_parameter("ik_eps").as_double();
        double eps_v  = get_parameter("ik_eps_visual").as_double();
        double dt0    = get_parameter("ik_dt").as_double();
        double dt_min = get_parameter("ik_dt_min").as_double();
        double dt_max = get_parameter("ik_dt_max").as_double();
        double damp_b = get_parameter("ik_damp").as_double();
        double man_k  = get_parameter("ik_man_k").as_double();
        double man_th = get_parameter("ik_man_thresh").as_double();
        double w_jc   = get_parameter("weight_jc").as_double();
        double w_man  = get_parameter("weight_man").as_double();
        double w_prev = get_parameter("weight_prev").as_double();
        double w_pref = get_parameter("weight_pref").as_double();
        double w_drift = get_parameter("weight_drift").as_double();
        Eigen::VectorXd q_pref = Eigen::VectorXd::Map(
            get_parameter("preferred_q").as_double_array().data(),
            model_.nv);
        int    stuck_patience = get_parameter("ik_stuck_patience").as_int();
        Eigen::VectorXd v_max = Eigen::VectorXd::Map(
            get_parameter("vel_max").as_double_array().data(),
            model_.nv);
        double mu_boundary = get_parameter("mu_boundary").as_double();
        double w_collision     = get_parameter("weight_collision").as_double();
        double collision_d_min = get_parameter("collision_d_min").as_double();
        double collision_margin = get_parameter("collision_margin").as_double();

        Eigen::VectorXd q_mid = 0.5 * (model_.lowerPositionLimit.array() +
                                       model_.upperPositionLimit.array());
        std::mt19937 rng(123);
        bool auto_flip = get_parameter("auto_flip_restart").as_bool();

        Eigen::VectorXd best_q = q_start;
        double best_err = std::numeric_limits<double>::infinity();
        bool converged = false;

        auto runIKLoop = [&](Eigen::VectorXd q) {
            double dt = dt0;
            double prev_err = std::numeric_limits<double>::infinity();
            int stuck_count = 0;

            for (int i = 0; i < max_iter; ++i) {
                pinocchio::forwardKinematics(model_, data_, q);
                pinocchio::updateFramePlacement(model_, data_, ee_id_);
                Eigen::Vector3d ee_pos = data_.oMf[ee_id_].translation();
                Eigen::Vector3d err = target - ee_pos;
                double err_norm = err.norm();
                if (err_norm < eps) { converged = true; best_q = q; best_err = err_norm; break; }
                if (err_norm < best_err) { best_q = q; best_err = err_norm; }

                Eigen::MatrixXd J(6, model_.nv);
                J.setZero();
                pinocchio::computeJointJacobians(model_, data_, q);
                pinocchio::getFrameJacobian(model_, data_, ee_id_,
                                            pinocchio::LOCAL_WORLD_ALIGNED, J);
                Eigen::MatrixXd Jp = J.topRows(3);

                double mu = std::sqrt(std::max(0.0,
                    (Jp * Jp.transpose()).determinant()));
                double lambda = damp_b + man_k * std::max(0.0, 1.0 - mu / (man_th + 1e-12));
                lambda = std::min(lambda, 0.5);

                Eigen::Matrix3d JJt = Jp * Jp.transpose();
                JJt.diagonal().array() += lambda * lambda;
                Eigen::Vector3d x = JJt.ldlt().solve(err);
                Eigen::VectorXd v_task = Jp.transpose() * x;

                Eigen::MatrixXd Jplus = Jp.transpose() * JJt.ldlt().solve(Eigen::Matrix3d::Identity());
                Eigen::MatrixXd N = Eigen::MatrixXd::Identity(model_.nv, model_.nv) - Jplus * Jp;

                Eigen::VectorXd z = Eigen::VectorXd::Zero(model_.nv);
                z += w_jc   * (q_mid - q);
                z += w_prev * (q_start - q);
                if (w_drift > 0.0)
                    z += w_drift * (q_start - q);
                if (w_pref > 0.0)
                    z += w_pref * (q_pref - q);
                if (w_man > 0.0)
                    z += w_man * numericalManipGradient(Jp, q, man_th);

                // Self-collision CBF barrier — only when distance < d_min + margin
                if (w_collision > 0.0) {
                    worst_pair_idx_ = -1;
                    worst_d_ = std::numeric_limits<double>::infinity();
                    for (size_t pi = 0; pi < collision_pairs_.size(); ++pi) {
                        double d = computePairDistance(collision_pairs_[pi].first,
                                                       collision_pairs_[pi].second);
                        if (d < collision_d_min && d < worst_d_) {
                            worst_d_ = d;
                            worst_pair_idx_ = (int)pi;
                        }
                        if (d < collision_d_min + collision_margin) {
                            // Repulsive: d/dq pulled away when violated
                            Eigen::VectorXd grad_d = numericalDistanceGradient(
                                collision_pairs_[pi], q);
                            z += w_collision * (collision_d_min - d) * grad_d;
                        }
                    }
                }

                Eigen::VectorXd dq = v_task + N * z;

                if (err_norm > prev_err) {
                    dt *= 0.8;
                    ++stuck_count;
                } else {
                    dt = std::min(dt * 1.02, dt_max);
                    stuck_count = 0;
                }
                if (stuck_count > stuck_patience) {
                    RCLCPP_DEBUG(get_logger(),
                        "IK stuck at iter %d (err=%.4f)", i, err_norm);
                    break;
                }
                dt = std::clamp(dt, dt_min, dt_max);
                prev_err = err_norm;

                dq = dq.cwiseMin(v_max.cwiseMin(mu_boundary * (model_.upperPositionLimit - q)))
                       .cwiseMax((-v_max).cwiseMax(mu_boundary * (model_.lowerPositionLimit - q)));
                q = pinocchio::integrate(model_, q, dt * dq);
                q = q.cwiseMax(model_.lowerPositionLimit)
                    .cwiseMin(model_.upperPositionLimit);
            }
        };

        // 0-й запуск: чистый q_start; далее — рестарты с разными масштабами возмущения
        for (size_t restart = 0; restart <= perturb_scales.size() && !converged; ++restart) {
            Eigen::VectorXd q;
            if (restart == 0) q = q_start;
            else              q = perturbConfig(q_start, rng, perturb_scales[restart - 1]);
            runIKLoop(q);
        }

        // Финальная попытка: явный «flip» shoulder_pan для целей за спиной base
        if (auto_flip && !converged) {
            Eigen::VectorXd q = flipShoulderPan(q_start, rng, target);
            runIKLoop(q);
        }

        if (status) {
            if (converged)                       *status = IKStatus::Converged;
            else if (*status == IKStatus::Projected) {} // оставить Projected
            else if (best_err < eps_v)           *status = IKStatus::Converged;
            else if (best_err < 0.05)            *status = IKStatus::Approximate;
            else                                 *status = IKStatus::Failed;
        }
        return best_q;
    }

    Eigen::VectorXd perturbConfig(const Eigen::VectorXd& q0,
                                  std::mt19937& rng, double scale) const
    {
        std::uniform_real_distribution<double> uni(-1.0, 1.0);
        Eigen::VectorXd q = q0;
        for (int i = 0; i < q.size(); ++i) {
            double range = model_.upperPositionLimit[i] - model_.lowerPositionLimit[i];
            q[i] += scale * range * uni(rng);
        }
        q = q.cwiseMax(model_.lowerPositionLimit)
            .cwiseMin(model_.upperPositionLimit);
        return q;
    }

    Eigen::VectorXd flipShoulderPan(const Eigen::VectorXd& q0,
                                    std::mt19937& rng,
                                    const Eigen::Vector3d& target)
    {
        Eigen::VectorXd q = q0;
        if (!model_.existJointName("shoulder_pan")) return q;
        pinocchio::JointIndex sp = model_.getJointId("shoulder_pan");

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, ee_id_);
        Eigen::Vector3d ee_dir = (data_.oMf[ee_id_].translation() - base_pos_).normalized();
        Eigen::Vector3d tgt_dir = (target - base_pos_).normalized();

        double dot = ee_dir.dot(tgt_dir);
        if (dot < 0.0) {
            double limit = model_.upperPositionLimit[sp];
            double cur = q[sp];
            if (std::abs(cur) > 0.1) {
                q[sp] = (cur > 0) ? -limit : limit;
            } else {
                q[sp] = (rng() & 1) ? limit : -limit;
            }
        }
        q = q.cwiseMax(model_.lowerPositionLimit)
            .cwiseMin(model_.upperPositionLimit);
        return q;
    }

    Eigen::VectorXd numericalManipGradient(const Eigen::MatrixXd& Jp,
                                            const Eigen::VectorXd& q,
                                            double man_th)
    {
        // μ = sqrt(det(J·Jᵀ)) для 3D-position
        // ∂μ/∂q_i ≈ (μ(q+h·e_i) − μ(q−h·e_i)) / (2h)
        const double h = 1e-3;
        Eigen::VectorXd grad = Eigen::VectorXd::Zero(model_.nv);
        auto mu_of = [&](const Eigen::VectorXd& qq) -> double {
            Eigen::MatrixXd J(6, model_.nv); J.setZero();
            pinocchio::computeJointJacobians(model_, data_, qq);
            pinocchio::getFrameJacobian(model_, data_, ee_id_,
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);
            Eigen::MatrixXd Jp_ = J.topRows(3);
            return std::sqrt(std::max(0.0, (Jp_ * Jp_.transpose()).determinant()));
        };
        // Если манипулятивность уже выше порога — не тратим итерации
        if (mu_of(q) > man_th) return grad;

        for (int i = 0; i < model_.nv; ++i) {
            Eigen::VectorXd qp = q; qp[i] += h;
            Eigen::VectorXd qm = q; qm[i] -= h;
            qp = qp.cwiseMax(model_.lowerPositionLimit)
                    .cwiseMin(model_.upperPositionLimit);
            qm = qm.cwiseMax(model_.lowerPositionLimit)
                    .cwiseMin(model_.upperPositionLimit);
            grad[i] = (mu_of(qp) - mu_of(qm)) / (2.0 * h);
        }
        return grad;
    }

    Eigen::Vector3d getLinkPosition(const std::string& link_name) {
        // Approximate link world position via parent joint origin (oMi).
        // SO-101 URDF: link name ≠ joint name. Map link → parent joint.
        static const std::map<std::string, std::string> link_to_joint = {
            {"base_link", ""},
            {"shoulder_link", "shoulder_pan"},
            {"upper_arm_link", "shoulder_lift"},
            {"lower_arm_link", "elbow_flex"},
            {"wrist_link", "wrist_flex"},
            {"gripper_link", "wrist_roll"},
        };
        auto it = link_to_joint.find(link_name);
        if (it == link_to_joint.end()) {
            throw std::runtime_error("no joint mapping for link: " + link_name);
        }
        if (it->second.empty()) return Eigen::Vector3d::Zero();  // base_link root
        if (!model_.existJointName(it->second)) {
            throw std::runtime_error("joint not found: " + it->second);
        }
        return data_.oMi[model_.getJointId(it->second)].translation();
    }

    double computePairDistance(const std::string& link_a,
                                const std::string& link_b) {
        Eigen::Vector3d pa = getLinkPosition(link_a);
        Eigen::Vector3d pb = getLinkPosition(link_b);
        double ra = sphere_radii_.at(link_a);
        double rb = sphere_radii_.at(link_b);
        return (pa - pb).norm() - (ra + rb);
    }

    Eigen::VectorXd numericalDistanceGradient(const std::pair<std::string,std::string>& pair,
                                               const Eigen::VectorXd& q) {
        const double h = 1e-3;
        double d0 = computePairDistance(pair.first, pair.second);
        Eigen::VectorXd grad = Eigen::VectorXd::Zero(model_.nv);
        for (int i = 0; i < model_.nv; ++i) {
            Eigen::VectorXd qp = q; qp[i] += h;
            Eigen::VectorXd qm = q; qm[i] -= h;
            qp = qp.cwiseMax(model_.lowerPositionLimit)
                    .cwiseMin(model_.upperPositionLimit);
            qm = qm.cwiseMax(model_.lowerPositionLimit)
                    .cwiseMin(model_.upperPositionLimit);
            pinocchio::forwardKinematics(model_, data_, qp);
            double dp = computePairDistance(pair.first, pair.second);
            pinocchio::forwardKinematics(model_, data_, qm);
            double dm = computePairDistance(pair.first, pair.second);
            grad[i] = (dp - dm) / (2.0 * h);
        }
        // Restore FK to q for caller
        pinocchio::forwardKinematics(model_, data_, q);
        (void)d0;
        return grad;
    }

    void computeWorkspaceRadius() {
        // Базовая точка — позиция EE при q=neutral
        pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
        pinocchio::updateFramePlacement(model_, data_, ee_id_);
        base_pos_ = data_.oMf[ee_id_].translation();

        // Оценка r_max: FK при полностью вытянутом локте
        Eigen::VectorXd q_ext = pinocchio::neutral(model_);
        if (model_.existJointName("elbow_flex"))
            q_ext[model_.getJointId("elbow_flex")] = 1.5;
        if (model_.existJointName("shoulder_lift"))
            q_ext[model_.getJointId("shoulder_lift")] = -0.3;
        if (model_.existJointName("shoulder_pan"))
            q_ext[model_.getJointId("shoulder_pan")] = 0.0;
        if (model_.existJointName("wrist_flex"))
            q_ext[model_.getJointId("wrist_flex")] = -0.5;
        q_ext = q_ext.cwiseMax(model_.lowerPositionLimit)
                    .cwiseMin(model_.upperPositionLimit);

        pinocchio::forwardKinematics(model_, data_, q_ext);
        pinocchio::updateFramePlacement(model_, data_, ee_id_);
        double r_ext = (data_.oMf[ee_id_].translation() - base_pos_).norm();

        // Длина кинематической цепи по смещениям суставов (НЕ центры масс)
        double link_sum = 0.0;
        for (pinocchio::JointIndex j = 1; j < (pinocchio::JointIndex)model_.njoints; ++j) {
            link_sum += model_.jointPlacements[j].translation().norm();
        }

        // Семплирование случайных конфигураций для настоящего r_max (с учётом лимитов)
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> uni(0.0, 1.0);
        double r_sampled = 0.0;
        const int N = 2000;
        for (int s = 0; s < N; ++s) {
            Eigen::VectorXd q = q_current_;
            for (int i = 0; i < model_.nq; ++i) {
                q[i] = model_.lowerPositionLimit[i] +
                       uni(rng) * (model_.upperPositionLimit[i] -
                                   model_.lowerPositionLimit[i]);
            }
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacement(model_, data_, ee_id_);
            double r = (data_.oMf[ee_id_].translation() - base_pos_).norm();
            r_sampled = std::max(r_sampled, r);
        }

        r_max_ = std::min(link_sum * 1.02, r_sampled * 0.95);
        if (r_max_ > link_sum * 1.05) r_max_ = link_sum * 1.05;
        RCLCPP_INFO(get_logger(),
            "Workspace: base=[%.3f,%.3f,%.3f], r_max=%.3f (r_ext=%.3f, links=%.3f, sampled=%.3f)",
            base_pos_.x(), base_pos_.y(), base_pos_.z(),
            r_max_, r_ext, link_sum, r_sampled);
    }

    void timerCallback() {
        // Интерполяция к целевой позе
        if (interp_step_ < interp_total_) {
            double alpha = (double)interp_step_ / interp_total_;
            // Сглаживание: quintic min-jerk, C2-гладкая (zero velocity & accel в endpoints)
            double smooth = alpha * alpha * alpha *
                            (10.0 - 15.0 * alpha + 6.0 * alpha * alpha);
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

    void publishTargetMarker(const Eigen::Vector3d &pos, IKStatus status) {
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
        switch (status) {
            case IKStatus::Converged:
                m.color.r = 0.2; m.color.g = 0.9; m.color.b = 0.2; break;
            case IKStatus::Approximate:
                m.color.r = 1.0; m.color.g = 0.85; m.color.b = 0.0; break;
            case IKStatus::Projected:
                m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.0; break;
            case IKStatus::Failed:
                m.color.r = 0.9; m.color.g = 0.1; m.color.b = 0.1; break;
        }
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration(0, 0);
        marker_pub_->publish(m);
    }

    void publishEEPose(const Eigen::Vector3d& ee_pos, IKStatus status) {
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = now();
        p.header.frame_id = "base_link";
        p.pose.position.x = ee_pos.x();
        p.pose.position.y = ee_pos.y();
        p.pose.position.z = ee_pos.z();
        p.pose.orientation.w = 1.0;
        ee_pose_pub_->publish(p);
    }

    void publishCollisionMarker() {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now();
        m.header.frame_id = "base_link";
        m.ns = "self_collision";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        if (worst_pair_idx_ < 0) {
            // No collision detected — delete marker
            m.action = visualization_msgs::msg::Marker::DELETE;
            collision_marker_pub_->publish(m);
            return;
        }
        m.action = visualization_msgs::msg::Marker::ADD;
        Eigen::Vector3d pa = getLinkPosition(collision_pairs_[worst_pair_idx_].first);
        Eigen::Vector3d pb = getLinkPosition(collision_pairs_[worst_pair_idx_].second);
        Eigen::Vector3d mid = 0.5 * (pa + pb);
        m.pose.position.x = mid.x();
        m.pose.position.y = mid.y();
        m.pose.position.z = mid.z();
        m.pose.orientation.w = 1.0;
        double penetration = std::max(0.0, -worst_d_);
        double scale = std::max(0.03, penetration);
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.r = 0.9; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 1.0;
        collision_marker_pub_->publish(m);
    }

    // Pinocchio
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_id_;

    // Self-collision: non-adjacent pairs (link_name, link_name)
    const std::vector<std::pair<std::string,std::string>> collision_pairs_ = {
        {"base_link", "lower_arm_link"},
        {"base_link", "wrist_link"},
        {"base_link", "gripper_link"},
        {"shoulder_link", "lower_arm_link"},
        {"shoulder_link", "wrist_link"},
        {"upper_arm_link", "wrist_link"},
        {"upper_arm_link", "gripper_link"},
        {"lower_arm_link", "gripper_link"},
    };
    // Sphere radii per link (hardcoded fallback — URDF <collision> for SO-101
    // is mostly visual meshes, not analytic primitives)
    const std::map<std::string, double> sphere_radii_ = {
        {"base_link", 0.030},
        {"shoulder_link", 0.025},
        {"upper_arm_link", 0.022},
        {"lower_arm_link", 0.020},
        {"wrist_link", 0.018},
        {"gripper_link", 0.025},
    };
    // Worst self-collision pair in current IK iteration (for marker)
    int worst_pair_idx_ = -1;
    double worst_d_ = std::numeric_limits<double>::infinity();
    std::vector<std::string> joint_names_;
    Eigen::VectorXd q_current_;
    Eigen::VectorXd q_target_;
    Eigen::VectorXd q_start_interp_;
    int interp_step_ = 0;
    int interp_total_ = 1;
    Eigen::Vector3d base_pos_;
    double r_max_ = 0.0;

    // ROS 2
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_marker_pub_;
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
