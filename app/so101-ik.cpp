#include <iostream>
#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>  // первым — предотвращает конфликты Boost
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"            // ← frames (множ. число!)
#include "pinocchio/algorithm/jacobian.hpp"          // ← для computeJointJacobians
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path/to/so101_new_calib.urdf>" << std::endl;
        return 1;
    }

    const std::string urdf_path = argv[1];

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    std::cout << "Model: " << model.name
              << "  nq=" << model.nq << "  nv=" << model.nv << std::endl;

    // Имя фрейма энд-эффектора SO-101
    const std::string EE_FRAME = "gripper_frame_link";
    const pinocchio::FrameIndex ee_id = model.getFrameId(EE_FRAME);

    // Начальная конфигурация
    Eigen::VectorXd q = pinocchio::neutral(model);

    // Целевая позиция
    Eigen::Vector3d target_pos(0.2, 0.0, 0.15);

    const int max_iter = 100;
    const double eps = 1e-4;
    const double dt = 1e-1;
    const double damp = 1e-6;

    for (int i = 0; i < max_iter; ++i) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacement(model, data, ee_id);

        Eigen::Vector3d cur_pos = data.oMf[ee_id].translation();
        Eigen::Vector3d err = target_pos - cur_pos;

        if (err.norm() < eps) {
            std::cout << "Converged at iter " << i
                      << ", error = " << err.norm() << std::endl;
            break;
        }

        // Якобиан: сначала общие якобианы суставов, потом фрейма
        Eigen::MatrixXd J(6, model.nv);
        J.setZero();
        pinocchio::computeJointJacobians(model, data, q);
        pinocchio::getFrameJacobian(model, data, ee_id,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);

        // Берём только позиционные строки (первые 3)
        Eigen::MatrixXd J_pos = J.topRows(3);

        // Демпфированный псевдообратный шаг
        Eigen::MatrixXd JJt = J_pos * J_pos.transpose();
        JJt.diagonal().array() += damp;
        Eigen::VectorXd v = J_pos.transpose() * JJt.ldlt().solve(err);

        q = pinocchio::integrate(model, q, dt * v);
    }

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, ee_id);

    std::cout << "Final q (rad): " << q.transpose() << std::endl;
    std::cout << "Final EE pos:  " << data.oMf[ee_id].translation().transpose() << std::endl;
    std::cout << "Target pos:   " << target_pos.transpose() << std::endl;

    return 0;
}
