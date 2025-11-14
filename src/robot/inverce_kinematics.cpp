# include "robot/inverce_kinematics.hpp"

namespace robot_lib{
    InverceKinematics::InverceKinematics(){
        
    }

    Eigen::Matrix3d InverceKinematics::invtransmatrix_cal(std::vector<double> copy_data){
        double denominator = cos(copy_data[2]) * sin(copy_data[2] + M_PI/2) - sin(copy_data[2]) * cos(copy_data[2] + M_PI/2);
        inverse_trans <<
            (+sin(copy_data[2] + M_PI/2)) / denominator,   (-cos(copy_data[2] + M_PI/2)) / denominator,  0,
            (-sin(copy_data[2])) / denominator,            (+cos(copy_data[2])) / denominator,           0,
            0,                                      0,                                     1; // 最後の要素は1に簡略化済み
        return inverse_trans;
    }

    Eigen::Vector3d InverceKinematics::invrobot_velocity_cal(Eigen::Matrix3d inverse_trans, Eigen::Vector3d velocity_data){
        invrobotvelocity = inverse_trans * velocity_data;
        return invrobotvelocity;
    }

    Eigen::Matrix3d InverceKinematics::invmatrix_cal(){ //初期化で呼び出しOK
        const double factor = 3.0 / 0.050;
        inverse << 
            factor * (1.0 / (2.0 * sqrt(3))),   factor * (-1.0 / 6.0),   factor * (0.115 / 3.0),
            factor * 0.0,                       factor * (1.0 / 3.0),    factor * (0.115 / 3.0),
            factor * (-1.0 / (2.0 * sqrt(3))),  factor * (-1.0 / 6.0),   factor * (0.115 / 3.0);
        return inverse;
    }

    Eigen::Vector3d InverceKinematics::invwheel_velocity_cal(Eigen::Matrix3d inverse, Eigen::Vector3d invrobotvelocity){
        invwheelvelocity = inverse * invrobotvelocity;
        return invwheelvelocity;
    }

}