#ifndef INVERCE_KINEMATICS_HPP
#define INVERCE_KINEMATICS_HPP
#define _USE_MATH_DEFINES  // M_PIを使用するために必要
# include <vector>
# include <cmath>
# include "common.hpp"

namespace robotkinematics {

    class InverceKinematics {

    public:

    InverceKinematics();

    Eigen::Matrix3d invtransmatrix_cal(std::vector<double> copy_data);

    Eigen::Vector3d invrobot_velocity_cal(Eigen::Matrix3d inverse_trans, Eigen::Vector3d velocity_data);
    
    Eigen::Matrix3d invmatrix_cal();

    Eigen::Vector3d invwheel_velocity_cal(Eigen::Vector3d velocity_data, Eigen::Vector3d invrobotvelocity);

    private:
    Eigen::Matrix3d inverse_trans;
    Eigen::Vector3d invrobotvelocity;
    Eigen::Matrix3d inverse;
    Eigen::Vector3d invwheelvelocity;

}; // class RobotControl
} // namespace robotcontrol

#endif // INVERCE_KINEMATICS_HPP