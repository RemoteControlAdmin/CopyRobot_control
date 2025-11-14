#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP
#define _USE_MATH_DEFINES  // M_PIを使用するために必要
# include <vector>
# include <cmath>
# include "common.hpp"

namespace robot_lib {

    class ForwardKinematics {

    public:

    ForwardKinematics();

    Eigen::Matrix3d formatrix_cal(Eigen::Matrix3d inverse);

    Eigen::Vector3d forrobot_velocity_cal(Eigen::Matrix3d inverse_trans, Eigen::Vector3d velocity_data, Eigen::Vector3d invrobotvelocity);

    Eigen::Matrix3d fortransmatrix_cal(Eigen::Matrix3d forward_trans, std::vector<double> copy_data);

    Eigen::Vector3d forglobal_velocity_cal(Eigen::Matrix3d inverse_trans, Eigen::Vector3d velocity_data, Eigen::Vector3d invrobotvelocity);
    

    private:
    



}; // class RobotControl
} // namespace robotcontrol

#endif // FORWARD_KINEMATICS_HPP