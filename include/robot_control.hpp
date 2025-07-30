#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP
#define _USE_MATH_DEFINES  // M_PIを使用するために必要
# include <vector>
# include <cmath>

# include "common.hpp"

namespace robotcontrol {

    class RobotControl {

    public:

    RobotControl();

    void chenge_pid(double kp, double ki, double kd);

    Eigen::Vector3d unilateral_force_control(std::vector<double> force_actual_data, std::vector<double> force_virtual_data,
         std::vector<double> force_ideal_data, std::vector<double> master_data, int microdt);

    Eigen::Vector3d CLPositionControllerPIDCal(std::array<double, 3> err_data, std::array<double, 3> last_err_data, int microdt);

    bool VelocityLimitationCal(Eigen::Vector3d velocity_data);


    private:
    

    std::vector<double> Integral;
    std::vector<double> Derivative;

    Eigen::Vector3d velocity_data;

    Eigen::Vector3d actual_vector_data;
    Eigen::Vector3d ideal_vector_data;
    Eigen::Vector3d force_err;
    Eigen::Vector3d last_force_err;
    Eigen::Vector3d pid_force_data;

    Eigen::Vector3d force_pos_data;

    double Kp, Ki, Kd;
    double Kp_force, Ki_force, Kd_force;




}; // class RobotControl
} // namespace robotcontrol

#endif // ROBOT_CONTROL_HPP