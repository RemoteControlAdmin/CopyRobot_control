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

    ControlData CLPositionControllerPIDCal(RobotData robotdata);
    Eigen::Vector3d VelocityLimitationCal(Eigen::Vector3d velocity_data);


    private:
    

    std::vector<double> Integral;
    std::vector<double> Derivative;

    Eigen::Vector3d velocity_data;
    
    double Kp, Ki, Kd;




}; // class RobotControl
} // namespace robotcontrol

#endif // ROBOT_CONTROL_HPP