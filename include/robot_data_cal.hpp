#ifndef ROBOT_DATA_CAL_HPP
#define ROBOT_DATA_CAL_HPP
#define _USE_MATH_DEFINES  // M_PIを使用するために必要
# include <vector>
# include <cmath>

# include "common.hpp"

namespace robotcontrol {

    class RobotDataCal {

    public:


    RobotDataCal();

    std::pair<std::vector<double>, std::vector<double>, std::vector<double>> convert_robotdata(std::vector<double> master_data,
        std::vector<double> copy_data, std::vector<double> partner_master_data);
    std::array<double,3> err_robotposition_cal(std::vector<double> master_data, std::vector<double> copy_data);
    std::vector<double> MRobot_Linear_PositionCal(std::vector<double> data, int time);
    private:
    std::vector<double> convert_m_rad(std::vector<double> data);



}; // class RobotControl
} // namespace robotcontrol

#endif // ROBOT_DATA_CAL_HPP