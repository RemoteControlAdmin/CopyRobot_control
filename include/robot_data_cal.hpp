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

    RobotData convert_robotdata(RobotData robotdata);
    std::vector<double> err_robotposition_cal(RobotData robotdata);

    private:
    std::vector<double> convert_m_rad(std::vector<double> data);



}; // class RobotControl
} // namespace robotcontrol

#endif // ROBOT_DATA_CAL_HPP