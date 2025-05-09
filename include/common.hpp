#ifndef COMMON_HPP
#define COMMON_HPP

#define _USE_MATH_DEFINES  // M_PIを使用するために必要
#include <atomic>
# include <vector>
# include <cmath>
#include <Eigen/Dense>

#include <signal.h>   // signal, SIGINTなど
#include <fcntl.h>    // open, O_WRONLYなど
#include <unistd.h>   // close, writeなど
#include <string.h>   // strlenなど（文字列処理）
#include <csignal>


extern std::atomic<bool> stop_flag;

struct RobotData {
    std::vector<double> master_data;
    std::vector<double> copy_data;
    std::array<double,3> err_data;
    std::array<double,3> last_err_data;
};

struct Mat3x1 {
    Eigen::Vector3d velocity_data;
    
    Eigen::Vector3d invrobotvelocity;
    Eigen::Vector3d invwheelvelocity;

    Eigen::Vector3d forrobotvelocity;
    Eigen::Vector3d forwheelvelocity;

    Eigen::Vector3d mortor_voltage;
};



struct Mat3X3{
    Eigen::Matrix3d  inverse_trans;
    Eigen::Matrix3d  inverse;
    Eigen::Matrix3d  forward_trans;
    Eigen::Matrix3d  forward;
    
};
#endif // COMMON_HPP