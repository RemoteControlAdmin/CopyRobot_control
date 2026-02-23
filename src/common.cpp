// common.cpp
#include "common.hpp"

std::atomic<bool> stop_flag{false};

// RobotData構造体のコンストラクタ定義
RobotData::RobotData()
    : master_data(6, 0.0),
      copy_data(6, 0.0),
      partner_master_data(6, 0.0),
      remote_copy_data(6, 0.0),
      err_data{{0.0, 0.0, 0.0}},
      last_err_data{{0.0, 0.0, 0.0}},
      force_actual_data(4, 0.0),
      force_virtual_data(6, 0.0),
      force_ideal_data(6, 0.0),
      force_udp_data(4, 0.0)
{}

// Mat3x1構造体のコンストラクタ定義
Mat3x1::Mat3x1()
    : velocity_data(Eigen::Vector3d::Zero()),
      invrobotvelocity(Eigen::Vector3d::Zero()),
      invwheelvelocity(Eigen::Vector3d::Zero()),
      forrobotvelocity(Eigen::Vector3d::Zero()),
      forwheelvelocity(Eigen::Vector3d::Zero()),
      mortor_voltage(Eigen::Vector3d::Zero())
{}

// Mat3X3構造体のコンストラクタ定義
Mat3X3::Mat3X3()
    : inverse_trans(Eigen::Matrix3d::Zero()),
      inverse(Eigen::Matrix3d::Zero()),
      forward_trans(Eigen::Matrix3d::Zero()),
      forward(Eigen::Matrix3d::Zero())
{}
