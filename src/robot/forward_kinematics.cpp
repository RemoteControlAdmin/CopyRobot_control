# include "robot/forward_kinematics.hpp"

namespace robot_lib{
    ForwardKinematics::ForwardKinematics(){
        
    }


    Eigen::Matrix3d ForwardKinematics::formatrix_cal(Eigen::Matrix3d forward){ //初期化で呼び出しOK
        const double factor = 0.050 / 3.0;
        forward <<
            factor * sqrt(3),        factor * 0.0,            factor * (-sqrt(3)),
            factor * (-1.0),         factor * 2.0,            factor * (-1.0),
            factor * (1.0 / 0.115),  factor * (1.0 / 0.115),  factor * (1.0 / 0.115);
        return forward;
    }

    Eigen::Vector3d ForwardKinematics::forrobot_velocity_cal(Eigen::Matrix3d forward, Eigen::Vector3d forwheelvelocity/*invwheelvelocity*/, Eigen::Vector3d forrobotvelocity){
        forrobotvelocity = forward * forwheelvelocity;
        return forrobotvelocity;
    }

    Eigen::Matrix3d ForwardKinematics::fortransmatrix_cal(Eigen::Matrix3d forward_trans, std::vector<double> copy_data){
        forward_trans <<
            cos(copy_data[2]),           cos(copy_data[2] + M_PI / 2),    0,
            sin(copy_data[2]),           sin(copy_data[2] + M_PI / 2),    0,
            0,                               0,                                     1; // 最後の要素は1に簡略化済み
        return forward_trans;
    }

    Eigen::Vector3d ForwardKinematics::forglobal_velocity_cal(Eigen::Matrix3d forward_trans, Eigen::Vector3d forrobotvelocity, Eigen::Vector3d forglobal_velocity){
        forglobal_velocity = forward_trans * forrobotvelocity;
        return forglobal_velocity;
    }



}