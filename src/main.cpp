# include <vector>
# include <cmath>
# include <iostream>
# include "common.hpp"
# include "motor_control.hpp"
# include "robot_data_cal.hpp"
# include "robot_control.hpp"
# include "inverce_kinematics.hpp"
# include "udp_connect.hpp"

//構造体定義
RobotData robotdata;
Mat3x1 mat3x1;
Mat3X3 mat3x3;

int main(){
    //クラス定義
    motorcontrol::MotorControl motor_control{}; // Motor設定
    
    robotcontrol::RobotDataCal robot_data_cal{};

    robotcontrol::RobotControl robot_control{};

    robotkinematics::InverceKinematics inverce_kinematics{};

    udp_lib::UdpConnect udpConnection("192.168.11.29", 4102, 6); // UDP初期化


    mat3x3.inverse = inverce_kinematics.invmatrix_cal();
    robotdata.last_err_data =  {0,0,0};

    while(1){
        //UDP受信
        // pass
        robotdata.master_data = std::vector<double> {0,0,0,0,0,0};
        robotdata.copy_data = std::vector<double> {0,0,0,0,0,0};

        robotdata = robot_data_cal.convert_robotdata(robotdata);//Get MasterRobot's Position for manual path trajectory
        robotdata.err_data = robot_data_cal.err_robotposition_cal(robotdata);

        mat3x1.velocity_data  = robot_control.CLPositionControllerPIDCal(robotdata);
        mat3x1.velocity_data = robot_control.VelocityLimitationCal(mat3x1.velocity_data);

        mat3x3.inverse_trans = inverce_kinematics.invtransmatrix_cal(robotdata.copy_data);
        mat3x1.invrobotvelocity = inverce_kinematics.invrobot_velocity_cal(mat3x3.inverse_trans, mat3x1.velocity_data);
        // 初期化で実行
        mat3x1.invwheelvelocity = inverce_kinematics.invwheel_velocity_cal(mat3x3.inverse, mat3x1.invrobotvelocity );

       mat3x1.mortor_voltage = motor_control.convert_wheeltovoltage(mat3x1.invwheelvelocity);
        
        robotdata.last_err_data = robotdata.err_data;
	std::cout << "test";

    }

}
