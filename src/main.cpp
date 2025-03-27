# include <thread>
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
    /*
    * インスタンス作成 Instance creation
    */
    motorcontrol::MotorControl motor_control{};     // Motor setup
    robotcontrol::RobotDataCal robot_data_cal{};    // data calculation about robot
    robotcontrol::RobotControl robot_control{};     // robot control
    robotkinematics::InverceKinematics inverce_kinematics{}; // inverce kinematics

    /*
    * UDP設定 UDP settings
    * https://planet-louse-95d.notion.site/1c047abc426580638ceff46276d2df59?pvs=4
    */
    udp_lib::UdpConnect udpConnection_raspberrypi("192.168.11.29", 4102, 6); // UDP初期化
    udp_lib::UdpConnect udpConnection_from_master("0.0.0.0", 40011, 6); // from Master Robot
    udpConnection_from_master.udp_bind();
    udp_lib::UdpConnect udpConnection_from_copy("0.0.0.0", 41031, 6); // from Master Robot
    udpConnection_from_copy.udp_bind();
    /*
    * ローカル変数定義　local variable definition
    */
    mat3x3.inverse = inverce_kinematics.invmatrix_cal();    // inverce matrix definition
    robotdata.last_err_data =  {0,0,0};     // initialize about last error data
    robotdata.master_data =  {0,0,0,0,0,0};     // initialize about last master data
    robotdata.copy_data =  {0,0,0,0,0,0};     // initialize about last copy data
    // dt計算用 dt calculation
    std::chrono::high_resolution_clock::time_point last_clock;  // 前回の時刻 previous time
    std::chrono::microseconds micro_last_clock; 
    std::chrono::high_resolution_clock::time_point current_clock; // 現在の時刻　current time
    std::chrono::microseconds micro_current_clock;
    std::chrono::microseconds micro_dt; //dt
    std::chrono::microseconds dt(10*1000); // calculation cycle

    /*
    * ============== 処理 process ==============
    */
    last_clock = std::chrono::high_resolution_clock::now(); // 現在時刻を取得 get the current time
    micro_last_clock = std::chrono::duration_cast<std::chrono::microseconds>(last_clock.time_since_epoch()); // μs（マイクロ秒）単位で取得 convert to micro s

    /*
    * ============== main roop ==============
    */
    while(1){
        /*
        * UDP受信 UDP recive
        */
        std::pair<std::vector<double>, int> receiveddata_master = udpConnection_from_master.udp_recv(); // from master robot
        //std::pair<std::vector<double>, int> receiveddata_copy = udpConnection_from_copy.udp_recv();     // from copy robot (own)
        robotdata.master_data =receiveddata_master.first; // first is robot data, second is robot time information maked by raspberrypi
        robotdata.copy_data = std::vector<double>{0,0,0,0,0,0};// receiveddata_copy.first;

        /*
        *  data calculation about robot
        */
        robotdata = robot_data_cal.convert_robotdata(robotdata);    //Get MasterRobot's Position for manual path trajectory
        robotdata.master_data = robot_data_cal.MRobot_Linear_PositionCal(robotdata.master_data);
        robotdata.err_data = robot_data_cal.err_robotposition_cal(robotdata);
        
        /*
        * robot control
        */
        mat3x1.velocity_data  = robot_control.CLPositionControllerPIDCal(robotdata);        // PID control
        mat3x1.velocity_data = robot_control.VelocityLimitationCal(mat3x1.velocity_data);   // limit

        /*
        * inverce kinematics
        */
        mat3x3.inverse_trans = inverce_kinematics.invtransmatrix_cal(robotdata.copy_data);
        mat3x1.invrobotvelocity = inverce_kinematics.invrobot_velocity_cal(mat3x3.inverse_trans, mat3x1.velocity_data);
        mat3x1.invwheelvelocity = inverce_kinematics.invwheel_velocity_cal(mat3x3.inverse, mat3x1.invrobotvelocity );

        /*
        * move robot
        */
        mat3x1.mortor_voltage = motor_control.convert_wheeltovoltage(mat3x1.invwheelvelocity);
        motor_control.EnableMotorDrive(mat3x1.mortor_voltage);

        robotdata.last_err_data = robotdata.err_data;

        
        /*
        *  adjusting the cycle
        */
        current_clock = std::chrono::high_resolution_clock::now();// 現在時刻を取得
        micro_current_clock = std::chrono::duration_cast<std::chrono::microseconds>(current_clock.time_since_epoch());// μs（マイクロ秒）単位で取得
        micro_dt = micro_current_clock - micro_last_clock;
        if (micro_dt <= dt){
            std::this_thread::sleep_for(dt - micro_dt);
        }
        current_clock = std::chrono::high_resolution_clock::now();
        micro_current_clock = std::chrono::duration_cast<std::chrono::microseconds>(current_clock.time_since_epoch());
            micro_dt = micro_current_clock - micro_last_clock;
        micro_last_clock = micro_current_clock;
        std::cout << "dt = " << micro_dt.count() << std::endl;

    }

}
