# include <thread>
#include <deque>
#include <mutex>
# include <vector>
# include <cmath>
# include <iostream>
# include "common.hpp"
# include "motor_control.hpp"
# include "robot_data_cal.hpp"
# include "robot_control.hpp"
# include "inverce_kinematics.hpp"
# include "udp_connect.hpp"

std::atomic<bool> stop_flag(false);// グローバル変数として定義
void end_task(int signum){
    if(signum == SIGINT) {
        std::cout << "\n[INFO] Ctrl+C detected. Exiting..." << std::endl;
        stop_flag = 1;
    }
}


int main(){
    // 強制終了処理　end proccess
    std::signal(SIGINT, end_task);

    /*
    * キュー作成
    * make queue
    */
    std::deque<std::pair<std::vector<double>, int>> deque_master;
    std::deque<std::pair<std::vector<double>, int>> deque_copy;
    std::mutex queue_mutex_master;
    std::mutex queue_mutex_copy;
    
    /*
    * インスタンス作成 Instance creation
    */
    motorcontrol::MotorControl motor_control{};     // Motor setup
    robotcontrol::RobotDataCal robot_data_cal{};    // data calculation about robot
    robotcontrol::RobotControl robot_control{};     // robot control
    robotkinematics::InverceKinematics inverce_kinematics{}; // inverce kinematics
    udp_lib::UdpCommunicator udp_communicator(deque_master, deque_copy, queue_mutex_master, queue_mutex_copy); // UDP communication
    /*
    * ローカル変数定義　local variable definition
    */
    //構造体定義
    RobotData robotdata;
    Mat3x1 mat3x1;
    Mat3X3 mat3x3;
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
    std::chrono::microseconds first_clock;  // 最初の時刻
    /*
    * ============== 処理 process ==============
    */
    last_clock = std::chrono::high_resolution_clock::now(); // 現在時刻を取得 get the current time
    micro_last_clock = std::chrono::duration_cast<std::chrono::microseconds>(last_clock.time_since_epoch()); // μs（マイクロ秒）単位で取得 convert to micro s
    first_clock = micro_last_clock;

    /*
    * 受信スレッドの開始
    */
    std::thread udp_thread_master(&udp_lib::UdpCommunicator::recive_thread_from_master, &udp_communicator); // UDP receive thread from master
    std::thread udp_thread_copy(&udp_lib::UdpCommunicator::recive_thread_from_copy, &udp_communicator); // UDP receive thread from master
    /*
    * ============== main roop ==============
    */
    while(!stop_flag){
        
        /*
        * queue取り出し
        * get queue
        */
        // master
        {
            std::lock_guard<std::mutex> lock(queue_mutex_master);
            if (!deque_master.empty()){
                robotdata.master_data = deque_master.front().first;
                deque_master.pop_front();
            }
        } // unlock用
        // copy
        {
            std::lock_guard<std::mutex> lock(queue_mutex_copy);
            if (!deque_copy.empty()){
                robotdata.copy_data = deque_copy.front().first;
                deque_copy.pop_front();
            }
        } // unlock用

        /*
        *  data calculation about robot
        */
        robotdata = robot_data_cal.convert_robotdata(robotdata);    //Get MasterRobot's Position for manual path trajectory
        robotdata.master_data = robot_data_cal.MRobot_Linear_PositionCal(robotdata.master_data, (micro_current_clock - first_clock).count());
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

        std::cout << "Mpx = "<< robotdata.master_data[0] << "Mpy = "<< robotdata.master_data[1] << "Mpt = "<< robotdata.master_data[2] << std::endl;
        std::cout << "Cpx = "<< robotdata.copy_data[0] << "Cpy = "<< robotdata.copy_data[1] << "Cpt = "<< robotdata.copy_data[2] << std::endl;
        //std::cout << "dt = " << micro_dt.count() << std::endl;

    }
    // 終了前の後始末
    udp_thread_master.join(); // UDP receive thread from master
    udp_thread_copy.join(); // UDP receive thread from copy
    motor_control.DisableMotorDrive();
    std::cout << "[INFO] Program terminated gracefully." << std::endl;
    return 0;

}
