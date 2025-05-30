# include <thread>
# include <deque>
# include <mutex>
# include <vector>
# include <cmath>
# include <iostream>
# include <iomanip>
# include "common.hpp"
# include "motor_control.hpp"
# include "robot_data_cal.hpp"
# include "robot_control.hpp"
# include "inverce_kinematics.hpp"
# include "udp_connect.hpp"
# include "force_get.hpp"

std::atomic<bool> stop_flag(false);// グローバル変数として定義
void end_task(int signum){
    if(signum == SIGINT) {
        std::cout << "\n[INFO] Ctrl+C detected. Exiting..." << std::endl;
        stop_flag = 1;
    }
}

void show_data(RobotData robotdata, Eigen::Vector3d velocity_data, int dt){
    std::cout << "\033[2J\033[1;1H"; // Clear the console
    std::cout << "================== show data ==================" << std::endl;/*
    std::cout << std::left << std::setw(20) << ("Mpx = " + std::to_string(robotdata.master_data[0])) 
              << std::left << std::setw(20) << ("Mpy = " + std::to_string(robotdata.master_data[1]))
              << std::left << std::setw(20) << ("Mpt = " + std::to_string(robotdata.master_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Cpx = " + std::to_string(robotdata.copy_data[0])) 
              << std::left << std::setw(20) << ("Cpy = " + std::to_string(robotdata.copy_data[1]))
              << std::left << std::setw(20) << ("Cpt = " + std::to_string(robotdata.copy_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Errx = " + std::to_string(robotdata.err_data[0])) 
              << std::left << std::setw(20) << ("Erry = " + std::to_string(robotdata.err_data[1]))
              << std::left << std::setw(20) << ("Errt = " + std::to_string(robotdata.err_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Vx = " + std::to_string(velocity_data[0])) 
              << std::left << std::setw(20) << ("Vy = " + std::to_string(velocity_data[1]))
              << std::left << std::setw(20) << ("Vt = " + std::to_string(velocity_data[2]))
              << std::endl;*/
    std::cout << std::left << std::setw(20) << ("FEactM = " + std::to_string(robotdata.force_actual_data[0])) 
              << std::left << std::setw(20) << ("FEVir = " + std::to_string(robotdata.force_virtual_data[4]))
              << std::left << std::setw(20) << ("F = " + std::to_string(robotdata.force_ideal_data[4]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("dt = " + std::to_string(dt)) 
              << std::endl;
    std::cout << "==============================================" << std::endl;
}


int main(){
    // 強制終了処理　end proccess
    std::signal(SIGINT, end_task);

    /*
    * キュー作成
    * make queue
    */
    std::deque<std::pair<std::vector<double>, int64_t>> deque_master;
    std::deque<std::pair<std::vector<double>, int64_t>> deque_copy;
    std::mutex queue_mutex_master;
    std::mutex queue_mutex_copy;
    
    /*
    * インスタンス作成 Instance creation
    */
    motorcontrol::MotorControl motor_control{};     // Motor setup
    robotcontrol::RobotDataCal robot_data_cal{};    // data calculation about robot
    robotcontrol::RobotControl robot_control{};     // robot control
    robotkinematics::InverceKinematics inverce_kinematics{}; // inverce kinematics
    forceget::ForceActual force_actual{}; // force actual
    forceget::ForceIdeal force_ideal{}; // force ideal
    udp_lib::UdpCommunicator udp_communicator(deque_master, deque_copy, queue_mutex_master, queue_mutex_copy); // UDP communication
    udp_lib::UdpConnect udpConnection_raspberrypi("192.168.11.29", 65000, 26); // UDP初期化
    /*
    * ローカル変数定義　local variable definition
    */
    int64_t send_time = 0; // 送信時間 send time
    //構造体宣言
    RobotData robotdata;
    Mat3x1 mat3x1;
    Mat3X3 mat3x3;
    mat3x3.inverse = inverce_kinematics.invmatrix_cal();    // inverce matrix definition

    // 一時的にcopyとpartnerのデータを保持するための変数
    std::vector<double> temp_copy_data{6, 0.0};
    // 送信用変数
    std::vector<double> send_data(26, 0.0);
    // dt計算用 dt calculation
    std::chrono::high_resolution_clock::time_point last_clock;  // 前回の時刻 previous time
    std::chrono::microseconds micro_last_clock; 
    std::chrono::high_resolution_clock::time_point current_clock; // 現在の時刻　current time
    std::chrono::microseconds micro_current_clock;
    std::chrono::microseconds micro_dt; //dt
    std::chrono::microseconds dt(15*1000); // calculation cycle
    std::chrono::microseconds first_clock;  // first clock
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
                send_time = deque_master.front().second;
                robotdata.last_master_data = robotdata.master_data;
                deque_master.pop_front();
            }
            else{
                robotdata.master_data = robotdata.last_master_data;
            }
        } // unlock用
        // copy
        {
            std::lock_guard<std::mutex> lock(queue_mutex_copy);
            if (!deque_copy.empty()){
                temp_copy_data = deque_copy.front().first;
                robotdata.copy_data.assign(temp_copy_data.begin(), temp_copy_data.begin()+6);
                robotdata.partner_master_data.assign(temp_copy_data.begin()+6, temp_copy_data.end());
                robotdata.last_copy_data = robotdata.copy_data;
                robotdata.last_partner_master_data = robotdata.partner_master_data;
                deque_copy.pop_front();
            }
	        else{
	    	    robotdata.copy_data = robotdata.last_copy_data;
                robotdata.partner_master_data = robotdata.last_partner_master_data;
            }
        } // unlock用
        /*
        *  force getting
        */
        robotdata.force_actual_data = force_actual.FEActCal();
        robotdata.force_virtual_data  = force_ideal.FEVirCal(robotdata.partner_master_data, robotdata.copy_data);
        robotdata.force_ideal_data = force_ideal.FIdCal(robotdata.partner_master_data, robotdata.master_data);
        /*
        *  data calculation about robot
        */
        robotdata = robot_data_cal.convert_robotdata(robotdata);    //Get MasterRobot's Position for manual path trajectory
        //robotdata.master_data = robot_data_cal.MRobot_Linear_PositionCal(robotdata.master_data, (micro_current_clock - first_clock).count());
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

        // デバック用
        send_data.clear();
        send_data.insert(send_data.end(), robotdata.err_data.begin(), robotdata.err_data.end()); // 3
        send_data.insert(send_data.end(), mat3x1.velocity_data.data(), mat3x1.velocity_data.data()+3); // 3
        send_data.insert(send_data.end(), mat3x1.invwheelvelocity.data(), mat3x1.invwheelvelocity.data()+3); // 3
        send_data.insert(send_data.end(), mat3x1.mortor_voltage.data(), mat3x1.mortor_voltage.data()+3); // 3
        send_data.insert(send_data.end(), robotdata.force_actual_data.begin(), robotdata.force_actual_data.end()); // 2
        send_data.insert(send_data.end(), robotdata.force_virtual_data.begin(), robotdata.force_virtual_data.end()); // 6
        send_data.insert(send_data.end(), robotdata.force_ideal_data.begin(), robotdata.force_ideal_data.end()); // 6
        udpConnection_raspberrypi.udp_send(send_data, send_time);
        show_data(robotdata, mat3x1.velocity_data, micro_dt.count());
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
        micro_current_clock = std::chrono::duration_cast<std::chrono::microseconds>(current_clock.time_since_epoch());// μs（マイクロ秒）単位で取得
        micro_dt = micro_current_clock - micro_last_clock;
        micro_last_clock = micro_current_clock;
        //std::cout << "dt = " << micro_dt.count() << std::endl;
    }
    // 終了前の後始末
    udp_thread_master.join(); // UDP receive thread from master
    udp_thread_copy.join(); // UDP receive thread from copy
    motor_control.DisableMotorDrive();
    std::cout << "[INFO] Program terminated gracefully." << std::endl;
    return 0;

}
