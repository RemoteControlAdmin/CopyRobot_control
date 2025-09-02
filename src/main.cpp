# include <thread>
# include <deque>
# include <mutex>
# include <vector>
# include <cmath>
# include <iterator>
# include <iostream>
# include <cstdlib>
# include "common.hpp"
# include "motor_control.hpp"
# include "robot_data_cal.hpp"
# include "robot_control.hpp"
# include "inverce_kinematics.hpp"
# include "udp_connect.hpp"
# include "force_get.hpp"
#include "data_logger.hpp"


void end_task(int signum){
    if(signum == SIGINT) {
        std::cout << "\n[INFO] Ctrl+C detected. Exiting..." << std::endl;
        stop_flag = 1;
    }
}

void set_cpu_governor(const std::string& governor) {
    // CPUのガバナーを設定する関数
    std::string command = "sudo cpufreq-set -g" + governor;
    int ret = system(command.c_str());
    if (ret != 0) {
        std::cerr << "Error setting CPU governor to " << governor << std::endl;
    } else {
        std::cout << "\n[INFO] CPU governor set to " << governor << std::endl;
    }
}


int main(){
    set_cpu_governor("performance");
    std::cout << "================== runnning ==================" << std::endl;
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
    //forceget::ForceActual force_actual{}; // force actual
    //forceget::ForceIdeal force_ideal{}; // force ideal
    udp_lib::UdpCommunicator udp_communicator(deque_master, deque_copy, queue_mutex_master, queue_mutex_copy); // UDP communication
    //udp_lib::UdpConnect udpConnection_raspberrypi("192.168.11.202", 65000, 26); // UDP初期化
    DataLogger data_logger{}; // data logger
    /*
    * ローカル変数定義　local variable definition
    */
    int64_t send_time = 0; // 送信時間 send time
    // システムクロック定義
    std::chrono::nanoseconds nano_receive_clock = std::chrono::nanoseconds(0); // 受信時間 receive time 
    //構造体宣言
    RobotData robotdata;
    Mat3x1 mat3x1;
    Mat3X3 mat3x3;
    mat3x3.inverse = inverce_kinematics.invmatrix_cal();    // inverce matrix definition

    // 一時的にcopyとpartnerのデータを保持するための変数
    std::vector<double> temp_copy_data(12, 0.0);
    std::tuple <std::vector<double>, std::vector<double>, std::vector<double>> temp_convert_data;

    // dt計算用 dt calculation
    std::chrono::high_resolution_clock::time_point last_clock;  // 前回の時刻 previous time
    std::chrono::microseconds micro_last_clock; 
    std::chrono::high_resolution_clock::time_point current_clock; // 現在の時刻　current time
    std::chrono::microseconds micro_current_clock;
    std::chrono::microseconds micro_dt(10*1000); //dt
    std::chrono::microseconds dt(10*1000); // calculation cycle
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
    int safety_count = 0; // 安全カウント
    std::vector<int> not_get_count = {0,0}; // データが取得できなかった回数
    while(!stop_flag){
        /*
        * queue取り出し
        * get queue
        */
        // master
        {
            std::lock_guard<std::mutex> lock(queue_mutex_master);
            if (!deque_master.empty()){
                std::pair<std::vector<double>, int64_t> getdata = deque_master.front();
                robotdata.master_data = getdata.first;
                send_time = getdata.second;
                robotdata.last_master_data = robotdata.master_data;
                deque_master.pop_front();
            }
            else{
                robotdata.master_data = robotdata.last_master_data;
                not_get_count[0]++;
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
                not_get_count[1]++;
            }
        } // unlock用
        /*
        *  data calculation about robot
        */
        temp_convert_data = robot_data_cal.convert_robotdata(robotdata.master_data, robotdata.copy_data, robotdata.partner_master_data);    //Get MasterRobot's Position for manual path trajectory
        //robotdata.master_data = robot_data_cal.MRobot_Linear_PositionCal(robotdata.master_data, (micro_current_clock - first_clock).count());
        robotdata.master_data = std::get<0>(temp_convert_data); robotdata.copy_data = std::get<1>(temp_convert_data); robotdata.partner_master_data = std::get<2>(temp_convert_data);
        robotdata.err_data = robot_data_cal.err_robotposition_cal(robotdata.master_data, robotdata.copy_data);
        
        /*
        *  force getting
        */
        //robotdata.force_actual_data = force_actual.FEActCal(robotdata.copy_data);
        //robotdata.force_virtual_data  = force_ideal.FEVirCal(robotdata.partner_master_data, robotdata.copy_data);
        //robotdata.force_ideal_data = force_ideal.FIdCal(robotdata.partner_master_data, robotdata.master_data);

        /*
        * robot control
        */
        mat3x1.velocity_data  = robot_control.CLPositionControllerPIDCal(robotdata.err_data, robotdata.last_err_data, micro_dt.count());        // PID control
        bool result = robot_control.VelocityLimitationCal(mat3x1.velocity_data);   // limit
        if (! result) {
            safety_count++;
            if (safety_count > 15){
                std::cout << "強制終了" << std::endl;
                stop_flag = true;
            }
            //robot_control.chenge_pid(6.5, 0.02, 2.5); // PIDを変更
        }
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
        //motor_control.EnableMotorDrive(mat3x1.mortor_voltage);
        bool motror_result = motor_control.send_voltage(mat3x1.mortor_voltage[0], mat3x1.mortor_voltage[1], mat3x1.mortor_voltage[2]);
        if (!motror_result) {
            std::cout << "Voltage out of range. Motor control failed." << std::endl;
            stop_flag = true; // Stop the program if voltage is out of range
        }
        robotdata.last_err_data = robotdata.err_data;

        /* 
        * debug用保存および表示
        */
        current_clock = std::chrono::high_resolution_clock::now();
        nano_receive_clock = std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch());
        double delay_time = (nano_receive_clock.count() - send_time)/ 1000000.0;
        data_logger.save_csv(robotdata, mat3x1, send_time, nano_receive_clock.count(),delay_time);
        data_logger.show_data(robotdata, mat3x1.velocity_data, micro_dt.count(), delay_time);
        data_logger.send_monitor(robotdata, delay_time, nano_receive_clock.count());
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
    set_cpu_governor("ondemand");
    udp_thread_master.join(); // UDP receive thread from master
    udp_thread_copy.join(); // UDP receive thread from copy
    std::cout << "[INFO] Program terminated gracefully." << std::endl;
    std::cout << "[Warning] Not get data count: Master = " << not_get_count[0] << ", Copy = " << not_get_count[1] << std::endl;
    return 0;
    
}
