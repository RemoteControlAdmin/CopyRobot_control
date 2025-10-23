# include <thread>
# include <deque>
# include <mutex>
# include <vector>
# include <cmath>
# include <iterator>
# include <iostream>
# include <cstdlib>
# include "common.hpp"
# include "spi_service.hpp"
# include "motor_control.hpp"
# include "robot_data_cal.hpp"
# include "robot_control.hpp"
# include "inverce_kinematics.hpp"
# include "udp_connect.hpp"
# include "force_get.hpp"
# include "data_logger.hpp"
# include "rlsarpmin.hpp"
# include "cycle_timer.hpp"



void end_task(int signum){
    if(signum == SIGINT) {
        std::cout << "\n[Info] Ctrl+C detected. Exiting..." << std::endl;
        stop_flag = 1;
    }
}

void set_cpu_governor(const std::string& governor) {
    // CPUのガバナーを設定する関数
    std::string command = "sudo cpufreq-set -g" + governor;
    int ret = system(command.c_str());
    if (ret != 0) {
        std::cerr << "[Error] setting CPU governor to " << governor << std::endl;
    } else {
        std::cout << "\n[Info] CPU governor set to " << governor << std::endl;
    }
}

double cal_delay_time(int64_t send_time){
    std::chrono::high_resolution_clock::time_point current_clock = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds nano_current_clock = std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch());
    double delay_time = (nano_current_clock.count() - send_time)/ 1000000.0;
    return delay_time;
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
    // masterとcopyのデータを格納するキュー
    std::deque<std::pair<std::vector<double>, int64_t>> deque_master;
    std::deque<std::pair<std::vector<double>, int64_t>> deque_copy;
    std::mutex queue_mutex_master;
    std::mutex queue_mutex_copy;
    // 力センサーの値を格納するキュー
    std::deque<std::vector<double>> deque_force;
    std::mutex queue_mutex_force;
    // 力センサー（UDP）の値を格納するキュー
    std::deque<std::pair<std::vector<double>, int64_t>> deque_udpforce;
    std::mutex queue_mutex_udpforce;

    /*
    * インスタンス作成 Instance creation
    */
    
    robotcontrol::RobotDataCal robot_data_cal{};    // data calculation about robot
    robotcontrol::RobotControl robot_control{};     // robot control
    robotkinematics::InverceKinematics inverce_kinematics{}; // inverce kinematics
    forceget::ForceActual force_actual{ deque_force, queue_mutex_force}; // force actual
    forceget::ForceIdeal force_ideal{}; // force ideal
    udp_lib::UdpCommunicator udp_communicator(deque_master, deque_copy, queue_mutex_master, queue_mutex_copy, deque_udpforce, queue_mutex_udpforce); // UDP communication
    //udp_lib::UdpConnect udpConnection_raspberrypi("192.168.11.202", 65000, 26); // UDP初期化    
    DataLogger data_logger{}; // data logger
    std::array<rlsarpmin::RLSARPMin, 3> rls = {
        rlsarpmin::RLSARPMin(9, 10, 0.99999, 1e3, 1e-9,0),
        rlsarpmin::RLSARPMin(9, 10, 0.99999, 1e3, 1e-9,1),
        rlsarpmin::RLSARPMin(9, 10, 0.99999, 1e3, 1e-9,2),
    };
    /*
    * ローカル変数定義　local variable definition
    */
    int64_t master_send_time = 0; // 送信時間 send time
    int64_t force_send_time = 0; // 送信時間 send time
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
    std::vector<std::optional<double>> predict_master_data(3, std::nullopt);
    // 力センサー（UDP）の値を保持するための変数
    std::vector<double> force_udp_values(4, 0.0);
    // 力制御によるmasterロボットの一次変数
    std::vector<double> force_control_data(3, 0.0); // 力制御によるmasterロボットの一次変数

    // dt計算用 dt calculation
    std::chrono::high_resolution_clock::time_point current_clock; // 現在の時刻　current time
    std::chrono::microseconds T = std::chrono::microseconds(10*1000); // cycle time
    std::chrono::microseconds micro_dt = T; // dt
    int cycle_count = 0; // cycle count
    /*
    * ============== main roop ==============
    */
    cycle_timer::CycleTimer cycle_timer{T};
    int safety_count = 0; // 安全カウント
    std::vector<int> not_get_count = {0,0,0}; // データが取得できなかった回数
    std::cout << "================== start ==================" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 2秒待機 wait for 2 seconds

    /*
    * 受信スレッドの開始 start receive thread
    */
    std::thread udp_thread_master(&udp_lib::UdpCommunicator::recive_thread_from_master, &udp_communicator); // UDP receive thread from master
    std::thread udp_thread_copy(&udp_lib::UdpCommunicator::recive_thread_from_copy, &udp_communicator); // UDP receive thread from master
    std::thread recive_thread_get_forcevalue(&udp_lib::UdpCommunicator::recive_thread_get_forcevalue, udp_communicator);
    /*
    * SPI, Motor setup
    */
    SPIService spi_service{}; // SPI setup
    motorcontrol::MotorControl motor_control{};     // Motor setup
    std::thread force_thread(&forceget::ForceActual::force_get_thread, &force_actual, std::ref(spi_service)); // force get thread
    
    while(!stop_flag){
        cycle_count++;
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
                master_send_time = getdata.second;
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

        {
            std::lock_guard<std::mutex> lock(queue_mutex_udpforce);
            if (!deque_udpforce.empty()){
                force_udp_values = deque_udpforce.front().first;
                force_send_time = deque_udpforce.front().second;
                deque_udpforce.pop_front();
            }
            else{
                force_udp_values = {0.0, 0.0, 0.0, 0.0};
                not_get_count[2]++;
            }
        }
        /*
        *  data calculation about robot
        */
        temp_convert_data = robot_data_cal.convert_robotdata(robotdata.master_data, robotdata.copy_data, robotdata.partner_master_data);    //Get MasterRobot's Position for manual path trajectory
        robotdata.master_data = std::get<0>(temp_convert_data); robotdata.copy_data = std::get<1>(temp_convert_data); robotdata.partner_master_data = std::get<2>(temp_convert_data);
        /*
        * predict master position
        */
        double delay_time = cal_delay_time(master_send_time);
        int k = int(delay_time/10);
        for(int i = 0; i < 3; i++){
            predict_master_data[i] = rls[i](robotdata.master_data[i], k);
            robotdata.master_data[i] = predict_master_data[i].value_or(robotdata.master_data[i]);
        }
        if (cycle_count <= 500){
            motor_control.send_voltage(0, 0, 0, spi_service);
            const auto dt = cycle_timer.tick(); 
            micro_dt = std::chrono::duration_cast<std::chrono::microseconds>(dt);
            std::cout << "\033[2J\033[1;1H"; 
            std::cout << "[Info] RLS initializing... (" << cycle_count << "/500)" << std::endl;
            not_get_count = {0,0,0};
            continue;
        }
        robotdata.err_data = robot_data_cal.err_robotposition_cal(robotdata.master_data, robotdata.copy_data);
        /*
        *  force getting
        */
        robotdata.force_actual_data = force_actual.FEActCal(robotdata.copy_data);
        robotdata.force_virtual_data  = force_ideal.FEVirCal(robotdata.partner_master_data, robotdata.copy_data);
        robotdata.force_ideal_data = force_ideal.FIdCal(robotdata.partner_master_data, robotdata.master_data);
        robotdata.force_udp_data = force_actual.FUDPCal(force_udp_values);
        
        std::cout << robotdata.err_data[0] << ", " << robotdata.err_data[1] << ", " << robotdata.err_data[2] << std::endl;
        /*
        * force control
        */
        double force_delay_time = cal_delay_time(force_send_time);
        force_control_data = robot_control.bilateral_force_control(robotdata.force_actual_data, robotdata.force_virtual_data, 
            robotdata.force_udp_data, robotdata.master_data, micro_dt.count()); 
        robotdata.err_data = robot_data_cal.err_robotposition_cal(force_control_data, robotdata.copy_data);

        std::cout << robotdata.err_data[0] << ", " << robotdata.err_data[1] << ", " << robotdata.err_data[2] << std::endl;
        
        /*
        * robot control
        */
        mat3x1.velocity_data  = robot_control.CLPositionControllerPIDCal(robotdata.err_data, robotdata.last_err_data, micro_dt.count());        // PID control
        bool result = robot_control.VelocityLimitationCal(mat3x1.velocity_data);   // limit
        if (! result) {
            safety_count++;
            if (safety_count > 45){
                std::cout << "[Warning] Safety count exceeded" << std::endl;
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
        motor_control.send_voltage(mat3x1.mortor_voltage[0], mat3x1.mortor_voltage[1], mat3x1.mortor_voltage[2], spi_service);
        robotdata.last_err_data = robotdata.err_data;

        /* 
        * debug用保存および表示
        */
        
        current_clock = std::chrono::high_resolution_clock::now();// 現在時刻を取得
        nano_receive_clock = std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch());// ns（ナノ秒）単位で取得
        data_logger.save_csv(robotdata, mat3x1, master_send_time, nano_receive_clock.count(),delay_time, force_delay_time, predict_master_data, robotdata.force_udp_data);
        data_logger.show_data(robotdata, mat3x1.velocity_data, micro_dt.count(), delay_time, force_delay_time);
        data_logger.send_monitor(robotdata, delay_time, nano_receive_clock.count());
        /*
        *  adjusting the cycle
        */
        const auto dt = cycle_timer.tick(); 
        micro_dt = std::chrono::duration_cast<std::chrono::microseconds>(dt);
        //std::cout << "dt = " << micro_dt.count() << std::endl;
    }
    // 終了前の後始末
    set_cpu_governor("ondemand");
    udp_thread_master.join(); // UDP receive thread from master
    udp_thread_copy.join(); // UDP receive thread from copy
    force_thread.join(); // force get thread
    recive_thread_get_forcevalue.join();
    std::cout << "[Info] Program terminated gracefully." << std::endl;
    std::cout << "[Warning] Not get data count: Master = " << not_get_count[0] << ", Copy = " << not_get_count[1] << ", Force = " << not_get_count[2] << std::endl;
    return 0;
    
}
