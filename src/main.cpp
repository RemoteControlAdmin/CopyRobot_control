# include <vector>
# include <cmath>
# include <iterator>
# include <iostream>
# include <cstdlib>
# include "common.hpp"
# include "spi/spi_service.hpp"
# include "robot/motor_control.hpp"
# include "robot/robot_data_cal.hpp"
# include "robot/robot_control.hpp"
# include "robot/inverce_kinematics.hpp"
# include "robot/force_cal.hpp"
# include "data/data_logger.hpp"
# include "utils/cycle_timer.hpp"
# include "utils/deque_manager.hpp"
# include "utils/cpu_manager.hpp"
# include "utils/delaytime_cal.hpp"
# include "utils/commandline.hpp"
# include "stability/energy_cal.hpp"
# include "rlsarpmin.hpp"

void end_task(int signum){
    if(signum == SIGINT) {
        std::cout << "\n[Info] Ctrl+C detected. Exiting..." << std::endl;
        stop_flag = 1;
    }
}

int main(int argc, char* argv[]){
    
    auto [target_copyrobot_ip, monitor_port] = parse_command_line(argc, argv);

    set_cpu_governor("performance");
    std::cout << "================== runnning ==================" << std::endl;
    // 強制終了処理　end proccess
    std::signal(SIGINT, end_task);


    /*
    * インスタンス作成 Instance creation
    */
    robot_lib::RobotDataCal robot_data_cal{};    // data calculation about robot
    robot_lib::RobotControl robot_control{};     // robot control
    robot_lib::InverceKinematics inverce_kinematics{}; // inverce kinematics
    robot_lib::ForceCal force_actual{}; // force actual
    robot_lib::ForceIdeal force_ideal{}; // force ideal
    //net_lib::UdpConnect udpConnection_raspberrypi("192.168.11.202", 65000, 26); // UDP初期化
    data_lib::DataLogger data_logger{monitor_port}; // data logger
    utils::DequeManager deque_manager{target_copyrobot_ip}; // deque manager
    stability_lib::EnergyCal energy_cal{}; // energy calculation
    std::array<rlsarpmin::RLSARPMin, 3> rls = {
        rlsarpmin::RLSARPMin(9, 10, 0.9999, 1e3, 1e-9,0),
        rlsarpmin::RLSARPMin(9, 10, 0.9999, 1e3, 1e-9,1),
        rlsarpmin::RLSARPMin(9, 10, 0.9999, 1e3, 1e-9,2),
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
    std::tuple <std::vector<double>, std::vector<double>, std::vector<double>,std::vector<double>> temp_convert_data;
    
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
    utils::CycleTimer cycle_timer{T};
    int safety_count = 0; // 安全カウント
    std::cout << "==================  start   ==================" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 2秒待機 wait for 2 seconds
    deque_manager.udp_thread_manager(); // start udp receive thread
    /*
    * SPI, Motor setup
    */
    spi_lib::SPIService spi_service{}; // SPI setup
    robot_lib::MotorControl motor_control{};     // Motor setup
    deque_manager.force_thread_manager(spi_service); // start force get thread
    /*
    * もし，10ms以内にデータが取得できなかったら, Picoが強制終了する
    * If data cannot be acquired within 10 ms, force termination
    */
    while(!stop_flag){
        cycle_count++;
        /*
        * queue取り出し
        * get queue
        */
        auto master_data = deque_manager.get_master_data();
        robotdata.master_data = std::get<0>(master_data);
        robotdata.remote_copy_data = std::get<1>(master_data);
        master_send_time = std::get<2>(master_data);
        auto copy_data = deque_manager.get_copy_data();
        robotdata.copy_data = copy_data.first;
        robotdata.partner_master_data = copy_data.second;
        auto force_data = deque_manager.get_udpforce_data();
        force_udp_values = force_data.first;
        force_send_time = force_data.second;
        /*
        *  data calculation about robot
        */
        double delay_time = cal_delay_time(master_send_time);
        temp_convert_data = robot_data_cal.convert_robotdata(robotdata.master_data, robotdata.copy_data, robotdata.partner_master_data, robotdata.remote_copy_data);    //Get MasterRobot's Position for manual path trajectory
        //robotdata.master_data = robot_data_cal.MRobot_Linear_PositionCal(robotdata.master_data, (micro_current_clock - first_clock).count());
        robotdata.master_data = std::get<0>(temp_convert_data); robotdata.copy_data = std::get<1>(temp_convert_data); 
        robotdata.partner_master_data = std::get<2>(temp_convert_data); robotdata.remote_copy_data = std::get<3>(temp_convert_data);
        robotdata.err_data = robot_data_cal.err_robotposition_cal(robotdata.master_data, robotdata.copy_data);
        int k = int((delay_time)/10);
        std::cout << "k = " << k << std::endl;
        for(int i = 0; i < 3; i++){
            predict_master_data[i] = rls[i](robotdata.master_data[i], k);
            //robotdata.master_data[i] = predict_master_data[i].value_or(robotdata.master_data[i]);
        }
        if (cycle_count <= 500){
            motor_control.send_voltage(0, 0, 0, spi_service);
            const auto dt = cycle_timer.tick(); 
            micro_dt = std::chrono::duration_cast<std::chrono::microseconds>(dt);
            std::cout << "\033[2J\033[1;1H"; 
            std::cout << "[Info] Waiting... (" << cycle_count << "/500)" << std::endl;
            deque_manager.rest_not_get_coount(); // force data clear
            continue;
        }
        
        /*
        *  force getting
        */
        robotdata.force_actual_data = force_actual.FEActCal(robotdata.copy_data, deque_manager.get_actforce_data());
        robotdata.force_virtual_data  = force_ideal.FEVirCal(robotdata.partner_master_data, robotdata.copy_data);
        robotdata.force_ideal_data = force_ideal.FIdCal(robotdata.partner_master_data, robotdata.master_data);
        robotdata.force_udp_data = force_actual.FUDPCal(force_udp_values, robotdata.remote_copy_data);
        
        std::vector<double> energy = energy_cal.sum_energy_cal(robotdata, master_send_time);
        
        /*
        * force control
        */
        double force_delay_time = cal_delay_time(force_send_time);
        force_control_data = robot_control.bilateral_force_control(robotdata.force_actual_data, robotdata.force_virtual_data, 
            robotdata.force_udp_data, robotdata.master_data, micro_dt.count()); 
        //robotdata.err_data = robot_data_cal.err_robotposition_cal(force_control_data, robotdata.copy_data);
        
        /*
        * robot control
        */
        mat3x1.velocity_data  = robot_control.CLPositionControllerPIDCal(robotdata.err_data, robotdata.last_err_data, micro_dt.count());        // PID control
        bool result = robot_control.VelocityLimitationCal(mat3x1.velocity_data);   // limit
        if (! result) {
            safety_count++;
            if (safety_count > 15){
                std::cout << "[Warning] Safety count exceeded" << std::endl;
                stop_flag = true;
            }
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
        * 実験データ取得時はshow_dataはコメントアウト推奨
        * When acquiring experimental data, it is recommended to comment out show_data
        */
        current_clock = std::chrono::high_resolution_clock::now();// 現在時刻を取得
        nano_receive_clock = std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch());// ns（ナノ秒）単位で取得
        data_logger.save_csv(robotdata, mat3x1, master_send_time, nano_receive_clock.count(),delay_time, force_delay_time, energy);
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
    std::cout << "[Info] Stopping threads..." << std::endl;
    std::cout << "==================   end    ==================" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 2秒待機 wait for 2 seconds
    std::cout << "[Info] Cycle count: " << cycle_count << std::endl;
    std::cout << "[Info] Program terminated gracefully." << std::endl;
    return 0;
    
}
