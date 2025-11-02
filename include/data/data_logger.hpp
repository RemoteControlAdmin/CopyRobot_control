#pragma once 

#include "data/csv_edit.hpp"
#include "net/udp_connect.hpp"
#include "common.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>

namespace data_lib{
class DataLogger{
    /*
    debug用に表示，およびcsvにデータを書き込むクラス
    */
    private:
        std::string get_my_name();
        void initialize_csv();
        
        Csvedit csvWriter;
        std::pair<std::vector<int64_t>,std::vector<double>>  csv_data;
        net_lib::UdpConnect udpConnection_monitor;


        // 保存用データ
        std::vector<double> csv_vector;
        // udp用データ
        std::vector<double> udp_vector;

        // データをサーバに保存する関数
        void move_data();

    public:
        DataLogger(int monitor_port);
        
        void show_data(RobotData robotdata, Eigen::Vector3d velocity_data, int dt, double delay_time, double force_delay_time);
        void save_csv(RobotData robotdata, Mat3x1 mat3x1, int64_t send_clock, int64_t receive_clock, double delay_time, double force_delay_time, std::vector<double> energy);
        void send_monitor(RobotData robotdata, double delay_time, int64_t receive_clock);
        ~DataLogger();
};
}