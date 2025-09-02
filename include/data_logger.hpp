#pragma once 

#include "csv_edit.hpp"
#include "common.hpp"
#include <iostream>
#include <iomanip>

class DataLogger{
    /*
    debug用に表示，およびcsvにデータを書き込むクラス
    */
    private:
        std::string get_my_name();
        void initialize_csv();

        csv_lib::Csvedit csvWriter;
        std::pair<std::vector<int64_t>,std::vector<double>>  csv_data;

        // 送信用変数
        std::vector<double> csv_vector;

    public:
        DataLogger();
        
        void show_data(RobotData robotdata, Eigen::Vector3d velocity_data, int dt, double delay_time);
        void save_csv(RobotData robotdata, Mat3x1 mat3x1, int64_t send_clock, int64_t receive_clock, double delay_time); 
        
        ~DataLogger();
};