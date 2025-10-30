#pragma once

# include "net/udp_connect.hpp"
# include "spi/spi_service.hpp"
# include "robot/force_get.hpp"
# include <deque>
# include <mutex>
# include <thread>

namespace utils { 

    class DequeManager {

    public: // 以下public関数と変数の宣言

    DequeManager(std::string target_copyrobot_ip); // コンストラクタ
    ~DequeManager(); // デストラクタ
    
    void udp_thread_manager();
    void force_thread_manager(spi_lib::SPIService& spi_service);

    void rest_not_get_coount();

    std::pair<std::vector<double>, int64_t> get_master_data();
    std::pair<std::vector<double>, std::vector<double>> get_copy_data();
    std::pair<std::vector<double>, int64_t> get_udpforce_data();
    std::vector<double> get_actforce_data();

    private: // 以下private関数と変数の宣言

    net_lib::UdpCommunicator udp_communicator; 
    std::thread udp_thread_master;
    robot_lib::ForceGet force_get;
    std::thread udp_thread_copy;
    std::thread force_thread;
    std::thread recive_thread_get_forcevalue;

    std::vector<int> not_get_count = {0,0,0}; // データが取得できなかった回数

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
    
    std::vector<double> master_data;
    int64_t master_send_time;
    std::vector<double> master_last_data;

    std::vector<double> copy_data;
    std::vector<double> copy_last_data;
    std::vector<double> temp_copy_data;
    std::vector<double> partner_master_data;
    std::vector<double> partner_master_last_data;

    std::vector<double> force_udp_values;
    int64_t force_send_time = 0;

    std::vector<double> force_values;
    
    }; 
} // namespace utils