#ifndef FORCE_GET_HPP
#define FORCE_GET_HPP

#include <chrono>
#include <mutex>
#include <deque>
#include <atomic>
#include <iostream>
#include <thread>

#include "spi/spi_service.hpp"
#include "common.hpp"
#include "net/udp_connect.hpp"

// 必要なライブラリのインクルード

namespace robot_lib { // 名前空間 (任意の名前，ソースと合わせること)

    class ForceGet { // クラスの宣言（任意の名前）

    public: // 以下public関数と変数の宣言

    ForceGet( 
        std::deque<std::vector<double>>& deque_force, std::mutex& queue_mutex_force, std::string target_copyrobot_ip); // コンストラクタの宣言

    void force_get_thread(spi_lib::SPIService& spi_service); // 関数の宣言

    int force_freq = 1000;
    //===========Declaration the force filter parameter===========
    double iir_a[3], iir_b[3]; // IIR low pass
    double notch_a[3], notch_b[3]; // IIR notch
    double lowpass_fc = 8.0; // IIR low pass cut off frequency
    double notch_fc = 17.5; // IIR notch cut off frequency
    double bw = 0.3; // IIR notch bandwidth
    double Q = 0.3; // IIR notch Q factor
    double force_iir_x[4][3] = {0}, force_iir_y[4][3] = {0}; // IIR low pass
    double force_notch_x[4][3] = {0}, force_notch_y[4][3] = {0}; // IIR notch

    void lowpass_param_set();
    std::vector<double> filter_iirlowpass(std::vector<double> force_values);
    void notch_param_set();
    std::vector<double> filter_iirnotch(std::vector<double> force_values);
    
    private: // 以下private関数と変数の宣言
    std::deque<std::vector<double>>& deque_force_;
    std::mutex& queue_mutex_force_;
   
    //  ===========UDP通信処理===========
    net_lib::UdpConnect udpConnection_send_forcevalues;


    //===========Declaration the force sensor communication===========
    RobotData robotdata;
    }; 


} // namespace 

#endif // FORCE_GET_HPP