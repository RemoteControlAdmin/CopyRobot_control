#ifndef FORCE_GET_HPP
#define FORCE_GET_HPP

#include <chrono>
#include <mutex>
#include <deque>
#include <atomic>
#include <iostream>
#include <thread>
#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"
#include "rp1-spi-util.h"
#include "pi_pico_commands.h"
#include "rpi5-rp1-spi.h"
#include "spi_service.hpp"
#include "common.hpp"

// 必要なライブラリのインクルード

namespace forceget { // 名前空間 (任意の名前，ソースと合わせること)

    class ForceActual { // クラスの宣言（任意の名前）

    public: // 以下public関数と変数の宣言

    ForceActual( 
        std::deque<std::vector<double>>& deque_force, std::mutex& queue_mutex_force); // コンストラクタの宣言

    void force_get_thread(SPIService& spi_service); // 関数の宣言

    std::vector<double> FEActCal(std::vector<double> copy_data); // 力環境の実際の値を計算する関数の宣言

    //double FEActSwapCal(); // 力環境の実際の値をロボットフレームからグローバルフレームに変換する関数の宣言
    
    private: // 以下private関数と変数の宣言
    std::deque<std::vector<double>>& deque_force_;
    std::mutex& queue_mutex_force_;
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

    //===========Declaration the force sensor communication===========
    double FAx, FAy, FBx, FBy; // 力センサーの値を格納する変数の宣言
    double FEactM, FEactMa, FEactx, FEacty, FEactNull; // 力環境の実際の値を格納する変数の宣言
    double AngleA_off = -(M_PI/6), AngleB_off = (M_PI*5/6); // 力センサーのオフセット角度を格納する変数の宣言
    double FEactMThreshold = 1.0; // 力環境の実際の値のしきい値を格納する変数の宣言
    double FId[16] = {0.0,0.0,0.0,0.0,0.0};	
    double FAx_R,FAy_R,FA_R,FAa_R,FAx_G,FAy_G,FA_G,FAa_G;   // 力センサーのロボットフレームとグローバルフレームの値を格納する変数の宣言
    double FBx_R,FBy_R,FB_R,FBa_R,FBx_G,FBy_G,FB_G,FBa_G;   // 力センサーのロボットフレームとグローバルフレームの値を格納する変数の宣言
    double F_R,Fa_R,F_G,Fa_G; // 力センサーのロボットフレームとグローバルフレームの値を格納する変数の宣言
    double FEactMSat = 12.0*sqrt(2);   // 力環境の実際の値の最大値を格納する変数の宣言

    RobotData robotdata;
    }; 

    class ForceIdeal{
    public: // 以下public関数と変数の宣言

    ForceIdeal(); // コンストラクタの宣言
    std::vector<double> FIdCal(std::vector<double> partner_master_data, std::vector<double> copy_data); // 力理想値を計算する関数の宣言
    std::vector<double> FEVirCal(std::vector<double> partner_master_data, std::vector<double> master_data); // 力環境の仮想値を計算する関数の宣言
    //double ForceCompensateCal(); // 力補償を計算する関数の宣言



    private: // 以下private関数と変数の宣言
    double Kspring = 327.096700; // N/m (800 N/m)
    double robot_d = 0.357818;
    double ideal_robot_d = 0.355962;
    double DIde,A_DIde,FIdeal,A_FIdeal;
    double DVir,A_DVir,FEVir,A_FEVir;
    int MRTouchChk,CRTouchChk;
    double DVirMin=0.000, DVirMax=0.025;
    double FEv[16] = {0.0,0.0,0.0,0.0,0.0};						//Variable ideal force
    double Pdf[16] = {0,0,0};
    double Po[16]  = {0,0,0};
    double Pod = +0.000;                                        //Displacement compensataion desired in m. unit (- sign is decrese the gap (Dvir), + sign is increase the gap (Dvir)) Default = 0.007 meters
    int CompenChk;

    RobotData robotdata;
    };
} // namespace 

#endif // FORCE_GET_HPP