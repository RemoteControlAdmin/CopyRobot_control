#ifndef FORCE_GET_HPP
#define FORCE_GET_HPP

#include <common.hpp>
// 必要なライブラリのインクルード

namespace forceget { // 名前空間 (任意の名前，ソースと合わせること)

    class ForceActual { // クラスの宣言（任意の名前）

    public: // 以下public関数と変数の宣言

    ForceActual(); // コンストラクタの宣言

    std::vector<int> ReadAnalogInputs(); // 関数の宣言
    
    std::vector<double> FEActCal(); // 力環境の実際の値を計算する関数の宣言

    //double FEActSwapCal(); // 力環境の実際の値をロボットフレームからグローバルフレームに変換する関数の宣言
    
    private: // 以下private関数と変数の宣言
    //===========Declaration the force sensor communication===========
    int ADC_AI0,ADC_AI1,ADC_AI2,ADC_AI3; // アナログ入力のインデックスを格納する変数の宣言
    double Volt0, Volt1, Volt2, Volt3; // アナログ入力の電圧値を格納する変数の宣言
    double FAx, FAy, FBx, FBy; // 力センサーの値を格納する変数の宣言
    double fFAx, fFAy, fFBx, fFBy; // フィルタ後の力センサーの値を格納する変数の宣言
    double FEactM, FEactMa, FEactx, FEacty, FEactNull; // 力環境の実際の値を格納する変数の宣言
    double AngleA_off = -(M_PI/6), AngleB_off = (M_PI*5/6); // 力センサーのオフセット角度を格納する変数の宣言
    double FEactMThreshold = 1.0; // 力環境の実際の値のしきい値を格納する変数の宣言
    double FId[16] = {0.0,0.0,0.0,0.0,0.0};	
    double FAx_R,FAy_R,FA_R,FAa_R,FAx_G,FAy_G,FA_G,FAa_G;   // 力センサーのロボットフレームとグローバルフレームの値を格納する変数の宣言
    double FBx_R,FBy_R,FB_R,FBa_R,FBx_G,FBy_G,FB_G,FBa_G;   // 力センサーのロボットフレームとグローバルフレームの値を格納する変数の宣言
    double F_R,Fa_R,F_G,Fa_G; // 力センサーのロボットフレームとグローバルフレームの値を格納する変数の宣言
    double FEactMSat = 12.0*sqrt(2);   // 力環境の実際の値の最大値を格納する変数の宣言
    //===========Declaration the force filter variable===========
    double a=0.9391, b=0.0609; // Fc 10Hz, Ts 0.01 Sec

    RobotData robotdata;
    }; 

    class ForceIdeal{
    public: // 以下public関数と変数の宣言

    ForceIdeal(); // コンストラクタの宣言
    std::pair<std::vector<double>,bool> FIdCal(std::vector<double> partner_master_data, std::vector<double> copy_data); // 力理想値を計算する関数の宣言
    std::pair<std::vector<double>,bool> FEVirCal(std::vector<double> partner_master_data, std::vector<double> master_data); // 力環境の仮想値を計算する関数の宣言
    //double ForceCompensateCal(); // 力補償を計算する関数の宣言



    private: // 以下private関数と変数の宣言
    double Kspring = 16.000/0.020; // N/m (800 N/m)
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