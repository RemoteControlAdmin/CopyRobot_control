# include "robot/force_cal.hpp"
#include <fstream>  // ← 追加
#include <iomanip>  // ← setprecision で必要

namespace robot_lib { 
    ForceCal::ForceCal()
        {

    } //コンストラクタ
    
    std::vector<double> ForceCal::FEActCal(std::vector<double> copy_data, std::vector<double> force_values){
        
        FAx = force_values[0];
        FAy = force_values[1];
        FBx = force_values[2];
        FBy = force_values[3];
        //Result Force Robot Frame of Sensor A
        FA_R  = sqrt(pow(FAx,2)+pow(FAy,2));
        FAa_R = (atan2(FAy,FAx));
        FAx_R = FA_R*cos(FAa_R);
        FAy_R = FA_R*sin(FAa_R);
        //Result Force Global Frame of Sensor A
        FAx_G = FAx_R*cos(AngleA_off)+FAy_R*cos(AngleA_off+(M_PI/2));
        FAy_G = FAx_R*sin(AngleA_off)+FAy_R*sin(AngleA_off+(M_PI/2));    
        FA_G  = sqrt(pow(FAx_G,2)+pow(FAy_G,2));
        FAa_G = (atan2(FAy_G,FAx_G));
        //Result Force Robot Frame of Sensor B
        FB_R =sqrt(pow(FBx,2)+pow(FBy,2));
        FBa_R=(atan2(FBy,FBx));
        FBx_R = FB_R*cos(FBa_R);
        FBy_R = FB_R*sin(FBa_R);
        //Result Force Global Frame of Sensor B
        FBx_G = FBx_R*cos(AngleB_off)+FBy_R*cos(AngleB_off+(M_PI/2));
        FBy_G = FBx_R*sin(AngleB_off)+FBy_R*sin(AngleB_off+(M_PI/2));    
        FB_G  = sqrt(pow(FBx_G,2)+pow(FBy_G,2));
        FBa_G = (atan2(FBy_G,FBx_G));
        //Result Force Robot Frame of MR
        F_R =sqrt(pow(FAx_R+FBx_R,2)+pow(FAy_R+FBy_R,2));
        Fa_R=(atan2(FAy_R+FBy_R,FAx_R+FBx_R));
        //if(Fa_R<0) Fa_R=Fa_R+(2*M_PI);
        //Result Force Global Frame of MR
        F_G =sqrt(pow(FAx_G+FBx_G,2)+pow(FAy_G+FBy_G,2));
        Fa_G=(atan2(FAy_G+FBy_G,FAx_G+FBx_G));
        //if(Fa_G<0) Fa_G=Fa_G+(2*M_PI);
        /*if(F_G<FEactMThreshold){                                // Force actual threshold determining
        FEactM  =0;
        FEactMa =0;
        FEactx =0;
        FEacty =0;
        FEactNull =0;}
        else{*/
        FEactM  =F_G;
        FEactMa =Fa_G;
        FEactx =F_G*cos(Fa_G);
        FEacty =F_G*sin(Fa_G);//}
        FEactNull =0;
        // convert global frame
        FEactMa = FEactMa + copy_data[2]; // copy_data[2] is the angle offset for the global frame
        return {FEactM,FEactMa, FA_G, FB_G}; // Absolute Force, Angle of Force, Force A, Force B 
    }

    std::vector<double> ForceCal::FUDPCal(std::vector<double> force_values, std::vector<double> remote_copy_data){
        // force_values = [FAx, FAy, FBx, FBy]
        const double ax_local = force_values[0];
        const double ay_local = force_values[1];
        const double bx_local = force_values[2];
        const double by_local = force_values[3];

        // Sensor A ロボット→グローバル
        const double a_mag = std::sqrt(ax_local * ax_local + ay_local * ay_local);
        const double a_ang = std::atan2(ay_local, ax_local);
        const double ax_r = a_mag * std::cos(a_ang);
        const double ay_r = a_mag * std::sin(a_ang);
        const double ax_g = ax_r * std::cos(AngleA_off) + ay_r * std::cos(AngleA_off + M_PI / 2.0);
        const double ay_g = ax_r * std::sin(AngleA_off) + ay_r * std::sin(AngleA_off + M_PI / 2.0);

        // Sensor B ロボット→グローバル
        const double b_mag = std::sqrt(bx_local * bx_local + by_local * by_local);
        const double b_ang = std::atan2(by_local, bx_local);
        const double bx_r = b_mag * std::cos(b_ang);
        const double by_r = b_mag * std::sin(b_ang);
        const double bx_g = bx_r * std::cos(AngleB_off) + by_r * std::cos(AngleB_off + M_PI / 2.0);
        const double by_g = bx_r * std::sin(AngleB_off) + by_r * std::sin(AngleB_off + M_PI / 2.0);

        // グローバル合力のマグニチュード
        const double fx_total = ax_g + bx_g;
        const double fy_total = ay_g + by_g;
        //if (std::sqrt(fx_total * fx_total + fy_total * fy_total) < 1.0){
        //    return 0;
        //}

        return {std::sqrt(fx_total * fx_total + fy_total * fy_total), std::atan2(fy_total, fx_total)+ remote_copy_data[2],
                std::sqrt(ax_g * ay_g), std::sqrt(bx_g * by_g)};
    }


    /*
    * class ForceIdeal
    */
    ForceIdeal::ForceIdeal(){} //コンストラクタ

    //Virtual Force Environment calculate [FEVir]=[Ks]*[0.365-([PM1]-[PC1])]
    std::vector<double> ForceIdeal::FEVirCal(std::vector<double> partner_master_data, std::vector<double> copy_data){
        DVir   = sqrt(pow(partner_master_data[0] - copy_data[0],2.0)+pow(partner_master_data[1]-copy_data[1],2.0));                       //Displcement Virtual
        A_DVir = atan2((partner_master_data[1]-copy_data[1]),(partner_master_data[0]-copy_data[0]));                                    //Angle of Displcement Virtual
        if		(DVir>=robot_d) 				{FEVir = 0;                          CRTouchChk=0;}
        //else if	(DVir> robot_d&&DVir<robot_d)	{FEVir = Kspring*(0.365-DVir);       CRTouchChk=1;}
        else								{FEVir = Kspring*(robot_d - DVir);}
        
        return {FEVir, A_DVir, DVir,FEVir*cos(A_DVir), FEVir*sin(A_DVir), static_cast<double>(CRTouchChk)};
    }
    //Force Ideal calculate [FId]=[Ks]*[0.365-([PM1]-[PM2])]
    std::vector<double> ForceIdeal::FIdCal(std::vector<double> partner_master_data, std::vector<double> master_data){
        DIde   = sqrt(pow(partner_master_data[0]-master_data[0],2.0)+pow(partner_master_data[1]-master_data[1],2.0));                       //Displcement Ideal
        A_DIde = atan2((partner_master_data[1]-master_data[1]),(partner_master_data[0]-master_data[0]));                                    //Angle Virtual
        if		(DIde>=ideal_robot_d)               {FIdeal = 0;                          MRTouchChk=0;}
        //else if	(DIde> 0.340&&DIde<0.365)	{FIdeal = Kspring*(0.365-DIde);        MRTouchChk=1;}
        else								{FIdeal = Kspring*(ideal_robot_d-DIde);}

        return {FIdeal, A_DIde, DIde, FIdeal*cos(A_DIde), FIdeal*sin(A_DIde), static_cast<double>(MRTouchChk)};
    }

    
}
