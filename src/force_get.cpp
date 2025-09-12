# include "force_get.hpp"
#include <fstream>  // ← 追加
#include <iomanip>  // ← setprecision で必要
namespace forceget { 
    ForceActual::ForceActual( 
        std::deque<std::vector<double>>& deque_force, std::mutex& queue_mutex_force):
        deque_force_(deque_force), queue_mutex_force_(queue_mutex_force)
        {
        
    } //コンストラクタ

    void ForceActual::force_get_thread(SPIService& spi_service){ // 関数（任意の名前）
        spi_service.init_adc();
        using clock = std::chrono::steady_clock;
        const auto T = std::chrono::microseconds(force_freq);  // 1ms周期

        auto next = clock::now();
        auto last = next;

        std::ofstream csv_file("force_data.csv");
        csv_file << "time,force0,force1,force2,force3\n";

        while(!stop_flag){
            std::vector<double> force_values = spi_service.read_adc();

            {
                std::lock_guard<std::mutex> lock(queue_mutex_force_);
                if (!deque_force_.empty()){
                    deque_force_.pop_front();
                }
                deque_force_.push_back(force_values);
            }
            std::chrono::high_resolution_clock::time_point current_clock = std::chrono::high_resolution_clock::now();
            csv_file << std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch()).count() << ","
                     << force_values[0] << ","
                     << force_values[1] << ","
                     << force_values[2] << ","
                     << force_values[3] << "\n";
            auto now = clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last);
            last = now;
            next += T;
            std::this_thread::sleep_until(next);
        }
        csv_file.close();
    }
    
    std::vector<double> ForceActual::FEActCal(std::vector<double> copy_data){
        std::vector<double> force_values;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_force_);
            if (!deque_force_.empty()){
                force_values = deque_force_.front();
                deque_force_.pop_front();
            }
            else{
                force_values = {0.0, 0.0, 0.0, 0.0};
            }
        }
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
        if(F_G<FEactMThreshold){                                // Force actual threshold determining
        FEactM  =0;
        FEactMa =0;
        FEactx =0;
        FEacty =0;
        FEactNull =0;}
        else{
        FEactM  =F_G;
        FEactMa =Fa_G;
        FEactx =F_G*cos(Fa_G);
        FEacty =F_G*sin(Fa_G);}
        FEactNull =0;
        // convert global frame
        FEactMa = FEactMa + copy_data[2]; // copy_data[2] is the angle offset for the global frame
        return {FEactM,FEactMa};
    }


    /*
    * class ForceIdeal
    */
    ForceIdeal::ForceIdeal(){} //コンストラクタ

    //Virtual Force Environment calculate [FEVir]=[Ks]*[0.365-([PM1]-[PC1])]
    std::vector<double> ForceIdeal::FEVirCal(std::vector<double> partner_master_data, std::vector<double> copy_data){
        DVir   = sqrt(pow(partner_master_data[0] - copy_data[0],2.0)+pow(partner_master_data[1]-copy_data[1],2.0));                       //Displcement Virtual
        A_DVir = atan2((partner_master_data[1]-copy_data[1]),(partner_master_data[0]-copy_data[0]));                                    //Angle of Displcement Virtual
        if		(DVir>=0.365) 				{FEVir = 0;                          CRTouchChk=0;}
        else if	(DVir> 0.340&&DVir<0.365)	{FEVir = Kspring*(0.365-DVir);       CRTouchChk=1;}
        else								{FEVir = Kspring*(0.365-DVir);  DVir=0.025;              }

        return {FEVir*cos(A_DVir), FEVir*sin(A_DVir), DVir, FEVir, A_DVir, static_cast<double>(CRTouchChk)};
    }
    //Force Ideal calculate [FId]=[Ks]*[0.365-([PM1]-[PM2])]
    std::vector<double> ForceIdeal::FIdCal(std::vector<double> partner_master_data, std::vector<double> master_data){
        DIde   = sqrt(pow(partner_master_data[0]-master_data[0],2.0)+pow(partner_master_data[1]-master_data[1],2.0));                       //Displcement Ideal
        A_DIde = atan2((partner_master_data[1]-master_data[1]),(partner_master_data[0]-master_data[0]));                                    //Angle Virtual
        if		(DIde>=0.365)               {FIdeal = 0;                          MRTouchChk=0;}
        else if	(DIde> 0.340&&DIde<0.365)	{FIdeal = Kspring*(0.365-DIde);        MRTouchChk=1;}
        else								{FIdeal = Kspring*(0.365-DIde); DIde=0.025;}         
            
        return {FIdeal*cos(A_DIde), FIdeal*sin(A_DIde), DIde, FIdeal, A_DIde, static_cast<double>(MRTouchChk)};
    }

    
}
