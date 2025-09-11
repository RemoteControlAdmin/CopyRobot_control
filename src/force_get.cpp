# include "force_get.hpp"

namespace forceget { 
    ForceActual::ForceActual(){
        if (adc2_init(spi2, &ctx,
                      /*do_zero_calib=*/1,
                      ZERO_SAMPLES_DEFAULT,
                      /*cs_bit=*/0,
                      ADC_MAP_MODE_DEFAULT) != 0) {
            fprintf(stderr, "adc2_init failed\n");
        }
    }

    } //コンストラクタ

    std::vector<int> ForceActual::ReadAnalogInputs(){ // 関数（任意の名前）

    }
    
    std::vector<double> ForceActual::FEActCal(std::vector<double> copy_data){
        if (adc2_read_once(spi2, &ctx, &rd) != 0){
            std::cerr << "adc2_read_once failed" << std::endl;
            return {};
        }
        
        FAx = rd.FAx;
        FAy = rd.FAy;
        FBx = rd.FBx;
        FBy = rd.FBy;
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

    //Replace a angle of actual force from the robot frame to the global frame [FEa]=FEa(A_MRDisplacement)
    /*double ForceActual::FEActSwapCal(){
        if(FEactM<=FEactMThreshold){A_FEVir=0;}     // Force Env. Actual less than 0, Angle of Force Env. Virtual is 0
        else {A_FEVir;}
    
        FEa[3]= FEactM;								// Switch data storage in array
        FEactM = FEa[3];						    // Switch data storage in array
        FEa[4]= A_FEVir;				            // Switch data storage in array (Angle of Force Env. Actual will be Angle of Force Env. Virtual)
        FEactMa= FEa[4]*(180.0/M_PI);				// Switch data storage in array (Degree to Rad)
        FEa[0]= FEactM*cos(A_FEVir);		        // Actual Force in X direction on a global coordinate frame
        FEactx= FEa[0];								// Switch data storage in array
        FEa[1]= FEactM*sin(A_FEVir);		        // Actual Force in Y direction on a global coordinate frame
        FEacty= FEa[1];								// Switch data storage in array
        FEa[2]= FEactM*0.0;							// A moment is defined as zero
        FEactNull= FEa[2];							// Switch data storage in array
        return 0;
    }*/

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
    //Force compensation by constant [Pdf]=[Pd]+[Po]
    /*double ForceIdeal::ForceCompensateCal(){
        Po[0]=(Pod*cos(A_DVir));       //Pox in m.
        Po[1]=(Pod*sin(A_DVir));       //Poy in m.
        Po[2]=A_DVir;                  //Ac in rad

        Pdf[0]=Pd[0]+Po[0];            //Pfdx in m.
        Pdf[1]=Pd[1]+Po[1];            //Pfdx in m.
        Pdf[2]=Pd[2]+0;                //Afd in rad.   
        return 0;
    }*/
    
}
