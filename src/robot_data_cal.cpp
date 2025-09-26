#include "robot_data_cal.hpp"



namespace robotcontrol{
    RobotDataCal::RobotDataCal(){
    }
    //private
    std::vector<double> RobotDataCal::convert_m_rad(std::vector<double> data/*master robot data*/){
        data[0] = data[0]/1000.0; // converted to m
        data[1] = data[1]/1000.0; // converted to m
        data[2] = fmod(data[2]*M_PI /180.0+M_PI,2*M_PI)-M_PI;; //Wrap to the range from [0,360] to [-pi, +pi]
        data[3] = data[3]/1000.0; // converted to m
        data[4] = data[4]/1000.0; // converted to m
        data[5] = data[5]*M_PI/180.0; 	// converted to rad/s
        return data;
    }
    // public
    std::tuple <std::vector<double>, std::vector<double>, std::vector<double>> RobotDataCal::convert_robotdata(std::vector<double> master_data, 
        std::vector<double> copy_data, std::vector<double> partner_master_data){
        master_data = RobotDataCal::convert_m_rad(master_data);
        copy_data = RobotDataCal::convert_m_rad(copy_data);
        partner_master_data = RobotDataCal::convert_m_rad(partner_master_data);

        return {master_data, copy_data, partner_master_data};
    }
    //[Pe]=[Pd]-[Pg]
    std::array<double,3> RobotDataCal::err_robotposition_cal(std::vector<double> master_data, std::vector<double> copy_data){
        int i;
        for(i=0;i<2;i++){
            err_data[i]=master_data[i]-copy_data[i];
        }
        err_data[2] = atan2(sin(master_data[2] - copy_data[2]),
                                      cos(master_data[2] - copy_data[2]));
        return err_data;
    }

    
    //[Pd] Linear movement
    std::vector<double> RobotDataCal::MRobot_Linear_PositionCal(std::vector<double> data, int time){
        if     ((time>=0)&&(time<= 5*pow(10,6)))	{data[0]=0.4;data[1]=0.4;data[2]=M_PI/2;}
        else if((time> 5*pow(10,6))&&(time<= 8*pow(10,6)))	{data[0]=0.6;data[1]=0.6;data[2]=M_PI/2;}
        else										{data[0]=0.4;data[1]=0.4;data[2]=M_PI/2;}
        return data;
    }
    /*
        //[Pd] Rotate movement
    double MRobot_Rotate_PositionCal(){
        if     ((Elapsed>=0000)&&(Elapsed<= 5000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(1*M_PI)/4;}
        else if((Elapsed> 5000)&&(Elapsed<= 8000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(3*M_PI)/4;}
        else if((Elapsed> 8000)&&(Elapsed<=11000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(5*M_PI)/4;}
        else if((Elapsed>11000)&&(Elapsed<=14000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(7*M_PI)/4;}
        else if((Elapsed>14000)&&(Elapsed<=17000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(5*M_PI)/4;}
        else if((Elapsed>17000)&&(Elapsed<=20000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(3*M_PI)/4;}
        else										{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(1*M_PI)/4;}
        return 0;
    }
    //[Pd] Square movement
    double MRobot_Square_PositionCal(){
        if     ((Elapsed>=0000)&&(Elapsed<= 4000))	{Pd[0]=0.40;Pd[1]=0.40;Pd[2]=M_PI/2;}
        else if((Elapsed> 4000)&&(Elapsed<= 8000))	{Pd[0]=0.60;Pd[1]=0.40;Pd[2]=M_PI/2;}
        else if((Elapsed> 8000)&&(Elapsed<=12000))	{Pd[0]=0.60;Pd[1]=0.60;Pd[2]=M_PI/2;}
        else if((Elapsed>12000)&&(Elapsed<=16000))	{Pd[0]=0.40;Pd[1]=0.60;Pd[2]=M_PI/2;}
        else if((Elapsed>16000)&&(Elapsed<=20000))	{Pd[0]=0.40;Pd[1]=0.40;Pd[2]=M_PI/2;}
        else										{Pd[0]=0.40;Pd[1]=0.40;Pd[2]=M_PI/2;}
        return 0;
    }
    //[Pd] Circle movement
    double MRobot_Circle_PositionCal(){
        double F=0.05;
        double Wc=2.0*M_PI*F*(Elapsed/1000);
        if((Elapsed>=0)&&(Elapsed<=5000))			{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
        else if ((Elapsed>5000)&&(Elapsed<15000))	{Pd[0]=(0.10*cos(1*Wc))+0.50;Pd[1]=(0.10*sin(1*Wc))+0.50;Pd[2]=M_PI/2.0;}
        else										{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
        return 0;
    }
    //[Pd] Eight movement
    double MRobot_Eight_PositionCal(){
        double F=0.05;
        double Wc=2.0*M_PI*F*(Elapsed/1000);
        if((Elapsed>=0)&&(Elapsed<=5000))			{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
        else if ((Elapsed>5000)&&(Elapsed<25000))	{Pd[0]=(0.10*cos(1*Wc))+0.50;Pd[1]=(0.10*sin(2.0*Wc))+0.50;Pd[2]=M_PI/2.0;}
        else										{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
        return 0;
    }
    //[Pd] Heart movement
    double MRobot_Heart_PositionCal(){
        double F=0.05;
        double Wc=2.0*M_PI*F*(Elapsed/1000);
        if ((Elapsed>0)&&(Elapsed<20000))			{Pd[0]=0.02*16.0*pow(sin(Wc),3)+0.5,Pd[1]= 0.02*(13.0*cos(Wc)-5.0*cos(2.0*Wc)-2.0*cos(3.0*Wc)-1.0*cos(4.0*Wc))+0.55,Pd[2]=M_PI/2.0;}
        else										{Pd[0]=0.50,Pd[1]=0.65,Pd[2]=M_PI/2.0;}
        return 0;
    }
        */


    

}

