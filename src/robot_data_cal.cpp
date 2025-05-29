#include "robot_data_cal.hpp"



namespace robotcontrol{
    RobotDataCal::RobotDataCal(){
    }
    //private
    std::vector<double> RobotDataCal::convert_m_rad(std::vector<double> data/*master robot data*/){
        data[0] = data[0]/1000.0; // converted to m
        data[1] = data[1]/1000.0; // converted to m
        data[2] = fmod(data[2]*M_PI /180.0+M_PI,2*M_PI)-M_PI;; //Wrap to the range from [0,360] to [-pi, +pi]

        return data;
    }
    // public
    RobotData RobotDataCal::convert_robotdata(RobotData robotdata){
        robotdata.master_data = RobotDataCal::convert_m_rad(robotdata.master_data);
        robotdata.copy_data = RobotDataCal::convert_m_rad(robotdata.copy_data);

        return robotdata;
    }
    //[Pe]=[Pd]-[Pg]
    std::array<double,3> RobotDataCal::err_robotposition_cal(RobotData robotdata){
        int i;
        for(i=0;i<3;i++){
            robotdata.err_data[i]=robotdata.master_data[i]-robotdata.copy_data[i];
        }
        return robotdata.err_data;
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

