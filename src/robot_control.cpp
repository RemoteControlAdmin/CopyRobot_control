#include "robot_control.hpp"



namespace robotcontrol{
    RobotControl::RobotControl(){
        Integral = std::vector<double> {0,0,0};
        Derivative = std::vector<double> {0,0,0};
        Kp = 9; // medium 6.5 high 25
        Ki = 0;
        Kd = 0.2; // medium 2.5 high 17
    }
    void RobotControl::chenge_pid(double kp, double ki, double kd){
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
    //private
    // public
    //[Vc] PID Closed Loop Control Velocity Command
    Eigen::Vector3d RobotControl::CLPositionControllerPIDCal(std::array<double, 3> err_data, std::array<double, 3> last_err_data, int microdt){
        int i;
        //printf("Global Velocity Matrix Command");
        for(i=0;i<3;i++){
        //	printf("\n");
        //	[Vc]=Kp*[Pe] + Ki*integral([Pe]) + Kd*derivative([Pe]) 
            Integral[i]		+= err_data[i];								//Calculate the integral term
            Derivative[i]	= err_data[i] - last_err_data[i];						//Calculate the derivative term
            velocity_data[i] = Kp*err_data[i] + Ki*Integral[i] * microdt*1e-6 + Kd*Derivative[i] / (microdt*1e-6);	//Calculate the output term
        //	printf("%+.4f",Vc[i]);
        }
        return velocity_data;
    }

    bool RobotControl::VelocityLimitationCal(Eigen::Vector3d velocity_data){
        //[Wcmax]=[-m]*[Vlc]+[Wgmax],[Wcmin]=[+m]*[Vlc]-[Wgmax]
        double Vlc=sqrt((velocity_data[0]*velocity_data[0])+(velocity_data[1]*velocity_data[1]));
        double Wlc=velocity_data[2];
        double Wcmax=-8.6962*Vlc+14.3400;
        double Wcmin=+8.6962*Vlc-14.3400;

        if(((Vlc>=0)&&(Vlc<=1.6490))&&((Wlc>=Wcmin)&&(Wlc<=Wcmax))){
            return  true;
            //ChkStop=0;
        }
        
        return false;
        }

   

}
    
