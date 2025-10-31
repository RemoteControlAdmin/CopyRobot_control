#include "robot/robot_control.hpp"

#include <iostream>

namespace robot_lib{
    RobotControl::RobotControl(){
        Integral = std::vector<double> {0,0,0};
        Derivative = std::vector<double> {0,0,0};
        Kp = 35; // medium 35
        Ki = 35; // 5
        Kd = 1; //

        Kp_force = 0.003125; // medium 0.5 high 1.0
        Ki_force = 0.003125; // medium 0.01 high 0.1
        Kd_force = 0; // medium 0.01 high 0
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

    //[Vc] Unilateral with Force 

    std::vector<double> RobotControl::corrected_pos(std::vector<double> master_data, Eigen::Vector3d force_pos_data){
        std::vector<double> corrected_data(3);
        for (int i = 0; i < 3; i++) {
            corrected_data[i] = master_data[i] - force_pos_data[i]; //
        }
        return corrected_data;
    }

    std::vector<double> RobotControl::unilateral_force_control(std::vector<double> force_actual_data, std::vector<double> force_virtual_data,
         std::vector<double> force_ideal_data, std::vector<double> master_data, int microdt){
        actual_vector_data = Eigen::Vector3d(
            force_actual_data[0] * std::cos(force_virtual_data[4]),
            force_actual_data[0] * std::sin(force_virtual_data[4]),
            0.0
        );

        ideal_vector_data = Eigen::Vector3d(
            force_ideal_data[0],
            force_ideal_data[1],
            0.0
        );
        force_err = ideal_vector_data - actual_vector_data; // Force error
        //	[Vc]=Kp*[Pe] + Ki*integral([Pe]) + Kd*derivative([Pe]) 
        Integral_force		+= force_err;								//Calculate the integral term
        Derivative_force	= force_err - last_force_err;						//Calculate the derivative term
        pid_force_data = Kp_force * force_err + Ki_force * Integral_force * (microdt*1e-6) + Kd_force * Derivative_force / (microdt*1e-6);	//Calculate the output term

        force_pos_data = -pid_force_data / 320;

        last_force_err = force_err; // Update last force error for next iteration
        std::vector<double> corrected_data = RobotControl::corrected_pos(master_data, force_pos_data);
        //	printf("%+.4f",Vc[i]);
        return corrected_data;
    }

    std::vector<double> RobotControl::bilateral_force_control(std::vector<double> force_actual_data, std::vector<double> force_virtual_data,
        std::vector<double> force_udp_data, std::vector<double> master_data, int microdt){
        if(force_virtual_data[2] >= 0.38){
            return master_data;
        }
        double z = (0.38 - force_virtual_data[2])/0.001;
        double sigmoid = 1/(1 + std::exp(-z));
        actual_vector_data = Eigen::Vector3d(
            force_actual_data[0] * std::cos(force_virtual_data[1]),
            force_actual_data[0] * std::sin(force_virtual_data[1]),
            0.0
        );

        udp_vector_data = Eigen::Vector3d(
            force_udp_data[0] * std::cos(force_virtual_data[1]),
            force_udp_data[0] * std::sin(force_virtual_data[1]),
            0.0
        );
        force_err = udp_vector_data - actual_vector_data; // Force error
        //	[Vc]=Kp*[Pe] + Ki*integral([Pe]) + Kd*derivative([Pe]) 
        Integral_force		+= force_err;								//Calculate the integral term
        Derivative_force	= force_err - last_force_err;						//Calculate the derivative term
        pid_force_data = Kp_force * force_err + Ki_force * Integral_force * (microdt*1e-6) + Kd_force * Derivative_force / (microdt*1e-6);	//Calculate the output term

        force_pos_data = -pid_force_data * sigmoid;

        last_force_err = force_err; // Update last force error for next iteration
        std::vector<double> corrected_data = RobotControl::corrected_pos(master_data, force_pos_data);
        //	printf("%+.4f",Vc[i]);
        return corrected_data;
    }

    

   

}
    
