#include "robot/motor_control.hpp"

namespace robot_lib{
    MotorControl::MotorControl(){}        

    Eigen::Vector3d MotorControl::convert_wheeltovoltage(Eigen::Vector3d forwheelvelocity){
        motor_voltage = forwheelvelocity * wwtovoltgain;

        return motor_voltage;
    }


    /*
    *    ========= private =========
    */
    void MotorControl::send_voltage(float V1,float V2,float V3, spi_lib::SPIService& spi_service){
        result = spi_service.pico2_motor(V1, V2, V3);
        if(!result){
            std::cout << "[Error] Voltage out of range. Motor control failed" << std::endl;
            //stop_flag = true;
        }
    }


    //Disable DC motor drive
    MotorControl::~MotorControl(){}
    

    
}