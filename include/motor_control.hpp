#ifndef MORTOR_CONTROL_HPP
#define MORTOR_CONTROL_HPP
#define _USE_MATH_DEFINES  // M_PIを使用するために必要
# include <vector>
# include <cmath>
# include "common.hpp"

namespace motorcontrol {

    class MotorControl {

    public:
    /*
    *    ========= public =========
    */

    MotorControl();
    
    Eigen::Vector3d convert_wheeltovoltage(Eigen::Vector3d forwheelvelocity);

    void EnableMotorDrive();

    void DisableMotorDrive();

    private:
    /*
    *    ========= private =========
    */
    void setperiod_channel();
    
    void pinMode(int pin, const char *direction);

    void digitalWrite(int pin, int value);

    void frequencyWrite(int chip, int channel, int Period);
    
    void analogWrite(int chip, int channel, int duty_cycle);

    void enable_pwm(int chip, int channel);
    
    void disable_pwm(int chip, int channel);

    void handle_termination(int signum);

    /*
    *    ========= private parameter =========
    */
   Eigen::Vector3d mortor_voltage;

    int Possock;
    const int frequency_drive = 1000; 
    const int period=1.0/(frequency_drive*1e-9);

    const int DCM1A=0; //P8_19 PWM_2A_(7:0) //Motor 1
    const int DCM1B=1; //P8_13 PWM_2B_(7:1) //Motor 1
    const int DCM2A=0; //P9_31 PWM_0A_(1:0) //Motor 2
    const int DCM2B=1; //P9_29 PWM_0B_(1:1) //Motor 2
    const int DCM3A=0; //P9_14 PWM_1A_(4:0) //Motor 3
    const int DCM3B=1; //P9_16 PWM_1B_(4:1) //Motor 3
    const int EN1=26, EN2=65, EN3=46;
    const int pwm_1=7, pwm_2=1, pwm_3=4; //PWM Chip No. 7,1,4

    char DObufdat[2048];
    char Pathbufdat[2048];
    const double WwtoVoltGain = 12/32.9754;

    // Set voltage power supply for a DC Motor   
    const double Vss = 24;
    

}; // class MotorControl
} // namespace motorcontrol

#endif // MORTOR_CONTROL_HPP