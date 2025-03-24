# include "motor_control.hpp"

namespace motorcontrol{
    MotorControl::MotorControl(){
        setperiod_channel();
    }

    /*
    *    ========= private =========
    */
    
    void MotorControl::setperiod_channel(){

        if (signal(SIGINT, handle_termination) == SIG_ERR) {
            perror("Error setting up signal handler");
            return;
        }
        
        //Call script file to setup PWM pin
        system("bash config_pin.sh");
        //Set period for each channel
        frequencyWrite(pwm_1, DCM1A, period);
        frequencyWrite(pwm_1, DCM1B, period);
        frequencyWrite(pwm_2, DCM2A, period);
        frequencyWrite(pwm_2, DCM2B, period);
        frequencyWrite(pwm_3, DCM3A, period);
        frequencyWrite(pwm_3, DCM3B, period);
    }

    void MotorControl::pinMode(int pin, const char *direction){
        snprintf(Pathbufdat, sizeof(Pathbufdat), "%s%d/direction", "/sys/class/gpio/gpio", pin);
        int fd_direction = open(Pathbufdat, O_WRONLY);
        if (fd_direction == -1) {
            perror("Error opening direction file");
            exit(EXIT_FAILURE);
        }
    
        if (write(fd_direction, direction, strlen(direction)) == -1) {
            perror("Error writing to direction");

            exit(EXIT_FAILURE);
        }
        close(fd_direction);
    }

    void MotorControl::digitalWrite(int pin, int value){
        snprintf(Pathbufdat, sizeof(Pathbufdat), "%s%d/value", "/sys/class/gpio/gpio", pin);
        int fd_value = open(Pathbufdat, O_WRONLY);
        if (fd_value == -1) {
            perror("Error opening value file");
            exit(EXIT_FAILURE);
        }
        snprintf(DObufdat, sizeof(DObufdat), "%d", value);
    
        if (write(fd_value, DObufdat, strlen(DObufdat)) == -1) {
            perror("Error writing to value");
            exit(EXIT_FAILURE);
        }
        close(fd_value);
    }

    void MotorControl::frequencyWrite(int chip, int channel, int Period) {
        char path[50];
        sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/period", chip, chip, channel);
        int period_fd = open(path, O_WRONLY);
        if (period_fd == -1) {
            perror("Error opening period file");
            exit(EXIT_FAILURE);
        }
        dprintf(period_fd, "%d", Period);
        close(period_fd);
    }

    //Analog write a duty of a PWM pin
    void MotorControl::analogWrite(int chip, int channel, int duty_cycle) {
        char path[50];
        sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/duty_cycle", chip, chip, channel);
        int duty_cycle_fd = open(path, O_WRONLY);
        if (duty_cycle_fd == -1) {
            perror("Error opening duty_cycle file");
            exit(EXIT_FAILURE);
        }
        dprintf(duty_cycle_fd, "%d", duty_cycle);
        close(duty_cycle_fd);
    }

    //Set the enable of a PWM pin
    void MotorControl::enable_pwm(int chip, int channel) {
        char path[50];
        sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/enable", chip, chip, channel);    
        int enable_fd = open(path, O_WRONLY);
        if (enable_fd == -1) {
            perror("Error opening enable file");
            exit(EXIT_FAILURE);
        }
        dprintf(enable_fd, "1");
        close(enable_fd);
    }

    //Set the disable of a PWM pin
    void MotorControl::disable_pwm(int chip, int channel) {
        char path[50];
        sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/enable", chip, chip, channel); 
        int enable_fd = open(path, O_WRONLY);
        if (enable_fd == -1) {
            perror("Error opening enable file for disabling PWM");
            exit(EXIT_FAILURE);
        }
        dprintf(enable_fd, "0");
        close(enable_fd);
    }
    // Signal handler to stop the loop gracefully on Ctrl+C
    void MotorControl::handle_termination(int signum) {
        if(instance != nullptr){
            instance->DisableMotorDrive(); // インスタンス経由で呼び出す
            close(instance->Possock);      // インスタンス経由でアクセスする
        }
    }

    /*
    *    ========= public =========
    */
    

    Eigen::Vector3d MotorControl::convert_wheeltovoltage(Eigen::Vector3d forwheelvelocity){
        mortor_voltage = forwheelvelocity * wwtovoltgain;

        return mortor_voltage;
    }

    //Enable DC motor drive
    void MotorControl::EnableMotorDrive(Eigen::Vector3d mortor_voltage){
        
        // Set GPIO directions
        pinMode(EN1, "out");
        pinMode(EN2, "out");
        pinMode(EN3, "out");
        // Write digital value
        digitalWrite(EN1, 1);
        digitalWrite(EN2, 1);
        digitalWrite(EN3, 1);

        int V1A_peri = abs(((+(0.5*mortor_voltage[0])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
        int V1B_peri = abs(((-(0.5*mortor_voltage[0])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
        int V2A_peri = abs(((+(0.5*mortor_voltage[1])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
        int V2B_peri = abs(((-(0.5*mortor_voltage[1])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
        int V3A_peri = abs(((+(0.5*mortor_voltage[2])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
        int V3B_peri = abs(((-(0.5*mortor_voltage[2])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)

        // Enable PWM channels
        enable_pwm(pwm_1, DCM1A);
        enable_pwm(pwm_1, DCM1B);
        enable_pwm(pwm_2, DCM2A);
        enable_pwm(pwm_2, DCM2B);
        enable_pwm(pwm_3, DCM3A);
        enable_pwm(pwm_3, DCM3B);
        // Set duty cycle for each channel
        analogWrite(pwm_1, DCM1A, V1A_peri);
        analogWrite(pwm_1, DCM1B, V1B_peri);
        analogWrite(pwm_2, DCM2A, V2A_peri);
        analogWrite(pwm_2, DCM2B, V2B_peri);
        analogWrite(pwm_3, DCM3A, V3A_peri);
        analogWrite(pwm_3, DCM3B, V3B_peri);  
        return;  
    }
    //Disable DC motor drive
    void MotorControl::DisableMotorDrive(){
        disable_pwm(pwm_1, DCM1A);
        disable_pwm(pwm_1, DCM1B);
        disable_pwm(pwm_2, DCM2A);
        disable_pwm(pwm_2, DCM2B);
        disable_pwm(pwm_3, DCM3A);
        disable_pwm(pwm_3, DCM3B);
        digitalWrite(EN1, 0);
        digitalWrite(EN2, 0);
        digitalWrite(EN3, 0);
        return;  
    }
    

    
}