#ifndef MORTOR_CONTROL_HPP
#define MORTOR_CONTROL_HPP
#define _USE_MATH_DEFINES  // M_PIを使用するために必要
# include <vector>
# include <cmath>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <signal.h>

#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"
#include "rp1-spi-util.h"
#include "pi_pico_commands.h"
#include "rpi5-rp1-spi.h"
#include "spi_service.hpp"

# include "common.hpp"

namespace motorcontrol {

    class MotorControl {

    public:
    /*
    *    ========= public =========
    */

    MotorControl();

    Eigen::Vector3d convert_wheeltovoltage(Eigen::Vector3d forwheelvelocity);

    void send_voltage(float V1, float V2, float V3, SPIService& spi_service);

    ~MotorControl();

    private:

    /*
    *    ========= private =========
    */

    /*
    *    ========= private parameter =========
    */
    Eigen::Vector3d motor_voltage;
    bool result = false;
    const double wwtovoltgain = 12/32.9754;


}; // class MotorControl


} // namespace motorcontrol

#endif // MORTOR_CONTROL_HPP