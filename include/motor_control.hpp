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

# include "common.hpp"

namespace motorcontrol {

    class MotorControl {

    public:
    /*
    *    ========= public =========
    */

    MotorControl();

    Eigen::Vector3d convert_wheeltovoltage(Eigen::Vector3d forwheelvelocity);

    bool send_voltage(float V1, float V2, float V3);

    ~MotorControl();

    private:

    bool dummy_spi0_initialize(const char *device = "/dev/spidev0.0",
                                      uint8_t mode = 0,
                                      uint8_t bits = 16,
                                      uint32_t speed = 1000000);
    /*
    *    ========= private =========
    */

    static MotorControl* instance; 
    
    rp1_t *rp1;
    rp1_spi_instance_t *spi0;
    rp1_spi_instance_t *spi1;
    rp1_spi_instance_t *spi2;
    /*
    *    ========= private parameter =========
    */
    Eigen::Vector3d mortor_voltage;
    const double wwtovoltgain = 12/32.9754;


}; // class MotorControl


} // namespace motorcontrol

#endif // MORTOR_CONTROL_HPP