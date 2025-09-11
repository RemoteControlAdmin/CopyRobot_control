#ifndef RPIRP1_SPI_H
#define RPIRP1_SPI_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>

#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"
#include "rp1-spi-util.h"
#include "pi_pico_commands.h"

#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <signal.h>


#ifdef __cplusplus
extern "C" {
#endif

rp1_t* setup_rp1();
rp1_spi_instance_t* setup_spi(rp1_t *rp1, int ch,
                              int mode, int data_bits,
                              int baud_div, int frf);
void send_spi(rp1_spi_instance_t *spi, uint16_t txdata);
void auto_zero_calib_reg(rp1_spi_instance_t *spi,
                                double *b_fax, double *b_fay,
                                double *b_fbx, double *b_fby,
                                int samples);

typedef struct {
    int    raw[4];
    double volt[4];
    double force[4];
} mcp_result_t;

mcp_result_t read_ADC(rp1_spi_instance_t *spi2, double b_fax, double b_fay, double b_fbx, double b_fby);

uint16_t Input_value1(float V1,float V2);
uint8_t Input_value2(float V3);


#ifdef __cplusplus
}
#endif

#endif // RPIRP1_SPI_H
