#ifndef KANSU_H
#define KANSU_H

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
rp1_spi_instance_t* setup_spi(rp1_t *rp1,int ch);
void send_spi(rp1_spi_instance_t *spi, uint16_t txdata);
void read_ADC(rp1_spi_instance_t *spi2);
uint16_t Input_value1(float V1,float V2);
uint8_t Input_value2(float V3);

#ifdef __cplusplus
}
#endif

#endif // KANSU_H