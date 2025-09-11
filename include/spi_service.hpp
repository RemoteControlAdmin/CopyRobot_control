# pragma once

// 初期オフセット（ゼロ校正で上書き）
#define B_FAx_INIT  (-16.7597)
#define B_FAy_INIT  (-16.7597)
#define B_FBx_INIT  (-16.8527)
#define B_FBy_INIT  (-16.7597)
// ゼロ校正のサンプル数
#define ZERO_SAMPLES  300


#include "rpi5-rp1-spi.h"
#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"
#include "rp1-spi-util.h"
#include "pi_pico_commands.h"

#include <vector>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <signal.h>
#include <iostream>

class SPIService{
 public:
 SPIService();

 bool pico2_motor(float V1,float V2,float V3);
 void init_adc();
 std::vector<double> read_adc();
 
 ~SPIService();

 private:
 float V1;
 float V2;
 float V3;

  rp1_t *rp1;
  rp1_spi_instance_t *spi0;
  rp1_spi_instance_t *spi1;
  rp1_spi_instance_t *spi2;


  // ADC初期化
  double b_fax, b_fbx;
  double b_fay, b_fby;

};
