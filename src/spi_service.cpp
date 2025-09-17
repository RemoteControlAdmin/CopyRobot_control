#include <iostream>

#include "spi_service.hpp"


SPIService::SPIService(){
 // rp1_t* setup_rp1();
  rp1 = setup_rp1();
  if (!rp1) { printf("RP1初期化失敗\n");             //rp1初期化
                               return ;  }
    
  spi0 = setup_spi(rp1, 0, /*mode=*/0, /*data_bits=*/16, /*baud_div=*/200, /*frf=*/0);
  spi1 = setup_spi(rp1, 1, /*mode=*/0, /*data_bits=*/ 8, /*baud_div=*/200, /*frf=*/0);
  spi2 = setup_spi(rp1, 2, 1, 8, 100, 0); //SPI2初期化(固定,チャンネル)

  V1=0.0;
  V2=0.0;
  V3=0.0;

  uint8_t on=0xFA;
  send_spi(spi1, on);
  //printf("Start %x\n", on);
  //usleep(1000); //少し待つ

}


bool SPIService::pico2_motor(float V1,float V2,float V3){

  if((-12.0<=V1 && V1<=12.0)&&(-12.0<=V2 && V2<=12.0)&&(-12.0<=V3 && V3<=12.0)){

      //std::cout << "V1: " << V1 << " V2: " << V2 << " V3: " << V3 << std::endl;
      uint16_t hexa12=Input_value1(V1,V2); 
      uint8_t hexa3=Input_value2(V3);

      //printf("Sending 16bit data...\n");

      send_spi(spi0, hexa12); // SPI0送信
      send_spi(spi1, hexa3);  // SPI1送信
      //printf("[spi0] CTRLR0=0x%08X\n", *(volatile uint32_t*)(spi0->regbase + DW_SPI_CTRLR0));
      //printf("[spi1] CTRLR0=0x%08X\n", *(volatile uint32_t*)(spi1->regbase + DW_SPI_CTRLR0));

      return true;
  }
  return false;
}

void SPIService::init_adc(){
  b_fax = B_FAx_INIT;
  b_fbx = B_FBx_INIT;
  b_fay = B_FAy_INIT;
  b_fby = B_FBy_INIT;

  auto_zero_calib_reg(spi2, &b_fax, &b_fay, &b_fbx, &b_fby, ZERO_SAMPLES);

}

std::vector<double> SPIService::read_adc(){

  mcp_result_t adc_result = read_ADC(spi2, b_fax, b_fay, b_fbx, b_fby);  // ADC読み取り(ループ) */
  std::vector<double> forces = {adc_result.force[0], adc_result.force[1], adc_result.force[2], adc_result.force[3]};
  return forces;
}

SPIService::~SPIService(){
  send_spi(spi1, 0xF9); // SPI1に終了信号を送信
  std::cout << "[INFO] SPIService Destructed" << std::endl;
}
