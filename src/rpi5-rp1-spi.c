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
#include <string.h>
#include <signal.h>

void delay_ms(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

// pci bar info
// from: https://github.com/G33KatWork/RP1-Reverse-Engineering/blob/master/pcie/hacks.py
#define RP1_BAR1 0x1f00000000
#define RP1_BAR1_LEN 0x400000

// offsets from include/dt-bindings/mfd/rp1.h
// https://github.com/raspberrypi/linux/blob/rpi-6.1.y/include/dt-bindings/mfd/rp1.h
#define RP1_IO_BANK0_BASE 0x0d0000
#define RP1_RIO0_BASE 0x0e0000
#define RP1_PADS_BANK0_BASE 0x0f0000

// the following info is from the RP1 datasheet (draft & incomplete as of 2024-02-18)
// https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
#define RP1_ATOM_XOR_OFFSET 0x1000
#define RP1_ATOM_SET_OFFSET 0x2000
#define RP1_ATOM_CLR_OFFSET 0x3000

#define PADS_BANK0_VOLTAGE_SELECT_OFFSET 0
#define PADS_BANK0_GPIO_OFFSET 0x4

#define RIO_OUT_OFFSET 0x00
#define RIO_OE_OFFSET 0x04
#define RIO_NOSYNC_IN_OFFSET 0x08
#define RIO_SYNC_IN_OFFSET 0x0C
//                           3         2         1
//                          10987654321098765432109876543210
#define CTRL_MASK_FUNCSEL 0b00000000000000000000000000011111
#define PADS_MASK_OUTPUT  0b00000000000000000000000011000000

#define CTRL_FUNCSEL_RIO 0x05

#define SPI_SPEED 1000000
#define REF_VOLTAGE 3.3


#ifdef __cplusplus
extern "C" {
#endif

void *mapgpio(off_t dev_base, off_t dev_size)
{
    int fd;
    void *mapped;
    
    printf("sizeof(off_t) %d\n", sizeof(off_t));

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        printf("Can't open /dev/mem\n");
        return (void *)0;
    }

    mapped = mmap(0, dev_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base);
    // close(fd);

    printf("base address: %llx, size: %x, mapped: %p\n", dev_base, dev_size, mapped);

    if (mapped == (void *)-1)
    {
        printf("Can't map the memory to user space.\n");
        return (void *)0;
    }

    return mapped;
}



bool create_rp1(rp1_t **rp1, void *base)
{

    rp1_t *r = (rp1_t *)calloc(1, sizeof(rp1_t));
    if (r == NULL)
        return false;

    r->rp1_peripherial_base = base;
    r->gpio_base = base + RP1_IO_BANK0_BASE;
    r->pads_base = base + RP1_PADS_BANK0_BASE;
    r->rio_out = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OUT_OFFSET);
    r->rio_output_enable = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OE_OFFSET);
    r->rio_nosync_in = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_NOSYNC_IN_OFFSET);

    *rp1 = r;

    return true;
}

bool create_pin(uint8_t pinnumber, rp1_t *rp1)
{
    gpio_pin_t *newpin = (gpio_pin_t *)calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = pinnumber;

    // each gpio has a status and control register
    // adjacent to each other. control = status + 4 (uint8_t)
    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pinnumber);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pinnumber + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pinnumber * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = CTRL_FUNCSEL_RIO;  // now set the value we need

    rp1->pins[pinnumber] = newpin;
    printf("pin %d stored in pins array %p\n", pinnumber, rp1->pins[pinnumber]);

    return true;
}

bool create_pin_2(uint8_t pinnumber, rp1_t *rp1, uint32_t funcmask)
{
    gpio_pin_t *newpin = (gpio_pin_t *)calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = pinnumber;

    // each gpio has a status and control register
    // adjacent to each other. control = status + 4 (uint8_t)
    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pinnumber);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pinnumber + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pinnumber * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = funcmask;  // now set the value we need

    rp1->pins[pinnumber] = newpin;
    //printf("pin %d stored in pins array %p\n", pinnumber, rp1->pins[pinnumber]);

    return true;
}

int pin_enable_output(uint8_t pinnumber, rp1_t *rp1)
{

    printf("Attempting to enable output\n");
   
    // first enable the pad to output
    // pads needs to have OD[7] -> 0 (don't disable output)
    // and                IE[6] -> 0 (don't enable input)
    // we use atomic access to the bit clearing alias with a mask
    // divide the offset by 4 since we're doing uint32* math

    volatile uint32_t *writeadd = rp1->pins[pinnumber]->pad + RP1_ATOM_CLR_OFFSET / 4;

    printf("attempting write for %p at %p\n", rp1->pins[pinnumber]->pad, writeadd);

    *writeadd = PADS_MASK_OUTPUT;

    // now set the RIO output enable using the atomic set alias
    *(rp1->rio_output_enable + RP1_ATOM_SET_OFFSET / 4) = 1 << rp1->pins[pinnumber]->number;

    return 0;
}

void pin_on(rp1_t *rp1, uint8_t pin)
{
    *(rp1->rio_out + RP1_ATOM_SET_OFFSET / 4) = 1 << pin;
}
void pin_off(rp1_t *rp1, uint8_t pin)
{
    *(rp1->rio_out + RP1_ATOM_CLR_OFFSET / 4) = 1 << pin;
}


const uint8_t pins[] = {17, 27, 22, 23};

void setup_spi0_pins(rp1_t *rp1){

    create_pin_2(8, rp1, 0x00);     // CS0
    create_pin_2(9, rp1, 0x00);     // MISO
    create_pin_2(10, rp1, 0x00);    // MOSI
    create_pin_2(11, rp1, 0x00);    // SCLK

}

void setup_spi1_pins(rp1_t *rp1){

    create_pin_2(18, rp1, 0x00);     // CS0
    create_pin_2(19, rp1, 0x00);     // MISO
    create_pin_2(20, rp1, 0x00);    // MOSI
    create_pin_2(21, rp1, 0x00);    // SCLK

}


void setup_spi2_pins(rp1_t *rp1){

    create_pin_2(0, rp1, 0x08);     // CS0
    create_pin_2(1, rp1, 0x08);     // MISO
    create_pin_2(2, rp1, 0x08);    // MOSI
    create_pin_2(3, rp1, 0x08);    // SCLK

}

void setup_spi3_pins(rp1_t *rp1){

    create_pin_2(4, rp1, 0x08);     // CS0
    create_pin_2(5, rp1, 0x08);     // MISO
    create_pin_2(6, rp1, 0x08);    // MOSI
    create_pin_2(7, rp1, 0x08);    // SCLK

}


//////////////////////////////////////////////////////////////////////
//関数/////////////////////////////////////////////////////////////////

//RP1 初期化関数
rp1_t* setup_rp1(){   // RP1 初期化
    rp1_t *rp1;
            void *base = mapgpio(RP1_BAR1, RP1_BAR1_LEN);
    if (base == NULL) {
        printf("unable to map base\n");
        return NULL;
    }

    printf("creating rp1\n");
    if (!create_rp1(&rp1, base)) {
        printf("unable to create rp1\n");
        return NULL;
    }

    return rp1;
}


// SPI初期化関数
rp1_spi_instance_t* setup_spi(rp1_t *rp1,int ch){    // SPI初期化

    rp1_spi_instance_t *spi;
    //printf("spi=%p\n",spi);
    if (!rp1_spi_create(rp1, ch, &spi)) {
        printf("unable to create spi\n");
        return NULL;
    }

    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x0; // SPI無効化
    switch(ch){
     case 0: setup_spi0_pins(rp1); break;
     case 1: setup_spi1_pins(rp1); break;
     case 2: setup_spi2_pins(rp1); break;
     case 3: setup_spi3_pins(rp1); break;
     default: return NULL; // 無効なチャンネル
    }

    *(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR) = 200; // 10MHz設定（200/20）

    // SPIモード設定 (Mode1) と bit設定
    uint32_t reg_ctrlr = *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0);
    //reg_ctrlr0 |= DW_PSSI_CTRLR0_SCPHA;              // CPHA=1
    reg_ctrlr &= ~DW_PSSI_CTRLR0_SCPHA;
    reg_ctrlr &= ~DW_PSSI_CTRLR0_DFS_MASK;           // DFSビットクリア


    if(ch==0){reg_ctrlr|= 0x0F;} // DFS=15 (16bit)
    else if(ch==1||ch==2){reg_ctrlr |= 0x07;} // DFS=7 (8bit)
    else{return NULL;}

    
    *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0) = reg_ctrlr;


    /*デバック
   printf("ctrlr0 after setting: 0x%08X\n", reg_ctrlr);
   printf("CTRLR0 = 0x%08X\n", *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0));
   printf("baudr raw: %u\n", *(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR));
   printf("baudr: %d MHz\n", 200 / *(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR));
   */
   

    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x1; // SPI有効化


    return spi;
}


//ADC読み取り関数
void read_ADC(rp1_spi_instance_t *spi2){

    printf("Sending 8bit data...\n");

  const double VREF = 3.30;  // MCP3208の基準電圧
  //const uint8_t channels[4] = {0, 1, 2, 3};
  uint8_t recieveData[4]; // 受信データ用バッファ

 while (1) {
    uint8_t writeData[] = {0b00000110, 0x00, 0xFF};
    
    for (int ch = 0; ch < 4; ch++) {
    switch(ch){
      case 0:
        writeData[1] = 0b00000000; 
      break;

      case 1:
        writeData[1] = 0b01000000; 
      break;
      case 2:
        writeData[1] = 0b10000000; 
      break;
      case 3:
        writeData[1] = 0b11000000; 
      break;
    
    }

        // CSアサート
        *(volatile uint32_t *)(spi2->regbase + DW_SPI_SER) = 1 << 0;  //CSnをLOWに


    //データを送りきる!(データは送信順に溜まっていく)
    for(int i = 0; i < 3; i++) {
         while (!(*(volatile uint32_t *)(spi2->regbase + DW_SPI_SR) & DW_SPI_SR_TF_NOT_FULL)) {;} // 送信FIFOが空き待ち
        *(volatile uint8_t *)(spi2->regbase + DW_SPI_DR) = writeData[i];                                // データ送信

    }   
    //データを読みまくる(溜まったものを順に) 
    for (int i = 0; i < 3; i++) {
         while (!(*(volatile uint32_t *)(spi2->regbase + DW_SPI_SR) & DW_SPI_SR_RF_NOT_EMPT)) {;} //受信データを待つ
        recieveData[i] = *(volatile uint8_t *)(spi2->regbase + DW_SPI_DR);  
    }                              //1byte受信

    // CSデアサート
     *(volatile uint32_t *)(spi2->regbase + DW_SPI_SER) = 0; //通信終了

    // 受信3byteのデータから12bitを組立
    uint16_t value = ((recieveData[1] & 0x0F) << 8) | recieveData[2]; //下位4bitだけ残し、8bitずらす
    // 12bitの値を電圧に変換
    double voltage = VREF * value / 4095.0; 

    // printf("CH%u: %4u / 4095  (%.3f V)", ch, value, voltage);
    printf("CH%u: %0.3f V  ", ch, voltage);



    }

    printf("\n");
    usleep(10);  // 100ms 間隔
 }
}

// SPI送信関数
void send_spi(rp1_spi_instance_t *spi, uint16_t txdata){
    
        // BUSY解除待ち
        while (*(volatile uint32_t *)(spi->regbase + DW_SPI_SR) & DW_SPI_SR_BUSY) {;}
    
        // 送信FIFOが空き待ち
        while (!(*(volatile uint32_t *)(spi->regbase + DW_SPI_SR) & DW_SPI_SR_TF_NOT_FULL)) {;}

  
    
        *(volatile uint32_t *)(spi->regbase + DW_SPI_SER) = 1 << 0; // CSアサート


    
        *(volatile uint16_t *)(spi->regbase + DW_SPI_DR) = txdata; // 16bit送信

    
        // 送信データに対応する受信データ（ゴミ）を捨てる
        while (!(*(volatile uint32_t *)(spi->regbase + DW_SPI_SR) & DW_SPI_SR_RF_NOT_EMPT)) {;}
        uint16_t rxdata = *(volatile uint16_t *)(spi->regbase + DW_SPI_DR);


    //printf("Done sending 16bit data.\n");
}

//16bitデータ
uint16_t Input_value1(float V1,float V2){
    float Vs1=(V1+12.0)*10.0;
    float Vs2=(V2+12.0)*10.0;

    uint8_t hexa1=((int)Vs1<<0)|(int)Vs1;
    uint8_t hexa2=((int)Vs2<<0)|(int)Vs2; 

    char motor12[5];
    sprintf(motor12,"%2X%2X",hexa1,hexa2);

    uint16_t value1=(uint16_t)strtol(motor12, NULL, 16);
     // printf("hexa12=%X\n",value1);

    return value1;
}

//8bitデータ
uint8_t Input_value2(float V3){
    float Vs3=(V3+12.0)*10.0;
    uint8_t value2=((int)Vs3<<0)|(int)Vs3;
    //printf("hexa3=%X\n\n",value2);
    return value2;
}
#ifdef __cplusplus
}
#endif



////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


/*


int main(void)
{

    float V1=3.0;
    float V2=3.0;
    float V3=3.0;

    uint16_t hexa12=Input_value1(V1,V2); 
    uint8_t hexa3=Input_value2(V3);
  
     rp1_t *rp1 = setup_rp1();  //rp1初期化

    rp1_spi_instance_t *spi0 = setup_spi(rp1, 0); // SPI初期化(固定,チャンネル)
    rp1_spi_instance_t *spi1 = setup_spi(rp1, 1); // SPI初期化(固定,チャンネル)
    rp1_spi_instance_t *spi2 = setup_spi(rp1, 2); //SPI初期化(固定,チャンネル)

 
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start); 

        send_spi(spi0, hexa12); // SPI0送信
        send_spi(spi1, hexa3);  // SPI1送信

    clock_gettime(CLOCK_MONOTONIC, &end);
     long seconds = end.tv_sec - start.tv_sec;
    long nanoseconds = end.tv_nsec - start.tv_nsec;
    long total_microseconds = seconds * 1000000 + nanoseconds / 1000;
            
    printf("Time taken: %ld microseconds\n", total_microseconds);


    read_ADC(spi2);  // ADC読み取り(ループ)


    return 0;
}


*/