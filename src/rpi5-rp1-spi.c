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
#include "rpi5-rp1-spi.h"

#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
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


// ==== MCP3208用 設定（直叩き読み取りに適用） ====
// VREFは既存の #define REF_VOLTAGE 3.3 を使用

#define REF_VOLTAGE 3.3
#define SCALE 15.5038   // 電圧→力のスケール係数 α





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
rp1_spi_instance_t* setup_spi(rp1_t *rp1, int ch,
                              int mode, int data_bits,
                              int baud_div, int frf)
{
    rp1_spi_instance_t *spi;
    if (!rp1_spi_create(rp1, ch, &spi)) {
        printf("unable to create spi\n");
        return NULL;
    }

    // ピン
    switch (ch) {
    case 0: setup_spi0_pins(rp1); break;
    case 1: setup_spi1_pins(rp1); break;
    case 2: setup_spi2_pins(rp1); break;
    case 3: setup_spi3_pins(rp1); break;
    default: return NULL;
    }

    // 既定値
    if (data_bits <= 0) data_bits = (ch == 0) ? 16 : 8;
    if (baud_div  <= 0) baud_div  = (ch == 0) ? 200 : 100;
    if (frf       <  0) frf = 0;                  // Motorola
    if (mode < 0 || mode > 3) mode = 0;          // MODE0
    if (data_bits < 4)  data_bits = 4;
    if (data_bits > 16) data_bits = 16;
    if (baud_div & 1)   ++baud_div;              // 偶数に丸め

    // 無効化
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x0;

    // ボーレート
    *(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR) = (uint32_t)baud_div;

    // CTRLR0
    uint32_t r = *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0);

    // CPOL/CPHA（mode 0..3）
    r &= ~DW_PSSI_CTRLR0_SCPOL;
    r &= ~DW_PSSI_CTRLR0_SCPHA;
    if (mode & 0x2) r |= DW_PSSI_CTRLR0_SCPOL;
    if (mode & 0x1) r |= DW_PSSI_CTRLR0_SCPHA;

    // データ長 DFS=[20:16] に (bits-1) をセット
    r &= ~(0x1Fu << 16);
    r |=  ((uint32_t)(data_bits - 1) << 16);

    // フレームフォーマット（FRF は典型的に [5:4]）
    r &= ~(0x3u << 4);
    r |=  ((uint32_t)(frf & 0x3) << 4);

    *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0) = r;

    // 有効化
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x1;

    // デバッグ：DFSをデコード表示（確認用）
    uint32_t rchk = *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0);
    unsigned dfs = ((rchk >> 16) & 0x1F) + 1;
    printf("[spi%d] CTRLR0=0x%08X DFS=%u-bit mode=%d\n", ch, rchk, dfs, mode);

    return spi;
}



//ADC読み取り関数
// ---- 8bitフレーム×3バイトの連続転送（直叩き）----
static inline void spi_wait_tfnf(volatile uint32_t *sr) {
    while (!(*sr & DW_SPI_SR_TF_NOT_FULL)) { /* wait */ }
}
static inline void spi_wait_rne(volatile uint32_t *sr) {
    while (!(*sr & DW_SPI_SR_RF_NOT_EMPT)) { /* wait */ }
}
static inline void spi_wait_idle(volatile uint32_t *sr) {
    while (*sr & DW_SPI_SR_BUSY) { /* wait */ }
}
// ---- RX FIFO を空にする（開始前に呼ぶ）----
static inline void spi_flush_rx(volatile uint32_t *sr, volatile uint32_t *dr) {
    while (*sr & DW_SPI_SR_RF_NOT_EMPT) {
        (void)*dr; // 破棄読み
    }
}

// ---- 8bit×3の全二重転送（完了待ち＆CS制御を厳密化）----
static void mcp3208_xfer3(rp1_spi_instance_t *spi,
                          const uint8_t tx[3], uint8_t rx[3])
{
    volatile uint32_t *SR  = (volatile uint32_t *)(spi->regbase + DW_SPI_SR);
    volatile uint32_t *DR  = (volatile uint32_t *)(spi->regbase + DW_SPI_DR);
    volatile uint32_t *SER = (volatile uint32_t *)(spi->regbase + DW_SPI_SER);

    // 受信残りをクリア
    spi_flush_rx(SR, DR);

    // CS Low（CS0使用想定）
    *SER = (1u << 0);

    // 送信3バイト（DRはワードアクセス推奨：下位8bitのみ有効）
    for (int i = 0; i < 3; ++i) {
        spi_wait_tfnf(SR);
        *(volatile uint32_t *)DR = (uint32_t)tx[i];
    }

    // 受信3バイト
    for (int i = 0; i < 3; ++i) {
        spi_wait_rne(SR);
        rx[i] = (uint8_t)(*(volatile uint32_t *)DR & 0xFF);
    }

    // 送受信の完全終了を待ってから CS High
    spi_wait_idle(SR);
    *SER = 0;
}

// ---- 1ch読み取り（12bit組み立て）----
static int mcp3208_read_ch_reg(rp1_spi_instance_t *spi, uint8_t ch)
{
    if (ch > 7) return -1;

    // ここで（必要なら）SPI2の DFS=8bit, CPOL/CPHA=0, BAUDR=1MHz 等を
    // “SPI2のCTRLR0/BAUDRのみ”に設定し，終了時に元へ戻すのが安全

    uint8_t tx[3];
    tx[0] = (uint8_t)(0x06 | ((ch & 0x04) >> 2)); // Start=1, SGL=1, D2
    tx[1] = (uint8_t)((ch & 0x03) << 6);          // D1 D0 を上位2bitへ
    tx[2] = 0x00;

    uint8_t rx[3] = {0,0,0};
    mcp3208_xfer3(spi, tx, rx);

    // 12bit: [rx1下位4bit][rx2]
    int value = ((rx[1] & 0x0F) << 8) | rx[2];
    return value; // 0..4095
}



// ---- 変換ユーティリティ ----
static inline double adc_to_voltage(int adc) {
    return (adc / 4095.0) * (double)REF_VOLTAGE;
}
static inline double force_from_adc(int adc, double beta) {
    return (double)SCALE * adc_to_voltage(adc) + beta;
}

// ---- 自動ゼロ校正（ゼロ荷重・静止状態で実行）----
// β_i = -SCALE * V_{i,0}
void auto_zero_calib_reg(rp1_spi_instance_t *spi,
                         double *b_fax, double *b_fay,
                         double *b_fbx, double *b_fby,
                         int samples)
{
    if (samples <= 0) {
        fprintf(stderr, "[ZERO] samples <= 0\n");
        return;
    }
    if (!b_fax || !b_fay || !b_fbx || !b_fby) {
        fprintf(stderr, "[ZERO] null beta pointer\n");
        return;
    }

    int64_t sum_adc[4] = {0,0,0,0};
    int     cnt[4]     = {0,0,0,0};

    for (int n = 0; n < samples; ++n) {
        for (int ch = 0; ch < 4; ++ch) {
            int v = mcp3208_read_ch_reg(spi, (uint8_t)ch);
            if (v < 0) {
                // 失敗は平均から除外（ログのみ）
                fprintf(stderr, "[ZERO] CH%d 読み取り失敗 (n=%d)\n", ch, n);
                continue;
            }
            // 念のためクランプ（0..4095）
            if (v < 0) v = 0;
            else if (v > 4095) v = 4095;

            sum_adc[ch] += v;
            cnt[ch]     += 1;
        }
        usleep(1000);
    }

    // 有効サンプル確認
    for (int ch = 0; ch < 4; ++ch) {
        if (cnt[ch] == 0) {
            fprintf(stderr, "[ZERO] CH%d 有効サンプル0：校正を中止\n", ch);
            return;
        }
    }

    // 平均ADCをdoubleで保持 → 電圧へ
    const double a0 = (double)sum_adc[0] / (double)cnt[0];
    const double a1 = (double)sum_adc[1] / (double)cnt[1];
    const double a2 = (double)sum_adc[2] / (double)cnt[2];
    const double a3 = (double)sum_adc[3] / (double)cnt[3];

    // adc_to_voltage(double) が無ければ以下を使用：
    // #define REF_VOLTAGE 3.3
    // static inline double adc_to_voltage(double adc) { return (adc / 4095.0) * REF_VOLTAGE; }

    const double v0 = adc_to_voltage(a0);
    const double v1 = adc_to_voltage(a1);
    const double v2 = adc_to_voltage(a2);
    const double v3 = adc_to_voltage(a3);

    // β_i = -SCALE * V_{i,0}
    *b_fax = -SCALE * v0;
    *b_fay = -SCALE * v1;
    *b_fbx = -SCALE * v2;
    *b_fby = -SCALE * v3;

    //printf("[ZERO] V0..3=[%.4f %.4f %.4f %.4f] -> "
    //      "B=[fax=%.4f fay=%.4f fbx=%.4f fby=%.4f]\n",
    //       v0, v1, v2, v3, *b_fax, *b_fay, *b_fbx, *b_fby);
}

//ADC読み取り関数（spidevに準拠）
// ADC読み取り関数（直叩きのまま，設定・力計算・ゼロ校正を移植）

mcp_result_t read_ADC(rp1_spi_instance_t *spi2, double b_fax, double b_fay, double b_fbx, double b_fby){
    mcp_result_t result = {0};

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        for (int ch = 0; ch < 4; ++ch) {
            int v = mcp3208_read_ch_reg(spi2, (uint8_t)ch);
            if (v < 0) {
                fprintf(stderr, "CH%d 読み取り失敗\n", ch);
                return result;
            }
            result.raw[ch]  = v;
            result.volt[ch] = adc_to_voltage(v);
        }

        result.force[0] = force_from_adc(result.raw[0], b_fax);
        result.force[1] = force_from_adc(result.raw[1], b_fay);
        result.force[2] = force_from_adc(result.raw[2], b_fbx);
        result.force[3] = force_from_adc(result.raw[3], b_fby);

        clock_gettime(CLOCK_MONOTONIC, &t1);
        long dt_us = (t1.tv_sec - t0.tv_sec) * 1000000L
                   + (t1.tv_nsec - t0.tv_nsec) / 1000L;

        // 既存の出力体裁を極力維持
        //printf("| time=%ld us\n", dt_us);
    
    return result;
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