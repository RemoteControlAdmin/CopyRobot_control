#include "motor_control.hpp"

namespace motorcontrol{
    MotorControl* MotorControl::instance = nullptr; // make instans
    MotorControl::MotorControl(){
        if (!MotorControl::dummy_spi0_initialize()) {
            printf("SPI0初期化失敗\n");
            return;
        }

         // rp1_t* setup_rp1();                                    
        rp1 = setup_rp1();
        if (!rp1) { printf("RP1初期化失敗\n");             //rp1初期化
                                    return ;  }
            
        spi0 = setup_spi(rp1, 0); // SPI0初期化(固定,チャンネル)
        spi1 = setup_spi(rp1, 1); // SPI1初期化(固定,チャンネル)
        spi2 = setup_spi(rp1, 2); //SPI2初期化(固定,チャンネル)
        
        uint8_t on=0xFA;
        send_spi(spi1, on);
        
    }

     Eigen::Vector3d MotorControl::convert_wheeltovoltage(Eigen::Vector3d forwheelvelocity){
        mortor_voltage = forwheelvelocity * wwtovoltgain;

        return mortor_voltage;
    }


    /*
    *    ========= private =========
    */
    bool MotorControl::send_voltage(float V1,float V2,float V3){

        if((-12.0<=V1 && V1<=12.0)&&(-12.0<=V2 && V2<=12.0)&&(-12.0<=V3 && V3<=12.0)){

            uint16_t hexa12=Input_value1(V1,V2);
            uint8_t hexa3=Input_value2(V3);
        
            send_spi(spi0, hexa12); // SPI0送信
            send_spi(spi1, hexa3);  // SPI1送信
            
            return true;
        }
        return false;
    }

   bool MotorControl::dummy_spi0_initialize(const char *device,
                                            uint8_t mode,
                                            uint8_t bits,
                                            uint32_t speed){
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("SPI0 open failed");
        return false;
    }

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("SPI0 dummy init ioctl failed");
        close(fd);
        return false;
    }

    // ★ 1回だけ送信しておく（ここがカギ！）
    uint16_t tx = 0x0000;
    uint16_t rx = 0;
    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(&tx);
    tr.rx_buf = reinterpret_cast<unsigned long>(&rx);
    tr.len = sizeof(tx);
    tr.speed_hz = speed;
    tr.bits_per_word = bits;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI0 dummy transfer failed");
        close(fd);
        return false;
    }

    close(fd);
    return true;


    // 通信はせず，クローズ
    close(fd);
    return true;
}

    //Disable DC motor drive
    MotorControl::~MotorControl(){

        send_spi(spi1, 0xF9); // SPI1に終了信号を送信

        return;  
    }
    

    
}