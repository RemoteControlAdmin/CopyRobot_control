# Improved Makefile
CC       := /usr/bin/g++
CC_C     := /usr/bin/gcc
AR       := /usr/bin/ar
CFLAGS   := -std=c++20 -O3 -Wall -funroll-loops
DEFINES  := -D_REENTRANT -DAFFY
INCLUDES := -I./include -I/usr/include/eigen3
LDFLAGS  := -lpthread -lrt -lm
TARGET   := test

# source files
CPPSRCS  := src/robot/forward_kinematics.cpp src/robot/inverce_kinematics.cpp src/robot/motor_control.cpp\
            src/robot/robot_control.cpp src/robot/robot_data_cal.cpp src/robot/force_get.cpp src/robot/force_cal.cpp\
			src/data/csv_edit.cpp src/data/data_logger.cpp\
			src/stability/energy_cal.cpp\
			src/net/udp_connect.cpp\
			src/spi/spi_service.cpp\
			src/utils/deque_manager.cpp src/utils/cpu_manager.cpp src/utils/delaytime_cal.cpp\
			src/utils/cycle_timer.cpp src/utils/commandline.cpp\
			src/common.cpp src/rlsarpmin.cpp \
			src/estimation/poskalman.cpp \
			src/main.cpp
# test sources
#CPPSRCS  := src/spi/spi_service.cpp \
			src/check_main.cpp \


CSRCS    := src/spi/rp1-spi-util.c src/spi/rp1-spi.c src/spi/rpi5-rp1-spi.c

# オブジェクトファイル
OBJS     := $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o)

# デフォルトターゲット
all: $(TARGET)

# リンク処理
$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(TARGET)

# C++ コンパイル
%.o: %.cpp
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -c -o $@ $<

# C コンパイル
%.o: %.c
	$(CC_C) -O3 -Wall $(DEFINES) $(INCLUDES) -c -o $@ $<

# クリーン処理
clean:
	-rm -f $(TARGET) src/*.o src/robot/*.o src/data/*.o src/net/*.o src/spi/*.o src/utils/*.o src/stability/*.o
