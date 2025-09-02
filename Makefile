# Improved Makefile
CC       := /usr/bin/g++
CC_C     := /usr/bin/gcc
AR       := /usr/bin/ar
CFLAGS   := -std=c++20 -O3 -Wall -funroll-loops
DEFINES  := -D_REENTRANT -DAFFY
INCLUDES := -I./include -I/usr/include/eigen3
LDFLAGS  := -lpthread -lrt -lm
TARGET   := firstcheck

# ソースファイル
CPPSRCS  := src/forward_kinematics.cpp src/inverce_kinematics.cpp src/main.cpp src/motor_control.cpp \
            src/robot_control.cpp src/robot_data_cal.cpp src/udp_connect.cpp src/force_get.cpp src/csv_edit.cpp src/common.cpp\
			src/data_logger.cpp

CSRCS    := src/rp1-spi-util.c src/rp1-spi.c src/rpi5-rp1-spi.c

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
	-rm -f $(TARGET) src/*.o
