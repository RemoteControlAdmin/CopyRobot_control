# Improved Makefile
# - compile forcecontrol as static executable
# - test utility can be compiled separately
# (c) mobalean LLC

CC       := /usr/bin/g++
AR       := /usr/bin/ar
CFLAGS   := -O3 -Wall -marm -funroll-loops -march=armv7-a -mtune=cortex-a8 -mfpu=neon
DEFINES  := -D_REENTRANT -DAFFY
INCLUDES := -I./include -I/usr/include/eigen3
LDFLAGS  := -lpthread -lrt -lm
TARGET   := forcecontrol

# ソースファイル (必要に応じて追加)
SRCS     :=  src/forward_kinematics.cpp src/inverce_kinematics.cpp src/main.cpp src/motor_control.cpp src/robot_control.cpp src/robot_data_cal.cpp src/udp_connect.cpp src/force_get.cpp src/csv_edit.cpp src/common.cpp

# オブジェクトファイル (自動生成)
OBJS     := $(SRCS:.cpp=.o)

# デフォルトターゲット
all: $(TARGET)

# リンク処理
$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(TARGET)

# コンパイル処理
%.o: %.cpp
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -c -o $@ $<

# クリーン処理
clean:
	-rm -f $(TARGET) *.o *.elf *.gdb

.PHONY: all clean
