# mode:-*-makefile-*-
# - compile forcecontrol as static executable
# - test utility can be compiled separately
# (c) mobalean LLC

CC = /usr/bin/g++
AR = /usr/bin/ar
OPTS = -Wall
DEFINES= _REENTRANT -DAFFY
CFLAGS = -O3 -marm -funroll-loops -march=armv7-a -mtune=cortex-a8 -mfpu=neon -D$(DEFINES)
LDFLAGS = -lpthread -lrt -lm
INCLUDES =

USER-OBJS-COM1 = 03-Unilateral_Compensation.o  udp_connect.o
USER-EXEC-COM1 = a.out

all:$(USER-EXEC-COM1)

$(USER-EXEC-COM1):$(USER-OBJS-COM1)
	$(CC) $(LDFLAGS) -o $@ $(USER-OBJS-COM1)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o a.out

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<