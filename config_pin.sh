#!/bin/sh
# /dev/null 2>& is a preventing any output from being displayed on the terminal.
config-pin P9_22 pwm > /dev/null 2>&1
config-pin P9_21 pwm > /dev/null 2>&1
config-pin P9_16 pwm > /dev/null 2>&1
config-pin P9_14 pwm > /dev/null 2>&1
config-pin P8_19 pwm > /dev/null 2>&1
config-pin P8_13 pwm > /dev/null 2>&1

sudo echo 0 > /sys/class/pwm/pwmchip1/unexport
sudo echo 1 > /sys/class/pwm/pwmchip1/unexport
sudo echo 0 > /sys/class/pwm/pwmchip4/unexport
sudo echo 1 > /sys/class/pwm/pwmchip4/unexport
sudo echo 0 > /sys/class/pwm/pwmchip7/unexport
sudo echo 1 > /sys/class/pwm/pwmchip7/unexport

sudo echo 0 > /sys/class/pwm/pwmchip1/export
sudo echo 1 > /sys/class/pwm/pwmchip1/export
sudo echo 0 > /sys/class/pwm/pwmchip4/export
sudo echo 1 > /sys/class/pwm/pwmchip4/export
sudo echo 0 > /sys/class/pwm/pwmchip7/export
sudo echo 1 > /sys/class/pwm/pwmchip7/export

exit 0