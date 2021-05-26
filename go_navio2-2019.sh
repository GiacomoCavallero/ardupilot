#!/bin/bash
# -A is primary telemetry should always be tcp 5760
# -B is external GPS in the case below is airmar unit
# -C is secondary telemetry in the case below is RFD900
#
# sudo ./APMrover2.elf -A udp:192.168.1.100:14550 -F /dev/ttyUSB0 -C /dev/ttyUSB1 -B /dev/ttyUSB2
ulimit -c unlimited
sudo sysctl -w kernel.core_pattern=/tmp/core-%e.%t-%p.%h

#host ip address can be supplied as parameter 1, if not supplied then will use 192.169.1.100

# make sure logical devices are set up in /etc/udev/rules.d/99-usb-serial.rules
file1="/dev/rs485"
#file2="/dev/airmar"
#file3="tcc:192.168.1.30:4660"

RS485=""
AIRMAR=""
RFD900=""

if [ -e "$file1" ]
then
        RS485=" -F ${file1}"
else
        echo "NO RS485"
fi

#if [ -e "$file2" ]
#then
#        AIRMAR=" -B  ${file2}"
#else
#        echo "NO AIRMAR"
#fi
#AIRMAR=" -B tcc:192.168.1.5:100"
AIRMAR=" -F tcc:127.0.0.1:7799"

#if [ -e "$file3" ]
#then
#        RFD900=" -C ${file3}"
#        RFD900=" -C /dev/ttyUSB1"
#else
#        echo "NO RFD900"
#fi

sudo ardurover-2019 -A tcp:0.0.0.0:5760 $RFD900 $AIRMAR 
