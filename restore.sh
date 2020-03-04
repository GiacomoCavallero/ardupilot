#!/bin/bash
# if empty will use current dir
if [ $# -ge 1 ]; then
    SAVE_DIR=$1
else
    SAVE_DIR=saved
fi

mkdir -p /etc/ardupilot
mkdir -p /var/APM
mkdir -p /var/lib/ardupilot

cp $SAVE_DIR/usr/sbin/* /usr/sbin 
cp -r $SAVE_DIR/etc/ardupilot/* /etc/ardupilot
cp $SAVE_DIR/var/APM* /var/APM
cp $SAVE_DIR/var/lib/ardupilot /var/lib/ardupilot

