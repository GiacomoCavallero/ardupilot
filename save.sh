#!/bin/bash
if [ $# -ge 1 ]; then
    SAVE_DIR=$1
else
    SAVE_DIR=saved
fi

mkdir -p $SAVE_DIR/bin
mkdir -p $SAVE_DIR/etc/ardupilot
mkdir -p $SAVE_DIR/var/APM
mkdir -p $SAVE_DIR/var/lib/ardupilot

cp /usr/local/bin/ardupilot* $SAVE_DIR/bin
cp -r /etc/ardupilot/* $SAVE_DIR/etc/ardupilot
cp /var/APM/*.stg $SAVE_DIR/var/APM
cp /var/lib/ardupilot/*.stg $SAVE_DIR/var/lib/ardupilot

