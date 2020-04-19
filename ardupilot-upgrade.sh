#!/bin/bash
ARCH=`uname --m`
if [ -n "$PKG_HOST" ]; then
    PKG_HOST=peter@54.252.133.18
fi

scp $PKG_HOST:/pkg/ardupilot-$ARCH.tgz .
sudo monit stop ardupilot
./ardupilot-unpack.sh
./ardupilot-install.sh
sudo monit start ardupilot
