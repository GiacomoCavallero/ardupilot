#!/bin/bash
ARCH=`uname --m`
if [ -n "$PKG_HOST" ]; then
    PKG_HOST=pi@10.42.83.42
fi

scp $PKG_HOST:/pkg/ardupilot-$ARCH.tgz .
sudo monit stop ardupilot
sudo ./ardupilot-inst.sh
sudo monit start ardupilot
