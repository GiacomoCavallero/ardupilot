#!/bin/bash
PKGDIR=$PWD
ARCH=`uname --m`
mkdir -p /tmp/install/ardupilot && cd /tmp/install/ardupilot && tar xvf $PKGDIR/ardupilot-$ARCH.tgz
sudo sh install.sh
cd --
