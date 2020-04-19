#!/bin/bash
PKGDIR=$PWD
ARCH=`uname --m`
mkdir -p $HOME/install/ardupilot && \
cd $HOME/install/ardupilot && \
tar xvf $PKGDIR/ardupilot-$ARCH-$GLIBC_VER.tgz && \
cd --
