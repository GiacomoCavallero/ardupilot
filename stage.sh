#!/bin/bash
SRC_DIR=$PWD
BUILD_DIR=$SRC_DIR/build

VERSION=2019
# if empty will use current dir
if [ $# -ge 1 ]; then
    STAGE_DIR=$1
else
    STAGE_DIR=$SRC_DIR/stage
fi

STAGE_BIN=$STAGE_DIR/bin
STAGE_LIB=$STAGE_DIR/lib

STAGE_ETC=$STAGE_DIR/etc
STAGE_ETC_ARDUPILOT=$STAGE_ETC/ardupilot
STAGE_MONIT=$STAGE_ETC/monit

mkdir -p $STAGE_BIN
mkdir -p $STAGE_LIB
mkdir -p $STAGE_ETC_ARDUPILOT

###############################################################################
# executables
###############################################################################
echo "copying executables"
if [ -e "$BUILD_DIR/navio2/bin/ardurover" ]; then
    cp $BUILD_DIR/navio2/bin/ardurover $STAGE_BIN/ardurover-$VERSION
fi
if [ -e "$BUILD_DIR/sitl/bin/ardurover" ]; then
    cp $BUILD_DIR/sitl/bin/ardurover $STAGE_BIN/ardurover-$VERSION-sitl
fi

###############################################################################
# libs
###############################################################################
echo "copying libs"
if [ -e "$SRC_DIR/modules/epos2_bridge/libepos2_bridge.so" ]; then
    cp $SRC_DIR/modules/epos2_bridge/libepos2_bridge.so $STAGE_LIB
fi

###############################################################################
# config files
###############################################################################
echo "copying start scripts"
cp $SRC_DIR/go_* $STAGE_ETC_ARDUPILOT
cp $SRC_DIR/*_defaults.parm $STAGE_ETC_ARDUPILOT

###############################################################################
# monit oc_service
###############################################################################
echo "copying config for oc_service monit service"
cp -f $SRC_DIR/ardupilot.init $STAGE_ETC_ARDUPILOT
mkdir -p $STAGE_MONIT/conf.d
cp $SRC_DIR/ardupilot.monit $STAGE_ETC/monit/conf.d/

###############################################################################
# install scripts
###############################################################################
cp -f $SRC_DIR/install.sh $STAGE_DIR
cp -f $SRC_DIR/uninstall.sh $STAGE_DIR
cp -f $SRC_DIR/uninstall_monit.sh $STAGE_DIR
cp -f $SRC_DIR/makepkg.sh $STAGE_DIR
cp -f $SRC_DIR/save.sh $STAGE_DIR
cp -f $SRC_DIR/restore.sh $STAGE_DIR

echo "Done"
exit
