#!/bin/bash
if ! [ $(id -u) = 0 ]; then
	echo "Please run as root"
	exit 1
fi

# if empty will use current dir
if [ $# -ge 1 ]; then
	STAGE_DIR=$1
else
	STAGE_DIR=.
    
	if ls ./stage/bin/ardu* > /dev/null 2>&1; then
		STAGE_DIR=stage
	else
		ls ./bin/ardu*
		if ls ./bin/ardu* > /dev/null 2>&1; then
			STAGE_DIR=.
		else
			echo "ERROR: Can't find a default folder."
			exit 1
		fi
	fi
	echo "STAGE_DIR defaulting to $STAGE_DIR"
fi

if [ $# -ge 2 ]; then
	VARIANT=$2
else
    VARIANT=navio2-2019
fi
if [[ $VARIANT =~ sitl.* ]]; then
    echo Is SITL
    SITL=1
fi

if [ $# -ge 3 ]; then
	SYS_ID=$3
fi

if [ $# -ge 4 ]; then
	SIM_HOME=$4
fi

STAGE_BIN=$STAGE_DIR/bin
STAGE_LIB=$STAGE_DIR/lib
STAGE_ETC=$STAGE_DIR/etc
STAGE_ETC_ARDUPILOT=$STAGE_ETC/ardupilot

INSTALL_BIN=/usr/local/bin
INSTALL_LIB=/usr/local/lib
INSTALL_ETC=/etc
INSTALL_ETC_ARDUPILOT=$INSTALL_ETC/ardupilot
INSTALL_MONIT=$INSTALL_ETC/monit
INSTALL_VAR=/var/APM

if  ! ls $STAGE_BIN/ardu* > /dev/null 2>&1; then
  echo "Does't appear to be a valid install"
  echo "$STAGE_BIN/ardurover"
  exit 1
fi

mkdir -p $INSTALL_BIN
mkdir -p $INSTALL_LIB
mkdir -p $INSTALL_ETC_ARDUPILOT
mkdir -p $INSTALL_VAR

###############################################################################
# binaries
###############################################################################
echo "copying binaries"
cp $STAGE_BIN/* $INSTALL_BIN
cp $STAGE_LIB/* $INSTALL_LIB
ldconfig

###############################################################################
# config files
###############################################################################
echo "copying start script"
RUN_SCRIPT="go_$VARIANT.sh"
if [ ! -e "$INSTALL_ETC_ARDUPILOT/$RUN_SCRIPT" ]; then
    cp "$STAGE_ETC_ARDUPILOT/$RUN_SCRIPT" $INSTALL_ETC_ARDUPILOT/$RUN_SCRIPT
fi

if [ -n "$SITL" ]; then
    echo "set up sitl"
    echo "$STAGE_ETC_ARDUPILOT"/"$VARIANT"_defaults.parm $INSTALL_ETC_ARDUPILOT/"$VARIANT"_defaults.parm

    if [ ! -e $INSTALL_ETC_ARDUPILOT/"$VARIANT"_defaults.parm ]; then
        cp "$STAGE_ETC_ARDUPILOT"/"$VARIANT"_defaults.parm $INSTALL_ETC_ARDUPILOT/"$VARIANT"_defaults.parm
    fi
    if [ -n "$SYS_ID" ] && [ -e $INSTALL_ETC_ARDUPILOT/go_"$VARIANT".sh ]; then
        sed -i -e "s/SYS_ID=.*/SYS_ID=$SYS_ID/g" $INSTALL_ETC_ARDUPILOT/go_"$VARIANT".sh
    fi
    if [ -n "$SIM_HOME" ] && [ -e $INSTALL_ETC_ARDUPILOT/go_"$VARIANT".sh ]; then
        sed -i -e "s/SIM_HOME=.*/SIM_HOME=$SIM_HOME/g" $INSTALL_ETC_ARDUPILOT/go_"$VARIANT".sh
    fi
fi

###############################################################################
# monit oc_service
###############################################################################
echo "copying config for oc_service monit service"
if [ ! -f $INSTALL_ETC_ARDUPILOT/ardupilot.init ]; then
	cp $STAGE_ETC_ARDUPILOT/ardupilot.init $INSTALL_ETC_ARDUPILOT
fi

if [ -n "$VARIANT" ]; then
    sed -i -e "s/^SCRIPT=\/etc\/ardupilot\/go_.*/SCRIPT=\/etc\/ardupilot\/go_$VARIANT.sh/" $INSTALL_ETC_ARDUPILOT/ardupilot.init
fi
mkdir -p $INSTALL_MONIT/conf.d
cp $STAGE_ETC/monit/conf.d/ardupilot.monit $INSTALL_ETC/monit/conf.d
service monit restart
echo "done"
