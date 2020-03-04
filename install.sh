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
	echo STAGE_DIR defaulting to $STAGE_DIR
fi

if [ $# -ge 2 ]; then
	SYS_ID=$2
else
    SYS_ID=1
fi

if [ $# -ge 3 ]; then
	SIM_HOME=$3
else
    SIM_HOME=-12.394,130.763,1,90 
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
  echo $STAGE_BIN/ardurover
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
echo "copying config files"
CONFIG_FILES='sim_defaults-2017.parm  sim_defaults-2019.parm go_sim-2017.sh go_navio2-2017.sh go_sim-2019.sh go_navio2-2019.sh'
for config in $CONFIG_FILES ; do
	if [ ! -e "$INSTALL_ETC_ARDUPILOT/$config" ]; then
		cp "$STAGE_ETC_ARDUPILOT/$config" $INSTALL_ETC_ARDUPILOT/$config
	fi
done

sed -i -e "s/SYS_ID=.*/SYS_ID=$SYS_ID/g" $INSTALL_ETC_ARDUPILOT/go_sim-2017.sh
sed -i -e "s/SYS_ID=.*/SYS_ID=$SYS_ID/g" $INSTALL_ETC_ARDUPILOT/go_sim-2019.sh
sed -i -e "s/SIM_HOME=.*/SIM_HOME=$SIM_HOME/g" $INSTALL_ETC_ARDUPILOT/go_sim-2017.sh
sed -i -e "s/SIM_HOME=.*/SIM_HOME=$SIM_HOME/g" $INSTALL_ETC_ARDUPILOT/go_sim-2019.sh

###############################################################################
# monit oc_service
###############################################################################
echo "copying config for oc_service monit service"
mkdir -p $INSTALL_ETC/init.d
if [ ! -e $INSTALL_ETC/init.d/ardupilot.init ]; then
	cp $STAGE_ETC/init.d/ardupilot.init $INSTALL_ETC/init.d
fi
mkdir -p $INSTALL_MONIT/conf.d
cp $STAGE_ETC/monit/conf.d/ardupilot.monit $INSTALL_ETC/monit/conf.d
systemctl restart monit
echo "done"
