HOME_BOTANY_BAY=-33.98547,151.19311,1,280
HOME_JERVIS=-35.11986,150.70347,1,280
HOME_ULLUDULLA=-35.356,150.490,1,90
HOME_DARWIN=-12.394,130.763,1,90
HOME_KIOLA=-35.55563,150.38331,1,90

if [ -z "${SIM_HOME}" ]
then
	SIM_HOME=-12.394,130.763,1,90
fi
echo "SIM_HOME" $SIM_HOME

if [ -z "${SIM_SPEEDUP}" ]
then
	SIM_SPEEDUP=1
fi
echo "SIM_SPEEDUP" $SIM_SPEEDUP

SYS_ID=1

INSTANCE=0

ETC_DIR=/etc/ardupilot
EXEC=ardurover-2019-sitl
SIM_DIR=/var/lib/ardupilot/sitl/$INSTANCE

echo SIM_DIR=$SIM_DIR

# create a directory in which to run the simulator
mkdir -p $SIM_DIR

# ensure the simulator has a default set of parameters
DEFAULTS_FILE=$ETC_DIR/sim_defaults-2019.parm
SIM_DEFAULTS=$SIM_DIR/sim_defaults.parm

if [ ! -f $SIM_DEFAULTS ]
then
	echo "Copying default parameters for new instance"
	cp $DEFAULTS_FILE $SIM_DEFAULTS
	sed -i -e "s/SYSID_THISMAV,.*/SYSID_THISMAV,$SYS_ID/g" $SIM_DEFAULTS
fi

#change to the sim directory
cd $SIM_DIR

# Purge old log files
rm -rf $SIM_DIR/logs

$EXEC -M bluebottle --home $SIM_HOME --speedup $SIM_SPEEDUP --instance $INSTANCE --defaults $SIM_DEFAULTS 

