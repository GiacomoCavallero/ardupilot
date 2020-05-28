HOME_BOTANY_BAY=-33.98547,151.19311,1,280
HOME_JERVIS=-35.11986,150.70347,1,280
HOME_ULLUDULLA=-35.356,150.490,1,90
HOME_DARWIN=-12.394,130.763,1,90

if [ -z "$1" ]
then
	SYS_ID=2
else
	SYS_ID=$1
fi
echo "SYS_ID=" $SYS_ID

if [ -z "$2" ]
then
	MODEL=bluebottle
else
	MODEL=$(echo $2 | tr '[:upper:]' '[:lower:]')
fi
# boat|boat_skid

echo "MODEL=" $MODEL

if [ -z "$3" ]
then
	SIM_HOME=$HOME_BOTANY_BAY
else
	SIM_HOME=$3
fi
echo "SIM_HOME" $SIM_HOME

if [ -z "${SIM_SPEEDUP}" ]
then
	SIM_SPEEDUP=1
fi
echo "SIM_SPEEDUP" $SIM_SPEEDUP

INSTANCE=$(($SYS_ID))


EXEC_DIR=`pwd`
SIM_DIR=sitl_instances/instance_$INSTANCE

# create a directory in which to run the simulator
mkdir -p $SIM_DIR

# ensure the simulator has a default set of parameters
DEFAULTS_FILE=sitl-2019_defaults.parm
SIM_DEFAULTS=$SIM_DIR/$DEFAULTS_FILE

if [ ! -f $SIM_DEFAULTS ]
then
	echo "Copying default parameters for new instance"
	cp $DEFAULTS_FILE $SIM_DEFAULTS
	sed -i -e "s/SYSID_THISMAV,0/SYSID_THISMAV,$SYS_ID/g" $SIM_DEFAULTS
fi

#change to the sim directory
cd $SIM_DIR

# Purge old log files
rm -rf logs/

#echo "run" | gdb --args ./build/sitl/bin/ardurover -M boat
echo Starting $MODEL Instance $INSTANCE sysid $SYS_ID at $SIM_HOME.
$EXEC_DIR/build/sitl/bin/ardurover -M $MODEL --home $SIM_HOME --speedup $SIM_SPEEDUP --instance $INSTANCE --defaults $DEFAULTS_FILE
#--mmsi $SYS_ID

#cd $EXEC_DIR
