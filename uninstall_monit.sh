if ! [ $(id -u) = 0 ]; then
   echo "Please run as root"
   exit 1
fi

ETC_DIR=/etc
ETC_ARDUPILOT=$ETC_DIR/ardupilot
ETC_MONIT=$ETC_DIR/monit

monit stop ardupilot
sleep 1
sudo pkill -9 rover
rm $ETC_MONIT/conf.d/ardupilot*
service monit restart

