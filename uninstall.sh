if ! [ $(id -u) = 0 ]; then
   echo "Please run as root"
   exit 1
fi

ETC_DIR=/etc
ETC_ARDUPILOT=$ETC_DIR/ardupilot
ETC_MONIT=$ETC_DIR/monit
ETC_INIT=$ETC_DIR/init.d

sh uninstall_monit.sh

rm -rf /usr/local/bin/ardu*
rm -rf /usr/local/lib/libepos2_bridge.so
rm -rf $ETC_ARDUPILOT
rm $ETC_INIT/ardu*
ldconfig

