if ! [ $(id -u) = 0 ]; then
   echo "Please run as root"
   exit 1
fi

sh uninstall_monit.sh

rm -rf /usr/local/bin/ardu*
rm -rf /usr/local/lib/libepos2_bridge.so
rm -rf /etc/ardupilot
rm -rf /var/lib/ardupilot
ldconfig

echo Done
