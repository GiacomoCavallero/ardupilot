sudo monit stop ardupilot; ./waf rover -j 4 && ./stage.sh stage && sudo ./install.sh stage $1
