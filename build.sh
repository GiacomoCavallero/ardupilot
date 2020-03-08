sudo monit stop ardupilot; ./waf rover -j 4 && sh stage.sh stage && sudo sh install.sh stage $*
