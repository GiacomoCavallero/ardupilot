sudo monit stop ardupilot
sudo sh uninstall.sh
mkdir -p ~/ardupilot/install
cd ~/oc_usv/install
scp user@10.42.0.225:src/ardupilot/ardupilot.tgz .
tar xf ardupilot.tgz sudo
sudo sh install.sh
