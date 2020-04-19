cd modules/epos2_bridge/modules/canfestival-3-ocius && \
./configure_ardupilot.sh && \
make canfestival driver && \
cd ../.. && \
make && \
cd ../.. && \
sudo cp modules/epos2_bridge/libepos2_bridge.so /usr/local/lib/libepos2_bridge.so
#./waf configure --board=sitl && ./waf rover -j4 && \
./waf configure --board=navio2 && ./waf rover -j4 && \

