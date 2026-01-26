#!/bin/bash
echo "Setup unitree ros2 environment"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /opt/ros/foxy/setup.bash
source "$SCRIPT_DIR/cyclonedds_ws/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Auto-detect network interface
if ip link show eth0 &>/dev/null; then
    NETWORK_INTERFACE="eth0"
elif ip link show enp0s31f6 &>/dev/null; then
    NETWORK_INTERFACE="enp0s31f6"
else
    NETWORK_INTERFACE="lo"
fi

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name=\"$NETWORK_INTERFACE\" priority=\"default\" multicast=\"default\" />
                            </Interfaces></General><Discovery>
                        </Discovery></Domain></CycloneDDS>"
echo "Using network interface: $NETWORK_INTERFACE"

# enable intel realsense permission
#sudo chmod 666 /dev/bus/usb/$(lsusb | grep -i intel | awk '{print $2"/"$4}' | sed 's/://')
